// fm1-esp32-unbrick - USB host for the JieLi UBOOT, on the ESP32-S3's OTG controller.
//
// Takes the place of FM-1 Transporter src/pio_host.c (MIT, Leo Kuroshita): the same mount -> descriptors ->
// MSC bulk pair -> (1.5 s) -> GET MAX LUN -> INQUIRY x30 sequence, the same Bulk-Only commands. The usb_host
// library delivers transfer callbacks from usb_host_client_handle_events(), so that runs in a task of its own
// (client_task) and the worker blocks on a semaphore per transfer.

#include "host.h"

#include <string.h>

#include "compat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "jieli_uboot.h"
#include "log.h"
#include "usb/usb_host.h"

#define FM1_VID 0x4C4A
#define FM1_PID 0x8057
#define FM1_V15_PID 0xC755
#define XFER_TIMEOUT_MS 2000
#define FIRST_CBW_DELAY_MS 1500     // macOS had the device for seconds before its first CBW
#define INQUIRY_RETRY_MS 300
#define INQUIRY_TRIES 30
#define ATTACH_STATUS_MS 2000
#define XFER_BUF 4096 + 64          // the largest READ_FLASH, rounded up to the packet size

#define CBW_SIGNATURE 0x43425355u
#define CSW_SIGNATURE 0x53425355u

static usb_host_client_handle_t client;
static usb_device_handle_t dev;
static volatile uint8_t new_addr;           // set by the client callback, taken by the worker
static volatile bool gone;
static usb_transfer_t *x_out, *x_in, *x_ctrl;
static SemaphoreHandle_t done;
static uint8_t ep_in, ep_out, msc_itf;
static uint16_t mps_in = 64;
static bool claimed, uboot_ready, is_v15;
static uint32_t cbw_tag = 1;
static uint64_t host_start_us, mount_us, last_status_us, next_inquiry_us;
static int inquiry_tries;

static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

//--------------------------------------------------------------------+
// the library's tasks
//--------------------------------------------------------------------+

static void client_cb(const usb_host_client_event_msg_t *msg, void *arg) {
    (void)arg;
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) new_addr = msg->new_dev.address;
    else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) gone = true;
}

static void daemon_task(void *arg) {
    (void)arg;
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

static void client_task(void *arg) {
    (void)arg;
    for (;;) usb_host_client_handle_events(client, portMAX_DELAY);
}

static void xfer_cb(usb_transfer_t *t) {
    xSemaphoreGive((SemaphoreHandle_t)t->context);
}

void fm1_host_start(void) {
    usb_host_config_t cfg = {.skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1};
    ESP_ERROR_CHECK(usb_host_install(&cfg));
    host_start_us = last_status_us = time_us_64();
    xTaskCreatePinnedToCore(daemon_task, "usb_daemon", 4096, NULL, 6, NULL, 0);
    usb_host_client_config_t ccfg = {
        .is_synchronous = false,
        .max_num_event_msg = 8,
        .async = {.client_event_callback = client_cb, .callback_arg = NULL},
    };
    ESP_ERROR_CHECK(usb_host_client_register(&ccfg, &client));
    done = xSemaphoreCreateBinary();
    usb_host_transfer_alloc(XFER_BUF, 0, &x_out);
    usb_host_transfer_alloc(XFER_BUF, 0, &x_in);
    usb_host_transfer_alloc(64, 0, &x_ctrl);
    xTaskCreatePinnedToCore(client_task, "usb_client", 4096, NULL, 5, NULL, 0);
}

uint64_t fm1_host_started_us(void) {
    return host_start_us;
}

//--------------------------------------------------------------------+
// blocking transfers, from the worker task
//--------------------------------------------------------------------+

static bool xfer_sync(usb_transfer_t *t, uint8_t ep, size_t n, uint32_t *actual) {
    t->device_handle = dev;
    t->bEndpointAddress = ep;
    t->num_bytes = n;
    t->callback = xfer_cb;
    t->context = done;
    t->timeout_ms = XFER_TIMEOUT_MS;
    xSemaphoreTake(done, 0);
    esp_err_t e = ep == 0 ? usb_host_transfer_submit_control(client, t) : usb_host_transfer_submit(t);
    if (e != ESP_OK) {
        dlog("HOST: EP %02X submit failed (%s)", ep, esp_err_to_name(e));
        return false;
    }
    if (xSemaphoreTake(done, pdMS_TO_TICKS(XFER_TIMEOUT_MS)) != pdTRUE) {
        dlog("HOST: EP %02X timeout", ep);
        if (ep) {
            usb_host_endpoint_halt(dev, ep);
            usb_host_endpoint_flush(dev, ep);
            xSemaphoreTake(done, pdMS_TO_TICKS(500));   // the cancelled transfer's callback
            usb_host_endpoint_clear(dev, ep);
        }
        return false;
    }
    if (actual) *actual = (uint32_t)t->actual_num_bytes;
    if (t->status != USB_TRANSFER_STATUS_COMPLETED) {
        dlog("HOST: EP %02X status %d", ep, t->status);
        if (ep && t->status == USB_TRANSFER_STATUS_STALL) usb_host_endpoint_clear(dev, ep);
        return false;
    }
    return true;
}

static bool ctrl_sync(uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint8_t *buf, uint16_t len) {
    usb_setup_packet_t *s = (usb_setup_packet_t *)x_ctrl->data_buffer;
    s->bmRequestType = type;
    s->bRequest = req;
    s->wValue = value;
    s->wIndex = index;
    s->wLength = len;
    if (!(type & 0x80) && len) memcpy(x_ctrl->data_buffer + 8, buf, len);
    uint32_t got = 0;
    if (!xfer_sync(x_ctrl, 0, 8 + len, &got)) return false;
    if ((type & 0x80) && len) memcpy(buf, x_ctrl->data_buffer + 8, got > 8 ? got - 8 : 0);
    return true;
}

// One Bulk-Only Transport command. len == 0 means no data stage.
static bool bot_cmd(const uint8_t *cdb, uint8_t cdb_len, bool in, void *buf, uint16_t len, uint32_t *got) {
    uint8_t *c = x_out->data_buffer;
    uint32_t tag = cbw_tag++;
    memset(c, 0, 31);
    put_le32(c, CBW_SIGNATURE);
    put_le32(c + 4, tag);
    put_le32(c + 8, len);
    c[12] = in ? 0x80 : 0;
    c[13] = 0;                      // LUN
    c[14] = cdb_len;
    memcpy(c + 15, cdb, cdb_len);
    if (!xfer_sync(x_out, ep_out, 31, NULL)) {
        dlog("HOST: CBW not accepted");
        return false;
    }
    if (len) {
        if (in) {
            uint32_t n = 0;
            if (!xfer_sync(x_in, ep_in, usb_round_up_to_mps(len, mps_in), &n)) return false;
            if (n > len) n = len;
            memcpy(buf, x_in->data_buffer, n);
            if (got) *got = n;
        } else {
            memcpy(x_out->data_buffer, buf, len);
            if (!xfer_sync(x_out, ep_out, len, NULL)) return false;
        }
    }
    uint32_t n = 0;
    if (!xfer_sync(x_in, ep_in, mps_in, &n) || n < 13) {
        dlog("HOST: no CSW (%lu bytes)", (unsigned long)n);
        return false;
    }
    const uint8_t *s = x_in->data_buffer;
    if (get_le32(s) != CSW_SIGNATURE || get_le32(s + 4) != tag) {
        dlog("HOST: bad CSW sig=%08lx tag=%lu", (unsigned long)get_le32(s), (unsigned long)get_le32(s + 4));
        return false;
    }
    if (s[12] != 0) {
        dlog("HOST: CSW status %u residue %lu", s[12], (unsigned long)get_le32(s + 8));
        return false;
    }
    return true;
}

bool fm1_host_bot(const uint8_t *cdb, uint8_t cdb_len, bool in, void *buf, uint16_t len, uint32_t *got) {
    if (!dev || !uboot_ready) return false;
    return bot_cmd(cdb, cdb_len, in, buf, len, got);
}

bool fm1_host_uboot_ready(void) {
    return dev && uboot_ready;
}

bool fm1_host_v15_mounted(void) {
    return dev && is_v15;
}

bool fm1_host_softkey(void) {
    return false;
}

//--------------------------------------------------------------------+
// mount, probe
//--------------------------------------------------------------------+

static bool probe_descriptors(void) {
    const usb_device_desc_t *dd;
    if (usb_host_get_device_descriptor(dev, &dd) != ESP_OK) {
        dlog("HOST: device descriptor read failed");
        return false;
    }
    dlog("DEVICE DESCRIPTOR OK");
    dlog("  VID=%04X PID=%04X bcdDevice=%04X bcdUSB=%04X", dd->idVendor, dd->idProduct, dd->bcdDevice, dd->bcdUSB);
    is_v15 = dd->idVendor == FM1_VID && dd->idProduct == FM1_V15_PID;
    if (is_v15) dlog("  stock FM-1 application: no soft key on this board; power-cycle the FM-1 under the key");
    else if (dd->idVendor != FM1_VID || dd->idProduct != FM1_PID)
        dlog("  not the JieLi UBOOT (%04X:%04X expected)", FM1_VID, FM1_PID);

    const usb_config_desc_t *cfg;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK) {
        dlog("HOST: configuration descriptor read failed");
        return false;
    }
    const uint8_t *p = (const uint8_t *)cfg;
    uint16_t total = cfg->wTotalLength;
    dlog("CONFIGURATION DESCRIPTOR (%u bytes), bConfigurationValue=%u", total, cfg->bConfigurationValue);
    bool in_msc = false;
    ep_in = ep_out = 0;
    for (uint16_t off = 0; off + 2 <= total && p[off] >= 2; off += p[off]) {
        const uint8_t *d = &p[off];
        if (d[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *itf = (const void *)d;
            dlog("  interface %u alt %u: class=%02X/%02X/%02X endpoints=%u", itf->bInterfaceNumber,
                 itf->bAlternateSetting, itf->bInterfaceClass, itf->bInterfaceSubClass, itf->bInterfaceProtocol,
                 itf->bNumEndpoints);
            in_msc = itf->bInterfaceClass == USB_CLASS_MASS_STORAGE && itf->bAlternateSetting == 0;
            if (in_msc) msc_itf = itf->bInterfaceNumber;
        } else if (d[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const usb_ep_desc_t *ep = (const void *)d;
            dlog("    endpoint %02X attr=%02X maxpacket=%u", ep->bEndpointAddress, ep->bmAttributes,
                 USB_EP_DESC_GET_MPS(ep));
            if (in_msc && (ep->bmAttributes & 3) == USB_BM_ATTRIBUTES_XFER_BULK) {
                if (ep->bEndpointAddress & 0x80) {
                    ep_in = ep->bEndpointAddress;
                    mps_in = USB_EP_DESC_GET_MPS(ep);
                } else {
                    ep_out = ep->bEndpointAddress;
                }
            }
        }
    }
    if (!ep_in || !ep_out) {
        dlog("HOST: no MSC bulk IN/OUT pair found");
        return false;
    }
    esp_err_t e = usb_host_interface_claim(client, dev, msc_itf, 0);
    if (e != ESP_OK) {
        dlog("HOST: claiming interface %u failed (%s)", msc_itf, esp_err_to_name(e));
        return false;
    }
    claimed = true;
    dlog("HOST: MSC bulk OUT=%02X IN=%02X (max packet %u)", ep_out, ep_in, mps_in);
    return true;
}

static bool probe_inquiry(void) {
    static const uint8_t inquiry[6] = {0x12, 0, 0, 0, 36, 0};
    uint8_t data[36];
    uint32_t got = 0;
    if (!bot_cmd(inquiry, sizeof(inquiry), true, data, 36, &got) || got < 36) {
        dlog("INQUIRY failed (got %lu bytes)", (unsigned long)got);
        return false;
    }
    char vendor[9], product[17], rev[5];
    memcpy(vendor, &data[8], 8);
    vendor[8] = 0;
    memcpy(product, &data[16], 16);
    product[16] = 0;
    memcpy(rev, &data[32], 4);
    rev[4] = 0;
    dlog("INQUIRY OK: vendor=\"%s\" product=\"%s\" rev=\"%s\"", vendor, product, rev);
    return true;
}

static void unmount(void) {
    dlog("HOST: device detached (%.1f ms after mount)", (time_us_64() - mount_us) / 1000.0);
    if (claimed) usb_host_interface_release(client, dev, msc_itf);
    usb_host_device_close(client, dev);
    dev = NULL;
    claimed = uboot_ready = is_v15 = false;
    inquiry_tries = 0;
    jl_reset();
}

void fm1_host_task(void) {
    if (!host_start_us) return;
    uint64_t now = time_us_64();

    if (gone && dev) {
        gone = false;
        unmount();
    }
    uint8_t a = new_addr;
    if (a && !dev) {
        new_addr = 0;
        gone = false;
        if (usb_host_device_open(client, a, &dev) != ESP_OK) {
            dlog("HOST: opening device %u failed", a);
            dev = NULL;
        } else {
            mount_us = now;
            dlog("HOST: device mounted, addr=%u (%.1f ms after host start)", a, (now - host_start_us) / 1000.0);
            jl_reset();
            if (probe_descriptors()) {
                inquiry_tries = INQUIRY_TRIES;
                next_inquiry_us = mount_us + FIRST_CBW_DELAY_MS * 1000ull;
                dlog("HOST: first CBW in %d ms", FIRST_CBW_DELAY_MS);
            }
        }
    }

    if (inquiry_tries > 0 && dev && now >= next_inquiry_us) {
        if (inquiry_tries == INQUIRY_TRIES) {
            uint8_t lun = 0;
            if (ctrl_sync(0xA1, 0xFE, 0, msc_itf, &lun, 1)) dlog("GET MAX LUN: %u", lun);
        }
        inquiry_tries--;
        dlog("INQUIRY attempt %d (%.1f ms after mount)", INQUIRY_TRIES - inquiry_tries,
             (time_us_64() - mount_us) / 1000.0);
        if (probe_inquiry()) {
            inquiry_tries = 0;
            uboot_ready = true;
            dlog("UBOOT READY: enumerated and answered INQUIRY.");
        } else if (inquiry_tries == 0) {
            dlog("UBOOT: INQUIRY never accepted.");
        } else {
            next_inquiry_us = time_us_64() + INQUIRY_RETRY_MS * 1000ull;
        }
    }

    if (!dev && now - last_status_us > ATTACH_STATUS_MS * 1000ull) {
        dlog("HOST: no device mounted yet (%.1f s after host start)", (now - host_start_us) / 1e6);
        last_status_us = now;
    }
}
