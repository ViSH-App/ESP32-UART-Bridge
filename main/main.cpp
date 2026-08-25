#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <atomic>

#include "cmd.h"
#include "driver/uart.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"

using namespace esp_usb;

#include "driver/gpio.h"
#include "led_strip.h"

// GPIO assignment for LED strip
#define LED_STRIP_GPIO_PIN 48  // Dev Board GPIO for LED strip
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 1
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_BLINK_TIME 60
// Overall LED brightness, 0-255. WS2812 is very bright; keep this low.
#define LED_LEVEL 16
#define LED_LEVEL_DIM 4

// These values should be the most common for USB-Serial devices
#define BAUDRATE (115200)
#define STOP_BITS (0)  // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define PARITY (0)     // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define DATA_BITS (8)

#define SPP_QUEUE_LEN 256
#define UART_QUEUE_LEN 256

QueueHandle_t xQueueSpp;
QueueHandle_t xQueueUartTX;

extern "C" void spp_task(void *arg);

static const char *TAG = "VCP example";
static SemaphoreHandle_t device_disconnected_sem;
SemaphoreHandle_t led_sync;  // LED synchronization semaphore

int led_rx = 0;   // USB-Serial RX activity
int led_tx = 0;   // USB-Serial TX activity
int led_vcp = 0;  // USB-Serial connection status
int led_ble = 0;  // BLE connection status

TickType_t last_ble_activity = 0;
#define BLE_TIMEOUT_MS \
    5000  // 5 seconds of inactivity before we consider BLE disconnected

std::unique_ptr<CdcAcmDevice> vcp = nullptr;
// Guards vcp's lifetime: uart_tx_task calls into the object while
// vcp_open_task destroys/replaces it on USB reconnects
static SemaphoreHandle_t vcp_mutex;

// Serial settings last requested over BLE; re-applied when the USB serial
// device (re)connects. Written only by uart_tx_task after startup.
static cdc_acm_line_coding_t cur_line_coding = {
    .dwDTERate = BAUDRATE,
    .bCharFormat = STOP_BITS,
    .bParityType = PARITY,
    .bDataBits = DATA_BITS,
};
static uint8_t cur_ctrl_lines = 0;  // bit0 = DTR, bit1 = RTS

// Downstream USB identity. Captured in the host library's new-device
// callback — the only context where the device descriptor is reachable —
// and published over BLE (device info characteristic) only when the device
// actually OPENS in vcp_open_task, so the value always describes the device
// behind the data channel.
static std::atomic<uint16_t> last_usb_vid{0};
static std::atomic<uint16_t> last_usb_pid{0};

static void usb_new_dev_cb(usb_device_handle_t usb_dev) {
    const usb_device_desc_t *desc;
    if (usb_host_get_device_descriptor(usb_dev, &desc) != ESP_OK) {
        return;
    }
    if (desc->bDeviceClass == USB_CLASS_HUB) {
        return;  // hubs fire this callback too; not a serial device
    }
    // USB host context: cache only, publishing happens on open
    last_usb_vid.store(desc->idVendor);
    last_usb_pid.store(desc->idProduct);
    ESP_LOGI(TAG, "USB device attached: VID=0x%04X PID=0x%04X",
             desc->idVendor, desc->idProduct);
}

/// Queue the device-info characteristic update (VID LE16, PID LE16, flags)
static void publish_device_info(uint16_t vid, uint16_t pid, bool present) {
    CMD_t cmdBuf;
    cmdBuf.spp_event_id = DEVICE_INFO_EVT;
    cmdBuf.length = 5;
    cmdBuf.payload[0] = vid & 0xFF;
    cmdBuf.payload[1] = (vid >> 8) & 0xFF;
    cmdBuf.payload[2] = pid & 0xFF;
    cmdBuf.payload[3] = (pid >> 8) & 0xFF;
    cmdBuf.payload[4] = present ? 0x01 : 0x00;
    xQueueSend(xQueueSpp, &cmdBuf, 0);
}

/**
 * @brief Device event callback
 *
 * Handling device disconnection events
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event,
                         void *user_ctx) {
    // NOTE: never write status text into the BLE data channel here — it
    // corrupts binary protocols (e.g. esptool SLIP framing) mid-stream
    switch (event->type) {
        case CDC_ACM_HOST_ERROR:
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            ESP_LOGE(TAG, "CDC-ACM error, err_no = %d", event->data.error);
            // Treat transfer errors as a lost device: the target
            // re-enumerating (e.g. reset into the ROM bootloader) can
            // leave the open handle wedged with a dead IN pipe
            xSemaphoreGive(device_disconnected_sem);
            break;
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            ESP_LOGI(TAG, "Device suddenly disconnected");
            xSemaphoreGive(device_disconnected_sem);
            break;
        case CDC_ACM_HOST_SERIAL_STATE: {
            ESP_LOGI(TAG, "Serial state notif 0x%04X",
                     event->data.serial_state.val);
            // Forward to BLE clients via the serial state characteristic
            CMD_t cmdBuf;
            cmdBuf.spp_event_id = SERIAL_STATE_EVT;
            cmdBuf.length = 2;
            cmdBuf.payload[0] = event->data.serial_state.val & 0xFF;
            cmdBuf.payload[1] = (event->data.serial_state.val >> 8) & 0xFF;
            xQueueSend(xQueueSpp, &cmdBuf, 0);
            break;
        }
        case CDC_ACM_HOST_NETWORK_CONNECTION:
        default:
            break;
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg) {
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}

static void uart_tx_task(void *pvParameters) {
    CMD_t cmdBuf;

    while (1) {
        xQueueReceive(xQueueUartTX, &cmdBuf, portMAX_DELAY);

        // All vcp use happens under vcp_mutex: vcp_open_task destroys and
        // replaces the device object on reconnects, and calling into a
        // destroyed CdcAcmDevice crashes (LoadProhibited in tx_blocking).
        // The command is held, not dropped, while no device is open.
        while (1) {
            xSemaphoreTake(vcp_mutex, portMAX_DELAY);
            if (vcp != nullptr) {
                break;  // keep the mutex until the command is done
            }
            xSemaphoreGive(vcp_mutex);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (cmdBuf.spp_event_id == BLE_SET_CTRL_EVT) {
            if (cmdBuf.length == 1) {
                cur_ctrl_lines = cmdBuf.payload[0];
                ESP_LOGD(TAG, "ctrl line state 0x%02x", cur_ctrl_lines);
                esp_err_t ctrl_err = vcp->set_control_line_state(
                    cur_ctrl_lines & 0x01, cur_ctrl_lines & 0x02);
                if (ctrl_err != ESP_OK) {
                    ESP_LOGW(TAG, "set_control_line_state failed: %s",
                             esp_err_to_name(ctrl_err));
                }
            } else {
                // Timed batch: [state, delay/10ms] pairs, executed with
                // local timing so BLE latency cannot distort sequences
                // like the USB-Serial-JTAG bootloader reset
                ESP_LOGI(TAG, "ctrl sequence, %u steps", cmdBuf.length / 2);
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmdBuf.payload, cmdBuf.length,
                                         ESP_LOG_DEBUG);
                for (uint16_t i = 0; i + 1 < cmdBuf.length; i += 2) {
                    cur_ctrl_lines = cmdBuf.payload[i] & 0x03;
                    int64_t step_t0 = esp_timer_get_time();
                    esp_err_t ctrl_err = vcp->set_control_line_state(
                        cur_ctrl_lines & 0x01, cur_ctrl_lines & 0x02);
                    int64_t step_us = esp_timer_get_time() - step_t0;
                    if (ctrl_err != ESP_OK) {
                        ESP_LOGW(TAG, "ctrl sequence step %u failed: %s",
                                 i / 2, esp_err_to_name(ctrl_err));
                        break;
                    }
                    if (step_us > 20000) {
                        ESP_LOGW(TAG, "ctrl step %u (0x%02x) slow: %lld us",
                                 i / 2, cur_ctrl_lines, step_us);
                    }
                    if (cmdBuf.payload[i + 1]) {
                        vTaskDelay(pdMS_TO_TICKS(cmdBuf.payload[i + 1] * 10));
                    }
                }
            }
        } else if (cmdBuf.spp_event_id == BLE_SET_LINE_EVT) {
            memcpy(&cur_line_coding, cmdBuf.payload, sizeof(cur_line_coding));
            esp_err_t line_err = vcp->line_coding_set(&cur_line_coding);
            if (line_err != ESP_OK) {
                ESP_LOGW(TAG, "line_coding_set failed: %s",
                         esp_err_to_name(line_err));
            } else {
                ESP_LOGI(TAG, "Line coding: %" PRIu32 " baud",
                         cur_line_coding.dwDTERate);
            }
        } else {
            // Single attempt, drop on failure: the timed-out transfer may
            // still complete, so resending risks duplicating bytes in the
            // stream. Retries belong to the protocol above, not here.
            esp_err_t err = vcp->tx_blocking(cmdBuf.payload, cmdBuf.length, 1000);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "uart tx dropped %u bytes: %s", cmdBuf.length,
                         esp_err_to_name(err));
            } else if (xSemaphoreTake(led_sync, 1) == pdTRUE) {
                led_tx = 1;
                xSemaphoreGive(led_sync);
            }
        }

        xSemaphoreGive(vcp_mutex);
    }  // end while
    // Never reach here
    vTaskDelete(NULL);
}

/**
 * @brief Data received callback
 *
 * Just pass received data to stdout
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg) {
    size_t offset = 0;
    while (data_len > 0) {
        CMD_t cmdBuf;
        size_t chunk =
            data_len > SPP_DATA_MAX_LEN ? SPP_DATA_MAX_LEN : data_len;
        cmdBuf.spp_event_id = BLE_UART_EVT;
        cmdBuf.length = chunk;
        memcpy(cmdBuf.payload, data + offset, chunk);
        if (uxQueueSpacesAvailable(xQueueSpp) > 0) {
            BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, 0);
            if (err != pdTRUE) {
                ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
            }
        } else {
            ESP_LOGW(pcTaskGetName(NULL),
                     "BLE TX queue full, dropping serial data");
            return false;
        }
        offset += chunk;
        data_len -= chunk;
    }
    last_ble_activity = xTaskGetTickCount();
    // Never block the CDC data callback: a stalled callback stops IN
    // transfers and the device side (e.g. ROM bootloader) drops output
    if (xSemaphoreTake(led_sync, 0) == pdTRUE) {
        led_rx = 1;
        xSemaphoreGive(led_sync);
    }
    return true;
}

static void vcp_open_task(void *arg) {
    // Do everything else in a loop, to handle USB device reconnections
    while (1) {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = BLE_TIMEOUT_MS,
            .out_buffer_size = 5120,
            .in_buffer_size = 5120,
            .event_cb = handle_event,
            .data_cb = handle_rx,
            .user_arg = NULL,
        };

        // Retract the previous device before reopening. Destruction happens
        // under vcp_mutex so uart_tx_task can never be inside a call on the
        // dying object.
        xSemaphoreTake(vcp_mutex, portMAX_DELAY);
        vcp.reset();
        xSemaphoreGive(vcp_mutex);

        std::unique_ptr<CdcAcmDevice> dev(VCP::open(&dev_config));

        if (dev == nullptr) {
            // Not a known vendor VCP chip; try any standard CDC-ACM device
            // (e.g. an ESP32's native USB-Serial-JTAG port)
            auto cdc = std::make_unique<CdcAcmDevice>();
            if (cdc->open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, 0, &dev_config) ==
                ESP_OK) {
                ESP_LOGI(TAG, "Opened generic CDC-ACM device");
                dev = std::move(cdc);
            }
        }

        if (dev == nullptr) {
            ESP_LOGD(TAG, "Failed to open VCP device");
            continue;
        }
        vTaskDelay(10);

        // Restore the line coding last requested over BLE. Control lines
        // intentionally start deasserted: replaying a stale mid-reset
        // DTR/RTS state can hold the freshly enumerated target in reset
        // (open/disconnect storm).
        if (dev->line_coding_set(&cur_line_coding) != ESP_OK) {
            ESP_LOGW(TAG, "line_coding_set on open failed, reopening");
            continue;
        }
        cur_ctrl_lines = 0;

        ESP_LOGI(TAG, "VCP device opened");
        publish_device_info(last_usb_vid.load(), last_usb_pid.load(), true);

        xSemaphoreTake(led_sync, portMAX_DELAY);
        led_vcp = 1;
        xSemaphoreGive(led_sync);

        xSemaphoreTake(vcp_mutex, portMAX_DELAY);
        vcp = std::move(dev);
        xSemaphoreGive(vcp_mutex);

        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
        publish_device_info(0, 0, false);
        vTaskDelay(10);
    }
}

/**
 * @brief Configure the LED strip
 *
 * Initializes the LED strip with the specified GPIO pin, LED count, and RMT
 * configuration.
 *
 * @return led_strip_handle_t Handle to the initialized LED strip
 */
led_strip_handle_t configure_led(void) {
    // LED strip general initialization. It is only one on the dev board
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,  // The GPIO that connected to the
                                               // LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,  // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,    // LED strip model
        .color_component_format =
            LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // The color order of the strip:
                                                // GRB
        .flags = {
            .invert_out = false,  // don't invert the output signal
        }};

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols =
            64,  // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma =
                true,  // DMA feature is available on chips like ESP32-S3/P4
        }};

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    return led_strip;
}

/**
 * @brief LED task to control the LED strip based on USB-Serial and BLE activity
 *
 * This task updates the LED color based on the current activity state.
 * It uses a semaphore to synchronize access to the LED state variables.
 */
static void ledTask(void *arg) {
    int l_rx, l_tx, l_vcp, l_ble;
    led_strip_handle_t led_strip = configure_led();
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    while (1) {
        xSemaphoreTake(led_sync, portMAX_DELAY);
        l_rx = led_rx;
        l_tx = led_tx;
        l_vcp = led_vcp;
        l_ble = led_ble;
        xSemaphoreGive(led_sync);

        uint8_t r = 0, g = 0, b = 0;
        if (l_rx && l_tx) {  // Both RX and TX - yellow
            r = LED_LEVEL;
            g = LED_LEVEL;
        } else if (l_rx) {  // RX only - red
            r = LED_LEVEL;
        } else if (l_tx) {  // TX only - green
            g = LED_LEVEL;
        } else if (l_ble) {  // BLE connected, no traffic - purple
            r = LED_LEVEL_DIM;
            b = LED_LEVEL_DIM;
        } else if (l_vcp) {  // USB-Serial connected, no traffic - blue
            b = LED_LEVEL_DIM;
        }  // Nothing connected - off
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);

        xSemaphoreTake(led_sync, portMAX_DELAY);
        if (l_rx) led_rx = 0;
        if (l_tx) led_tx = 0;
        xSemaphoreGive(led_sync);
        vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Initialize queues for SPP and UART communication
 *
 * This function allocates memory for the queues in PSRAM if available, or falls
 * back to internal RAM if PSRAM allocation fails.
 * Queues in PSRAM can be much bigger than in internal RAM, which is useful for
 * handling larger data transfers without blocking when BLE connection is slow.
 */
static void init_queues() {
    size_t cmd_size = sizeof(CMD_t);
    void *spp_queue_storage = nullptr;
    void *uart_queue_storage = nullptr;
    StaticQueue_t *spp_queue_struct = nullptr;
    StaticQueue_t *uart_queue_struct = nullptr;

    if (esp_psram_is_initialized()) {
        // Allocate queue storage buffers
        spp_queue_storage = heap_caps_malloc(
            SPP_QUEUE_LEN * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        uart_queue_storage = heap_caps_malloc(
            UART_QUEUE_LEN * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        // Allocate queue structures
        spp_queue_struct = (StaticQueue_t *)heap_caps_malloc(
            sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        uart_queue_struct = (StaticQueue_t *)heap_caps_malloc(
            sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        if (spp_queue_storage && uart_queue_storage && spp_queue_struct &&
            uart_queue_struct) {
            xQueueSpp = xQueueCreateStatic(SPP_QUEUE_LEN, cmd_size,
                                           (uint8_t *)spp_queue_storage,
                                           spp_queue_struct);
            xQueueUartTX = xQueueCreateStatic(UART_QUEUE_LEN, cmd_size,
                                              (uint8_t *)uart_queue_storage,
                                              uart_queue_struct);
            ESP_LOGI(TAG, "Queues allocated in PSRAM");
        } else {
            // Free any allocated memory before falling back
            if (spp_queue_storage) heap_caps_free(spp_queue_storage);
            if (uart_queue_storage) heap_caps_free(uart_queue_storage);
            if (spp_queue_struct) heap_caps_free(spp_queue_struct);
            if (uart_queue_struct) heap_caps_free(uart_queue_struct);

            ESP_LOGW(TAG,
                     "PSRAM allocation failed, falling back to internal RAM");
            xQueueSpp = xQueueCreate(32, cmd_size);
            xQueueUartTX = xQueueCreate(32, cmd_size);
        }
    } else {
        xQueueSpp = xQueueCreate(32, cmd_size);
        xQueueUartTX = xQueueCreate(32, cmd_size);
        ESP_LOGI(TAG, "Queues allocated in internal RAM");
    }

    configASSERT(xQueueSpp);
    configASSERT(xQueueUartTX);
}

/**
 * @brief Main application entry point
 *
 * Initializes NVS, installs USB Host driver, creates tasks, and starts the
 * application.
 */
extern "C" void app_main(void) {
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    led_sync = xSemaphoreCreateMutex();
    vcp_mutex = xSemaphoreCreateMutex();
    assert(vcp_mutex);

    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Install USB Host driver. Should only be called once in entire application
    // ESP_LOGI(TAG, "Installing USB Host");
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events.
    // High priority, pinned to core 1 (Bluedroid owns core 0): slow IN
    // transfer turnaround makes USB serial devices drop TX data
    BaseType_t task_created = xTaskCreatePinnedToCore(
        usb_lib_task, "usb_lib", 4096, NULL, 15, NULL, 1);
    assert(task_created == pdTRUE);

    // ESP_LOGI(TAG, "Installing CDC-ACM driver");
    cdc_acm_host_driver_config_t cdc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 20,
        .xCoreID = 1,
        .new_dev_cb = usb_new_dev_cb,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install(&cdc_config));

    // Register VCP drivers to VCP service
    VCP::register_driver<FT23x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<CH34x>();

    // Neue Queue-Initialisierung
    init_queues();

    // Start tasks
    xTaskCreate(uart_tx_task, "UART-TX", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(spp_task, "SPP", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(vcp_open_task, "VCP Open", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(ledTask, "LED Task", 1024 * 4, NULL, 2, NULL);
}