#include "HSLink_Pro_expansion.h"
#include "LED.h"
#include "board.h"
#include "dap_main.h"
#include "setting.h"
#include "usb2uart.h"
#include "usb_configuration.h"
#include <eeprom_emulation.h>
#include <hid_comm.h>
#include <hpm_dma_mgr.h>
#include <hpm_ewdg_drv.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpiom_drv.h>
#include <hpm_nor_flash.h>
#include <hpm_romapi.h>
#include <memory>

#define LOG_TAG "main"

#include "elog.h"

static void serial_number_init(void) {
#define OTP_CHIP_UUID_IDX_START (88U)
#define OTP_CHIP_UUID_IDX_END (91U)
    uint32_t uuid_words[4];

    uint32_t word_idx = 0;
    for (uint32_t i = OTP_CHIP_UUID_IDX_START; i <= OTP_CHIP_UUID_IDX_END; i++) {
        uuid_words[word_idx++] = ROM_API_TABLE_ROOT->otp_driver_if->read_from_shadow(i);
    }

    sprintf(serial_number, "%08X%08X%08X%08X", uuid_words[0], uuid_words[1], uuid_words[2], uuid_words[3]);
    printf("Serial number: %s\n", serial_number);
}

ATTR_ALWAYS_INLINE
static inline void SWDIO_DIR_Init(void) {
    HPM_IOC->PAD[SWDIO_DIR].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);

    gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(SWDIO_DIR), GPIO_GET_PIN_INDEX(SWDIO_DIR), gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO0, GPIO_GET_PORT_INDEX(SWDIO_DIR), GPIO_GET_PIN_INDEX(SWDIO_DIR));
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(SWDIO_DIR), GPIO_GET_PIN_INDEX(SWDIO_DIR), 1);
}

static void EWDG_Init() {
    clock_add_to_group(clock_watchdog0, 0);
    ewdg_config_t config;
    ewdg_get_default_config(HPM_EWDG0, &config);

    config.enable_watchdog = true;
    config.int_rst_config.enable_timeout_reset = true;
    config.ctrl_config.use_lowlevel_timeout = false;
    config.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;

    /* Set the EWDG reset timeout to 5 second */
    config.cnt_src_freq = 32768;
    config.ctrl_config.timeout_reset_us = 5 * 1000 * 1000;

    /* Initialize the WDG */
    hpm_stat_t status = ewdg_init(HPM_EWDG0, &config);
    if (status != status_success) {
        printf(" EWDG initialization failed, error_code=%d\n", status);
    }
}

extern void WS2812_Init(void);

extern e2p_t e2p;
constexpr uint32_t BL_OFFSET = 0x400;
//extern uint8_t *__new_bl_start__;
uint32_t NEW_BL_PATH = (uint32_t) 0x80078000;
const uint32_t option[4] = {0xfcf90002, 0x00000006, 0x1000, 0x0};

#include "bl/HSLink-Pro-Bootloader.h"

[[noreturn]]// make compiler happy
int
main() {
    board_init();
    EWDG_Init();
    WS2812_Init();
    if (bl_setting.bl_version.major >= 2) {
        log_d("already upgraded");
        for (auto i = 0; i < 3; i++) {
            neopixel->SetPixel(0, 0, 0xFF / 8, 0);
            neopixel->Flush();
            board_delay_ms(500);
            neopixel->SetPixel(0, 0, 0, 0);
            neopixel->Flush();
            board_delay_ms(500);
            ewdg_refresh(HPM_EWDG0);
        }
        HSP_EnterHSLinkBootloader();
    } else {
        log_d("upgrading bootloader...");
        auto nor_cfg = (uint32_t *) malloc(sizeof(option));
        log_d("checking fw header...");
        nor_flash_read(&e2p.nor_config, reinterpret_cast<uint8_t *>(nor_cfg), NEW_BL_PATH, sizeof(option));
        elog_hexdump(LOG_TAG, 16, nor_cfg, sizeof(option));
        if (memcmp(nor_cfg, option, sizeof(option)) != 0) {
            log_e("fw header mismatch! No upgrade!");
            for (auto i = 0; i < 3; i++) {
                neopixel->SetPixel(0, 0xFF / 8, 0, 0);
                neopixel->Flush();
                board_delay_ms(500);
                neopixel->SetPixel(0, 0, 0, 0);
                neopixel->Flush();
                board_delay_ms(500);
                ewdg_refresh(HPM_EWDG0);
            }
            HSP_EnterHSLinkBootloader();
        }
        log_d("fw header check pass");
        {
            const uint32_t COPY_LEN = 4 * 1024;
            auto buf = std::make_unique<uint8_t[]>(COPY_LEN);
            disable_global_irq(CSR_MSTATUS_MIE_MASK);
            nor_flash_erase(&e2p.nor_config, 0, 128 * 1024);
            for (auto i = 0; i < 127 * 1024; i += COPY_LEN) {
                auto copy_len = std::min(COPY_LEN, uint32_t(127 * 1024 - i));
                log_d("copy from 0x%X to 0x%X, size 0x%X",
                      i + NEW_BL_PATH + BOARD_FLASH_BASE_ADDRESS,
                      i + BL_OFFSET + BOARD_FLASH_BASE_ADDRESS,
                      copy_len);
                nor_flash_read(&e2p.nor_config, buf.get(), i + NEW_BL_PATH , copy_len);
                nor_flash_write(&e2p.nor_config, buf.get(), i + BL_OFFSET, copy_len);
            }
            enable_global_irq(CSR_MSTATUS_MIE_MASK);
        }
        log_d("copy done");
        for (auto i = 0; i < 3; i++) {
            neopixel->SetPixel(0, 0, 0xFF / 8, 0);
            neopixel->Flush();
            board_delay_ms(500);
            neopixel->SetPixel(0, 0, 0, 0);
            neopixel->Flush();
            board_delay_ms(500);
            ewdg_refresh(HPM_EWDG0);
        }
        HSP_Reboot();
    }
    serial_number_init();
    board_init_usb(HPM_USB0);
    dma_mgr_init();

    SWDIO_DIR_Init();

    Setting_Init();

    HSP_Init();
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 5);
    uartx_preinit();
    USB_Configuration();

    led.SetNeoPixel(neopixel);

    bl_setting.app_version.major = CONFIG_BUILD_VERSION_MAJOR;
    bl_setting.app_version.minor = CONFIG_BUILD_VERSION_MINOR;
    bl_setting.app_version.patch = CONFIG_BUILD_VERSION_PATCH;

    bl_setting.fail_cnt = 0;

    while (true) {
        ewdg_refresh(HPM_EWDG0);
        chry_dap_handle();
        chry_dap_usb2uart_handle();
        usb2uart_handler();
        HSP_Loop();
#ifdef CONFIG_USE_HID_CONFIG
        HID_Handle();
#endif
        led.Handle();
    }
}
