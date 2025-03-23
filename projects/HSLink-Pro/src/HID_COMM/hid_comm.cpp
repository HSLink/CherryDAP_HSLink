#include "hid_comm.h"
#include "usbd_core.h"
#include "dap_main.h"
#include "setting.h"
#include "usb_configuration.h"
#include "HSLink_Pro_expansion.h"
#include "b64.h"

#ifdef CONFIG_USE_HID_CONFIG

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

using namespace rapidjson;

#include <unordered_map>
#include <functional>
#include "memory"
#include <hpm_nor_flash.h>
#include <hpm_crc32.h>
#include <crc32.h>

#define LOG_TAG "HID_Msg"

#include "elog.h"

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t HID_read_buffer[HID_PACKET_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t HID_write_buffer[HID_PACKET_SIZE];

typedef enum {
    HID_STATE_BUSY = 0,
    HID_STATE_DONE,
} HID_State_t;

typedef enum {
    HID_RESPONSE_SUCCESS = 0,
    HID_RESPONSE_FAILED,
} HID_Response_t;

const char *response_str[] = {
        "success",
        "failed"
};

std::string_view filed_miss_msg = "Some fields are missing";

static volatile HID_State_t HID_ReadState = HID_STATE_BUSY;

/*!< custom hid report descriptor */
const uint8_t hid_custom_report_desc[HID_CUSTOM_REPORT_DESC_SIZE] = {
        /* USER CODE BEGIN 0 */
        0x06, 0x00, 0xff, /* USAGE_PAGE (Vendor Defined Page 1) */
        0x09, 0x01, /* USAGE (Vendor Usage 1) */
        0xa1, 0x01, /* COLLECTION (Application) */
        0x85, 0x02, /*   REPORT ID (0x02) */
        0x09, 0x02, /*   USAGE (Vendor Usage 1) */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0) */
        0x25, 0xff, /*LOGICAL_MAXIMUM (255) */
        0x75, 0x08, /*   REPORT_SIZE (8) */
        0x96, 0xff, 0x03, /*   REPORT_COUNT (1023) */
        0x81, 0x02, /*   INPUT (Data,Var,Abs) */
        /* <___________________________________________________> */
        0x85, 0x01, /*   REPORT ID (0x01) */
        0x09, 0x03, /*   USAGE (Vendor Usage 1) */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0) */
        0x25, 0xff, /*   LOGICAL_MAXIMUM (255) */
        0x75, 0x08, /*   REPORT_SIZE (8) */
        0x96, 0xff, 0x03, /*   REPORT_COUNT (1023) */
        0x91, 0x02, /*   OUTPUT (Data,Var,Abs) */

        /* <___________________________________________________> */
        0x85, 0x03, /*   REPORT ID (0x03) */
        0x09, 0x04, /*   USAGE (Vendor Usage 1) */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0) */
        0x25, 0xff, /*   LOGICAL_MAXIMUM (255) */
        0x75, 0x08, /*   REPORT_SIZE (8) */
        0x96, 0xff, 0x03, /*   REPORT_COUNT (1023) */
        0xb1, 0x02, /*   FEATURE (Data,Var,Abs) */
        /* USER CODE END 0 */
        0xC0 /*     END_COLLECTION	             */
};

void usbd_hid_custom_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    (void) busid;
    (void) ep;
    USB_LOG_DBG("actual in len:%d\r\n", nbytes);
    // custom_state = HID_STATE_IDLE;
}

void usbd_hid_custom_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    (void) busid;
    HID_ReadState = HID_STATE_DONE;
    // usbd_ep_start_read(0, HID_OUT_EP, HID_read_buffer, HID_PACKET_SIZE);// 重新开启读取
}

struct usbd_endpoint hid_custom_in_ep = {
        .ep_addr = HID_IN_EP,
        .ep_cb = usbd_hid_custom_in_callback,
};

struct usbd_endpoint hid_custom_out_ep = {
        .ep_addr = HID_OUT_EP,
        .ep_cb = usbd_hid_custom_out_callback,
};

static bool CheckField(const Value &object, std::pair<std::string_view, Type> field) {
    auto &[field_name, field_type] = field;
    if (!object.HasMember(field_name.data())) {
        return false;
    }
    if (object[field_name.data()].GetType() != field_type) {
        return false;
    }
    return true;
}

static bool CheckFields(const Value &object, std::vector<std::pair<std::string_view, Type>> fields) {
    for (auto &field: fields) {
        if (!CheckField(object, field)) {
            return false;
        }
    }
    return true;
}

static void FillStatus(HID_Response_t status, char *res, const char *message) {
    StringBuffer buffer;
    Writer writer(buffer);
    writer.StartObject();
    writer.Key("status");
    writer.String(response_str[status]);
    writer.Key("message");
    writer.String(message);
    writer.EndObject();
    std::strcpy(res, buffer.GetString());
}

static void FillStatus(HID_Response_t status, char *res, std::string_view message) {
    FillStatus(status, res, message.data());
}

static void FillStatus(HID_Response_t status, char *res) {
    FillStatus(status, res, "");
}

static void Hello(Document &root, char *res) {
    (void) root;
    StringBuffer buffer;

    Writer writer(buffer);
    writer.StartObject();

    writer.Key("serial");
    writer.String(serial_number);

    writer.Key("model");
    writer.String("HSLink-Pro");

    writer.Key("version");
    char version[32];
    snprintf(version, 32, "%d.%d.%d",
             bl_setting.app_version.major,
             bl_setting.app_version.minor,
             bl_setting.app_version.patch
    );
    writer.String(version);

    writer.Key("bootloader");
    snprintf(version, 32, "%d.%d.%d",
             bl_setting.bl_version.major,
             bl_setting.bl_version.minor,
             bl_setting.bl_version.patch
    );
    writer.String(version);

    writer.Key("hardware");
    if ((HSLink_Hardware_Version.major == 0x00 &&
         HSLink_Hardware_Version.minor == 0x00 &&
         HSLink_Hardware_Version.patch == 0x00) ||
        (HSLink_Hardware_Version.major == 0xFF &&
         HSLink_Hardware_Version.minor == 0xFF &&
         HSLink_Hardware_Version.patch == 0xFF)
            ) {
        writer.String("unknown");
    } else {
        char version[32];
        std::sprintf(version, "%d.%d.%d",
                     HSLink_Hardware_Version.major,
                     HSLink_Hardware_Version.minor,
                     HSLink_Hardware_Version.patch);
        writer.String(version);
    }

    writer.Key("nickname");
    writer.String(HSLink_Setting.nickname);

    writer.EndObject();

    std::strcpy(res, buffer.GetString());
}

static void settings(Document &root, char *res) {
    if (!CheckField(root, {"data", Type::kObjectType})) {
        FillStatus(HID_RESPONSE_FAILED, res, "data not found");
        return;
    }

    const Value &data = root["data"].GetObject();

    if (CheckField(data, {"boost", Type::kTrueType}) or
        CheckField(data, {"boost", Type::kFalseType})) {
        HSLink_Setting.boost = data["boost"].GetBool();
    }

    auto _get_mode_fn = [](const char *mode) {
        if (strcmp(mode, "spi") == 0) {
            return PORT_MODE_SPI;
        }
        return PORT_MODE_GPIO;
    };
    if (CheckField(data, {"swd_port_mode", Type::kStringType})) {
        HSLink_Setting.swd_port_mode = _get_mode_fn(data["swd_port_mode"].GetString());
    }
    if (CheckField(data, {"jtag_port_mode", Type::kStringType})) {
        HSLink_Setting.jtag_port_mode = _get_mode_fn(data["jtag_port_mode"].GetString());
    }

    if (CheckField(data, {"power", Type::kObjectType})) {
        const Value &power = data["power"];
        if (CheckField(power, {"vref", Type::kNumberType})) {
            HSLink_Setting.power.voltage = power["vref"].GetDouble();
        }
        if (CheckField(power, {"power_on", Type::kTrueType}) or CheckField(power, {"power_on", Type::kFalseType})) {
            HSLink_Setting.power.power_on = power["power_on"].GetBool();
        }
        if (CheckField(power, {"port_on", Type::kTrueType}) or CheckField(power, {"port_on", Type::kFalseType})) {
            HSLink_Setting.power.port_on = power["port_on"].GetBool();
        }
    }

    if (CheckField(data, {"reset", Type::kArrayType})) {
        for (auto &reset: data["reset"].GetArray()) {
            if (!SETTING_GET_RESET_MODE(HSLink_Setting.reset, RESET_NRST) && strcmp(reset.GetString(), "nrst") == 0) {
                SETTING_SET_RESET_MODE(HSLink_Setting.reset, RESET_NRST);
            } else if (!SETTING_GET_RESET_MODE(HSLink_Setting.reset, RESET_POR) &&
                       strcmp(reset.GetString(), "por") == 0) {
                SETTING_SET_RESET_MODE(HSLink_Setting.reset, RESET_POR);
            } else if (!SETTING_GET_RESET_MODE(HSLink_Setting.reset, RESET_ARM_SWD_SOFT) && strcmp(
                    reset.GetString(), "arm_swd_soft") == 0) {
                SETTING_SET_RESET_MODE(HSLink_Setting.reset, RESET_ARM_SWD_SOFT);
            }
        }
    }

    if (CheckField(data, {"led", Type::kTrueType}) or CheckField(data, {"led", Type::kFalseType})) {
        HSLink_Setting.led = data["led"].GetBool();
    }
    if (CheckField(data, {"led_brightness", Type::kNumberType})) {
        HSLink_Setting.led_brightness = data["led_brightness"].GetUint();
    }

    Setting_Save();

    FillStatus(HID_RESPONSE_SUCCESS, res);
}

static void set_nickname(Document &root, char *res) {
    if (!CheckField(root, {"nickname", Type::kStringType})) {
        FillStatus(HID_RESPONSE_FAILED, res, "nickname not found");
        return;
    }

    std::memset(HSLink_Setting.nickname, 0, sizeof(HSLink_Setting.nickname));
    std::strcpy(HSLink_Setting.nickname, root["nickname"].GetString());

    Setting_Save();

    FillStatus(HID_RESPONSE_SUCCESS, res);
}

static void upgrade(Document &root, char *res) {
    (void) root;

    FillStatus(HID_RESPONSE_SUCCESS, res);
    usbd_ep_start_write(0, HID_IN_EP, HID_write_buffer, HID_PACKET_SIZE);
    clock_cpu_delay_ms(5); // 确保已经发送完毕
    HSP_EnterHSLinkBootloader();
}

static void entry_sys_bl(Document &root, char *res) {
    (void) root;

    FillStatus(HID_RESPONSE_SUCCESS, res);
    usbd_ep_start_write(0, HID_IN_EP, HID_write_buffer, HID_PACKET_SIZE);
    clock_cpu_delay_ms(5); // 确保已经发送完毕
    HSP_EntrySysBootloader();
}

static void entry_hslink_bl(Document &root, char *res) {
    (void) root;

    FillStatus(HID_RESPONSE_SUCCESS, res);
    usbd_ep_start_write(0, HID_IN_EP, HID_write_buffer, HID_PACKET_SIZE);
    clock_cpu_delay_ms(5); // 确保已经发送完毕
    HSP_EnterHSLinkBootloader();
}

static void set_hw_ver(Document &root, char *res) {
    if (!CheckField(root, {"hw_ver", Type::kStringType})) {
        FillStatus(HID_RESPONSE_FAILED, res, "hw_ver not found");
        return;
    }

    auto hw_ver_s = std::string{root["hw_ver"].GetString()};
    size_t pos1 = hw_ver_s.find('.');
    size_t pos2 = hw_ver_s.find('.', pos1 + 1);

    version_t ver{0};
    ver.major = std::stoi(hw_ver_s.substr(0, pos1));
    ver.minor = std::stoi(hw_ver_s.substr(pos1 + 1, pos2 - pos1 - 1));
    ver.patch = std::stoi(hw_ver_s.substr(pos2 + 1));

    Setting_SaveHardwareVersion(ver);

    FillStatus(HID_RESPONSE_SUCCESS, res);
}

static void get_setting(Document &root, char *res) {
    (void) root;
    StringBuffer buffer;
    Writer writer(buffer);
    writer.StartObject();

    writer.Key("boost");
    writer.Bool(HSLink_Setting.boost);

    writer.Key("swd_port_mode");
    writer.String(HSLink_Setting.swd_port_mode == PORT_MODE_SPI ? "spi" : "gpio");

    writer.Key("jtag_port_mode");
    writer.String(HSLink_Setting.jtag_port_mode == PORT_MODE_SPI ? "spi" : "gpio");

    writer.Key("power");
    writer.StartObject();
    writer.Key("vref");
    writer.Double(HSLink_Setting.power.voltage);
    writer.Key("power_on");
    writer.Bool(HSLink_Setting.power.power_on);
    writer.Key("port_on");
    writer.Bool(HSLink_Setting.power.port_on);
    writer.EndObject();

    writer.Key("reset");
    writer.StartArray();
    if (SETTING_GET_RESET_MODE(HSLink_Setting.reset, RESET_NRST)) {
        writer.String("nrst");
    }
    if (SETTING_GET_RESET_MODE(HSLink_Setting.reset, RESET_POR)) {
        writer.String("por");
    }
    if (SETTING_GET_RESET_MODE(HSLink_Setting.reset, RESET_ARM_SWD_SOFT)) {
        writer.String("arm_swd_soft");
    }
    writer.EndArray();

    writer.Key("led");
    writer.Bool(HSLink_Setting.led);

    writer.Key("led_brightness");
    writer.Uint(HSLink_Setting.led_brightness);

    writer.EndObject();

    std::strcpy(res, buffer.GetString());
}

static void erase_bl_b(Document &root, char *res) {
    board_flash_erase(BL_B_SLOT_OFFSET + BOARD_FLASH_BASE_ADDRESS, BL_B_SLOT_SIZE);
    FillStatus(HID_RESPONSE_SUCCESS, res);
}

static void write_bl_b(Document &root, char *res) {
#define PACK_SIZE 512
    if (!CheckFields(root,
                     {{"addr", Type::kNumberType},
                      {"len",  Type::kNumberType},
                      {"data", Type::kStringType}})) {
        FillStatus(HID_RESPONSE_FAILED, res, filed_miss_msg);
        return;
    }
    auto addr = root["addr"].GetInt();
    auto len = root["len"].GetInt();
    auto data_b64 = root["data"].GetString();
    auto data_b64_len = root["data"].GetStringLength();
    log_d("addr: 0x%X, len: %d, data_len: %d", addr, len, data_b64_len);
//    elog_hexdump("b64", 16, data_b64, data_b64_len);
    if (addr + len > BL_B_SLOT_SIZE) {
        const char *message = "addr out of range";
        USB_LOG_WRN("%s\n", message);
        FillStatus(HID_RESPONSE_FAILED, res, message);
        return;
    }
    if (len % 4 != 0) {
        log_w("len %d not multiple of 4", len);
        return;
    }
    if (addr % 512 != 0) {
        log_w("addr %d not multiple of 512", addr);
        return;
    }
    log_i("addr: 0x%X, len: %d", addr, len);
    size_t data_len = 0;
    uint8_t *data = b64_decode_ex(data_b64, data_b64_len, &data_len);
    log_d("solve b64 data_len: %d", data_len);
//    elog_hexdump(LOG_TAG, 16, data, data_len);
    if (data_len != len) {
        log_w("data_len != len");
        return;
    }
    if (data_len > PACK_SIZE) {
        log_w("data_len > %d", PACK_SIZE);
        return;
    }
    auto write_addr = addr + BL_B_SLOT_OFFSET + BOARD_FLASH_BASE_ADDRESS;
    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    board_flash_write(data, write_addr, len);
    enable_global_irq(CSR_MSTATUS_MIE_MASK);
    log_i("write %d bytes to 0x%X done", len, write_addr);
    FillStatus(HID_RESPONSE_SUCCESS, res);
    free(data);
}

static void upgrade_bl(Document &root, char *res) {
    if (!CheckFields(root,
                     {{"len", Type::kNumberType},
                      {"crc", Type::kStringType}})) {
        FillStatus(HID_RESPONSE_FAILED, res, filed_miss_msg);
        return;
    }
    auto len = root["len"].GetInt();
    auto crc_str = root["crc"].GetString();
    auto crc = strtoul(crc_str + 2, nullptr, 16);
    if (len > BL_B_SLOT_SIZE) {
        log_w("len > %d", BL_B_SLOT_SIZE);
        return;
    }
    if (len % 4) {
        log_w("len %% 4 != 0");
        return;
    }

    {
        uint32_t crc_calc = 0xFFFFFFFF;
        const uint32_t CRC_CALC_LEN = 8 * 1024;
        auto buf = std::make_unique<uint8_t[]>(CRC_CALC_LEN);
        for (uint32_t i = 0; i < len; i += CRC_CALC_LEN) {
            auto calc_len = std::min(CRC_CALC_LEN, len - i);
            board_flash_read(buf.get(), i + BL_B_SLOT_OFFSET + BOARD_FLASH_BASE_ADDRESS, calc_len);
            crc_calc = CRC_CalcArray_Software(buf.get(), calc_len, crc_calc);
//        log_d("crc calc 0x%x", crc_calc);
        }
        if (crc != crc_calc) {
            log_w("crc != crc_calc recv 0x%x calc 0x%x", crc, crc_calc);
            return;
        }
    }

    log_d("crc check pass, start copy...");
    {
        const uint32_t COPY_LEN = 4 * 1024;
//        const uint32_t SKIP_COPY = 0x1000;
        auto buf = std::make_unique<uint8_t[]>(COPY_LEN);
        disable_global_irq(CSR_MSTATUS_MIE_MASK);
        board_flash_erase(BOARD_FLASH_BASE_ADDRESS, BL_SIZE);
//        board_flash_erase(BOARD_FLASH_BASE_ADDRESS + SKIP_COPY, BL_SIZE - SKIP_COPY);
        for (auto i = 0; i < BL_SIZE; i += COPY_LEN) {
//            if (i < SKIP_COPY) {
//                log_d("skip copy 0x%X", i);
//                continue;
//            }
            log_d("copy from 0x%X to 0x%X, size 0x%X",
                  i + BL_B_SLOT_OFFSET + BOARD_FLASH_BASE_ADDRESS,
                  i + BOARD_FLASH_BASE_ADDRESS,
                  COPY_LEN);
            board_flash_read(buf.get(), i + BL_B_SLOT_OFFSET + BOARD_FLASH_BASE_ADDRESS, COPY_LEN);
            board_flash_write(buf.get(), i + BL_OFFSET + BOARD_FLASH_BASE_ADDRESS, COPY_LEN);
        }
        enable_global_irq(CSR_MSTATUS_MIE_MASK);
    }
    log_d("copy done");
    {
        const uint32_t COPY_LEN = 512;
        auto buf_1 = std::make_unique<uint8_t[]>(COPY_LEN);
        auto buf_2 = std::make_unique<uint8_t[]>(COPY_LEN);
        disable_global_irq(CSR_MSTATUS_MIE_MASK);
        for (auto i = 0; i < BL_SIZE; i += COPY_LEN) {
            board_flash_read(buf_1.get(), i + BL_B_SLOT_OFFSET + BOARD_FLASH_BASE_ADDRESS, COPY_LEN);
            board_flash_read(buf_2.get(), i + BL_OFFSET + BOARD_FLASH_BASE_ADDRESS, COPY_LEN);
            if (memcmp(buf_1.get(), buf_2.get(), COPY_LEN) != 0) {
                log_w("copy fail at 0x%X", i);
                log_d("in b slot");
                elog_hexdump(LOG_TAG, 16, buf_1.get(), COPY_LEN);
                log_d("in bl");
                elog_hexdump(LOG_TAG, 16, buf_2.get(), COPY_LEN);
            }
        }
        enable_global_irq(CSR_MSTATUS_MIE_MASK);
    }

    log_d("check done");

}

static void HID_Write(const std::string &res) {
    std::strcpy(reinterpret_cast<char *>(HID_write_buffer + 1), res.c_str());
}

static void HID_Write(const char *res) {
    std::strcpy(reinterpret_cast<char *>(HID_write_buffer + 1), res);
}

using HID_Command_t = std::function<void(Document &, char *res)>;

void HID_Handle() {
    if (HID_ReadState == HID_STATE_BUSY) {
        return; // 接收中，不处理
    }
    memset(HID_write_buffer + 1, 0, HID_PACKET_SIZE - 1);

    static std::unordered_map<std::string_view, HID_Command_t> hid_command = {
            {"Hello",           Hello},
            {"settings",        settings},
            {"set_nickname",    set_nickname},
            {"get_setting",     get_setting},
            {"upgrade",         upgrade},
            {"entry_sys_bl",    entry_sys_bl},
            {"entry_hslink_bl", entry_hslink_bl},
            {"set_hw_ver",      set_hw_ver},
            {"erase_bl_b",      erase_bl_b},
            {"write_bl_b",      write_bl_b},
            {"upgrade_bl",      upgrade_bl}
    };

    Document root;
    const auto parse = reinterpret_cast<char *>(HID_read_buffer + 1);
    root.Parse(parse);
    [&]() {
        if (root.HasParseError()
            || !root.HasMember("name")
                ) {
            HID_Write("parse error!\n");
            return;
        }

        auto name = root["name"].GetString();
        auto it = hid_command.find(name);
        if (it == hid_command.end()) {
            // 没有这个命令
            HID_Write("command " + std::string(name) + " not found!\n");
            return;
        }
        USB_LOG_DBG("command %s\n", name);
        it->second(root, reinterpret_cast<char *>(HID_write_buffer + 1));
    }();

    HID_ReadState = HID_STATE_BUSY;
    usbd_ep_start_write(0, HID_IN_EP, HID_write_buffer, HID_PACKET_SIZE);
    usbd_ep_start_read(0, HID_OUT_EP, HID_read_buffer, HID_PACKET_SIZE);
}

#endif
