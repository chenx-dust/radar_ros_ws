#include <hik_camera/hik_camera.h>
// #include <rm_utils/common.h>
// #include <rm_utils/frame_info.h>
#include <unistd.h>

#include <iostream>
// #include <rm_utils/perf.hpp>
#include <string>
// 尝试func, 如果返回值不是MV_OK(即0)则调用logger记录WARN日志
#define UPDBW(func)                                                                        \
    nRet = func;                                                                           \
    if (nRet != MV_OK) {                                                                   \
        RCLCPP_WARN(this->get_logger(), #func " failed!, error code: %x", (unsigned)nRet); \
    }

// 尝试func, 如果返回值不是MV_OK(即0)则调用logger记录FATAL日志
#define UPDBF(func)                                                                         \
    nRet = func;                                                                            \
    if (nRet != MV_OK) {                                                                    \
        RCLCPP_FATAL(this->get_logger(), #func " failed!, error code: %x", (unsigned)nRet); \
    }

// 对于不可恢复性错误重启相机节点
#define UPDBE(func)      \
    UPDBF(func)          \
    if (nRet != MV_OK) { \
        reset();         \
    }

using namespace hik_camera;

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options) : Node("hik_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");
    declare_params();
    bool use_sensor_data_qos = get_parameter("use_sensor_data_qos").as_bool();
    std::string camera_info_url = get_parameter("camera_info_url").as_string();
    // 创建pub
    bool use_intra = options.use_intra_process_comms();
    if (!use_intra) {
        RCLCPP_WARN(get_logger(), "Not In Intra Process Mode");
    }
    if (!use_sensor_data_qos) {
        RCLCPP_WARN(get_logger(), "Not Use Sensor Data Qos");
    }
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    image_pub = image_transport::create_camera_publisher(this, "image", qos);

    // frame_id 由名命名空间决定
    auto ns = std::string_view(get_namespace());
    auto ns_pos = ns.rfind('/');
    if (ns_pos != std::string_view::npos && ns_pos + 1 < ns.size()) {
        frame_id = ns.substr(ns.rfind('/') + 1);
        frame_id.append("_frame");
    } else {
        frame_id = "default_camera_frame";
    }

    // load camera info
    camera_info_manager =
        std::make_unique<camera_info_manager::CameraInfoManager>(this);
    if (camera_info_manager->validateURL(camera_info_url)) {
        camera_info_manager->loadCameraInfo(camera_info_url);
        camera_info_msg = camera_info_manager->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                    camera_info_url.c_str());
    }
    init_camera();
    RCLCPP_WARN(get_logger(), "Starting Camera Monitor thread.");
    monitor_on = true;
    monitor_thread = std::thread(&HikCameraNode::monitor, this);

    param_client = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface());
    param_event_sub = param_client->on_parameter_event([this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
        for (auto & new_param : event->new_parameters) {
            RCLCPP_INFO(this->get_logger(), "Param %s changed.", new_param.name.c_str());
        }
        param_changed = true;
    });
};

HikCameraNode::~HikCameraNode() {
    monitor_on = false;
    if (monitor_thread.joinable()) {
        monitor_thread.join();
    }
    close_device();
};

void HikCameraNode::declare_params() {
    this->declare_parameter("sn", "");
    // https://github.com/ros-perception/image_common/blob/136807edb7ff13452214a296fb4819bc63b5b09e/image_transport/src/camera_common.cpp#L62
    this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    this->declare_parameter("exposure_time", 4000.0);
    this->declare_parameter("gain", 15.0);
    this->declare_parameter("digital_shift", 6.0);
    this->declare_parameter("frame_rate", 60.0);
    this->declare_parameter("use_sensor_data_qos", false);
    rotate_180 = this->declare_parameter("rotate_180", false);
    if (rotate_180) {
        RCLCPP_WARN(this->get_logger(), "Rotate 180 degree");
    }
}

void HikCameraNode::init_camera() {
    MV_CC_DEVICE_INFO_LIST device_list;
    bool device_found = false;
    while (!device_found && rclcpp::ok()) {
        // 枚举设备
        UPDBW(MV_CC_EnumDevices(MV_USB_DEVICE, &device_list));
        std::string sn_to_find = get_parameter("sn").as_string();
        char device_sn[INFO_MAX_BUFFER_SIZE];
        if (device_list.nDeviceNum > 0) {
            if (get_parameter("sn").as_string() == "") {
                // 未设置camera sn,选择第一个
                RCLCPP_WARN(this->get_logger(), "Camera SN not set, use the first camera device");
                UPDBE(MV_CC_CreateHandle(&camera_handle, device_list.pDeviceInfo[0]));
                memcpy(device_sn,
                    device_list.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chSerialNumber,
                    INFO_MAX_BUFFER_SIZE);
                device_sn[63] = '\0'; // 以防万一
                device_found = true;
            } else {
                for (size_t i = 0; i < device_list.nDeviceNum; ++i) {
                    memcpy(device_sn,
                           device_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber,
                           INFO_MAX_BUFFER_SIZE);
                    device_sn[63] = '\0';  // 以防万一
                    // RCLCPP_INFO(this->get_logger(), "Camera SN list: %s", device_sn);
                    if (std::strncmp(device_sn, sn_to_find.c_str(), 64U) == 0) {
                        UPDBE(MV_CC_CreateHandle(&camera_handle, device_list.pDeviceInfo[i]));
                        device_found = true;
                        break;
                    }
                }
            }
        }
        if (device_found) {
            RCLCPP_INFO(this->get_logger(), "Camera SN: %s", device_sn);
            break;
        } else {
            RCLCPP_WARN(this->get_logger(), "Camera SN: %s not found.", sn_to_find.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    if (device_found) {
        open_device();
        set_hk_params();
        start_grab();
    }
}

void HikCameraNode::monitor() {
    while (rclcpp::ok() && monitor_on) {
        if (camera_failed) {
            RCLCPP_ERROR(this->get_logger(), "Camera failed! restarting...");
            reset();
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void HikCameraNode::start_grab() {
    // 开始采集
    MV_CC_StartGrabbing(camera_handle);
    // 开启采集线程
    grab_on = true;
    camera_failed = false;
    capture_thread = std::thread(&HikCameraNode::grab, this);
}

void HikCameraNode::stop_grab() {
    grab_on = false;
    if (capture_thread.joinable()) {
        capture_thread.join();
    }
    if (camera_handle) {
        MV_CC_StopGrabbing(camera_handle);
    }
}

std::pair<int, int> HikCameraNode::get_sensor_height_width() {
    // 获取max height/width
    MVCC_INTVALUE _max_height, _max_width;
    UPDBW(MV_CC_GetIntValue(camera_handle, "WidthMax", &_max_width));
    UPDBW(MV_CC_GetIntValue(camera_handle, "HeightMax", &_max_height));
    return std::pair{_max_height.nCurValue, _max_width.nCurValue};
}

void HikCameraNode::set_hk_params() {
    MV_CC_GetImageInfo(camera_handle, &img_info);
    static bool first_set = true;
    if (first_set) {
        first_set = false;
        UPDBW(MV_CC_SetEnumValue(camera_handle, "TriggerMode", MV_TRIGGER_MODE_OFF))
        UPDBW(MV_CC_SetEnumValue(camera_handle, "ExposureMode", MV_EXPOSURE_AUTO_MODE_OFF))
        UPDBW(MV_CC_SetEnumValue(camera_handle, "GainAuto", MV_GAIN_MODE_OFF))
        UPDBW(MV_CC_SetBoolValue(camera_handle, "BlackLevelEnable", false))
        UPDBW(MV_CC_SetEnumValue(camera_handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_ONCE))
        UPDBW(MV_CC_SetEnumValue(camera_handle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS))
        UPDBW(MV_CC_SetBoolValue(camera_handle, "AcquisitionFrameRateEnable", true))
        UPDBW(MV_CC_SetBoolValue(camera_handle, "DigitalShiftEnable", true))
    }
    UPDBW(MV_CC_SetFloatValue(camera_handle, "AcquisitionFrameRate", get_parameter("frame_rate").as_double()))
    UPDBW(MV_CC_SetFloatValue(camera_handle, "ExposureTime", get_parameter("exposure_time").as_double()))
    UPDBW(MV_CC_SetFloatValue(camera_handle, "Gain", get_parameter("gain").as_double()))
    UPDBW(MV_CC_SetFloatValue(camera_handle, "DigitalShift", get_parameter("digital_shift").as_double()))
}

void HikCameraNode::grab() {
    MV_FRAME_OUT out_frame;
    RCLCPP_INFO(this->get_logger(), "Publishing image!");

    // Init convert param
    convert_param.nWidth = img_info.nWidthValue;
    convert_param.nHeight = img_info.nHeightValue;
    convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    // 创建消息
    sensor_msgs::msg::Image image_msg;
    image_msg.encoding = "bgr8";
    image_msg.height = img_info.nHeightValue;
    image_msg.width = img_info.nWidthValue;
    image_msg.step = img_info.nWidthValue * 3;
    image_msg.data.resize(img_info.nWidthValue * img_info.nHeightValue * 3);

    while (rclcpp::ok() && grab_on) {
        if (param_changed) {
            set_hk_params();
            param_changed = false;
        }
        nRet = MV_CC_GetImageBuffer(camera_handle, &out_frame, 1000);
        if (MV_OK == nRet) {
            convert_param.pDstBuffer = image_msg.data.data();
            convert_param.nDstBufferSize = image_msg.data.size();
            convert_param.pSrcData = out_frame.pBufAddr;
            convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
            MV_CC_ConvertPixelType(camera_handle, &convert_param);

            if (rotate_180) {
                for (unsigned i = 0; i < img_info.nWidthValue * img_info.nHeightValue / 2; i++) {
                    std::swap(image_msg.data[i * 3], image_msg.data[(img_info.nWidthValue * img_info.nHeightValue - 1 - i) * 3]);
                    std::swap(image_msg.data[i * 3 + 1], image_msg.data[(img_info.nWidthValue * img_info.nHeightValue - 1 - i) * 3 + 1]);
                    std::swap(image_msg.data[i * 3 + 2], image_msg.data[(img_info.nWidthValue * img_info.nHeightValue - 1 - i) * 3 + 2]);
                }
            }

            auto time_now = this->now();
            image_msg.header.stamp = this->now();
            image_msg.header.frame_id = frame_id;
            camera_info_msg.header.stamp = time_now;
            camera_info_msg.header.frame_id = frame_id;
            image_pub.publish(image_msg, camera_info_msg);

            MV_CC_FreeImageBuffer(camera_handle, &out_frame);
            fail_cnt = 0;
        } else {
            RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
            // MV_CC_StopGrabbing(camera_handle);
            // MV_CC_StartGrabbing(camera_handle);
            fail_cnt++;
        }

        if (fail_cnt > 5) {
            RCLCPP_FATAL(this->get_logger(), "Camera failed!");
            grab_on = false;
            camera_failed = true;
        }
    }
}

void HikCameraNode::reset() {
    close_device();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    init_camera();
}

void HikCameraNode::open_device() {
    UPDBE(MV_CC_OpenDevice(camera_handle));
    UPDBE(MV_CC_CloseDevice(camera_handle));
    // 来回开关, 确保相机状态正常(增强鲁棒性措施)
    UPDBE(MV_CC_OpenDevice(camera_handle));
}

void HikCameraNode::close_device() {
    stop_grab();
    if (camera_handle) {
        MV_CC_CloseDevice(camera_handle);
        MV_CC_DestroyHandle(camera_handle);
    }
    RCLCPP_INFO(this->get_logger(), "HikCamera closed!");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<HikCameraNode>(rclcpp::NodeOptions());
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
