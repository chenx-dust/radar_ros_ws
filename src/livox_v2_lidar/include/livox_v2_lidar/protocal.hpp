#include <boost/crc.hpp>
#include <boost/endian/arithmetic.hpp>
#include <radar_interface/livox_struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace livox_v2_lidar {
namespace protocal {

using namespace boost::endian;
constexpr size_t dot_num = 96;
constexpr size_t pc_msg_size = 1380;
constexpr uint16_t dest_port = 56000;
typedef boost::crc_optimal<16, 0x1021, 0xFFFF, 0x0000, false, false> livox_crc_16;
typedef boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true> livox_crc_32;

// 以下是 Livox 激光雷达接受到的数据包结构
#pragma pack(push, 1)
struct header {
    little_uint8_t version;
    little_uint16_t length;
    little_uint16_t time_interval;
    little_uint16_t dot_num;
    little_uint16_t udp_cnt;
    little_uint8_t frame_cnt;
    little_uint8_t data_type;
    little_uint8_t time_type;
    little_uint8_t pack_info;
    little_uint8_t _padding[11];
    little_uint32_t crc32;
    little_uint64_t timestamp;
};
struct pcd1 {
    little_int32_t x;
    little_int32_t y;
    little_int32_t z;
    little_uint8_t reflectivity;
    little_uint8_t tag;
};
using pcd1_span = std::array<struct pcd1, dot_num>;
struct imu {
    little_float32_t gyro_x;
    little_float32_t gyro_y;
    little_float32_t gyro_z;
    little_float32_t acc_x;
    little_float32_t acc_y;
    little_float32_t acc_z;
};

struct frame_header {
    little_uint8_t sof = 0xaa;
    little_uint8_t version = 0;
    little_uint16_t length;
    little_uint32_t seq_num;
    little_uint16_t cmd_id = 0;
    little_uint8_t cmd_type = 0;
    little_uint8_t sender_type = 0;
    little_uint8_t resv[6];
    little_uint16_t crc_16;
    little_uint32_t crc_32;
};

constexpr uint16_t PUSH_KEY_VALUE = 0x0102;
struct push_kv_start {
    uint16_t key_num = 1;
    uint16_t rsvd;
    uint16_t key = 0x001a;
    uint16_t length = 1;
    uint8_t value = 0x01;
};
#pragma pack(pop)

enum msg_type {
    MSG_IMU = 0,
    MSG_PCD1 = 1,
    MSG_PCD2 = 2, // 不使用
};

inline bool check_header_pcd1(const uint8_t* data)
{
    auto& header = *reinterpret_cast<const struct header*>(data);
    if (header.version.value() != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "version is not 0");
        return false;
    }
    if (header.length.value() != pc_msg_size) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "length is not 1380");
        return false;
    }
    if (header.dot_num.value() != dot_num) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "dot_num is not 96");
        return false;
    }
    // TODO: support other data_type
    if (header.data_type.value() != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "data_type is not 1");
        return false;
    }
    if ((header.pack_info.value() & 0x03) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "pack_info is not 0");
        return false;
    }
    livox_crc_32 crc;
    crc.process_bytes(data + 28, header.length - 28);
    if (crc.checksum() != header.crc32.value()) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "crc32 error");
        return false;
    }
    return true;
}

inline size_t write_frame_buffer(uint8_t* buf, uint16_t cmd_id, const uint8_t* data, size_t size, size_t max_size)
{
    static uint16_t seq_num = 0;
    frame_header header;
    size_t length = sizeof(frame_header) + size;
    if (length > max_size)
        throw std::out_of_range("Out of buffer size.");

    header.length = length;
    header.cmd_id = cmd_id;
    header.seq_num = seq_num++;
    const uint8_t* header_data = reinterpret_cast<const uint8_t*>(&header);

    livox_crc_16 crc16;
    livox_crc_32 crc32;
    crc16.process_bytes(header_data, 18);
    header.crc_16 = crc16.checksum();
    crc32.process_bytes(data, size);
    header.crc_32 = crc32.checksum();

    std::copy(header_data, header_data + sizeof(frame_header), buf);
    std::copy(data, data + size, buf + sizeof(frame_header));

    return length;
}
}
}