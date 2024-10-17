#include <rclcpp/rclcpp.hpp>
#include <boost/crc.hpp>
#include <boost/endian/arithmetic.hpp>
#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_v1_lidar/fast_crc.hpp>

namespace livox_v1_lidar {
namespace protocal {

using namespace boost::endian;
constexpr size_t dot_num = 96;

constexpr uint16_t dest_port = 65000;

constexpr uint8_t heartbeat_raw[] {
    0xaa, 0x01, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x04, 0xd7, 0x00, 0x03, 0x38, 0xba, 0x8d, 0x0c
};
constexpr uint8_t start_data[] {
    0x00, 0x04, 0x01, 0xab, 0xb2, 0x3b, 0x4b
};

constexpr uint16_t livox_crc16_init = 0x4c49;
constexpr uint32_t livox_crc32_init = 0x564f580a;

// 以下是 Livox 激光雷达接受到的数据包结构
#pragma pack(push, 1)

struct data_header {
    little_uint8_t version;
    little_uint8_t slot_id;
    little_uint8_t lidar_id;
    little_uint8_t reserved;
    little_uint32_t status_code;
    little_uint8_t timestamp_type;
    little_uint8_t data_type;
    little_uint64_t timestamp;
};

struct pc_type2 {
    little_int32_t x;
    little_int32_t y;
    little_int32_t z;
    little_uint8_t reflectivity;
    little_uint8_t tag;
};

using type2_span = std::array<struct pc_type2, dot_num>;

struct frame_header {
    little_uint8_t sof = 0xaa;
    little_uint8_t version = 1;
    little_uint16_t length;
    little_uint8_t cmd_type = 0;
    little_uint16_t seq_num;
    little_uint16_t crc_16;
};

struct handshake_data {
    little_uint8_t cmd_set = 0x00;
    little_uint8_t cmd_id = 0x01;
    little_uint8_t user_ip[4];
    little_uint16_t data_port;
    little_uint16_t cmd_port;
    little_uint16_t imu_port;
};
#pragma pack(pop)

enum msg_type {
    MSG_IMU = 0,
    MSG_PCD1 = 1,
    MSG_PCD2 = 2, // 不使用
};

inline bool check_header(const data_header& header)
{
    if (header.version.value() == 0xaa)
        return false;
    if (header.version.value() != 5) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "version is not 5, got %u", header.version.value());
        return false;
    }
    if (header.data_type.value() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "data_type is not 2, got %u", header.data_type.value());
        return false;
    }
    if (header.lidar_id.value() != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("header"), "lidar_id is not 1, got %u", header.lidar_id.value());
        return false;
    }
    return true;
}

inline size_t write_frame_buffer(uint8_t *buf, const uint8_t* data, size_t size, size_t max_size)
{
    static uint16_t seq_num = 0;
    size_t length = sizeof(frame_header) + size + sizeof(uint32_t);
    if (length > max_size)
        throw std::out_of_range("Out of buffer size.");
    frame_header header;

    header.length = length;
    header.seq_num = seq_num++;
    const uint8_t* header_data = reinterpret_cast<const uint8_t*>(&header);
    header.crc_16 = FastCRC::mcrf4xx_calc(livox_crc16_init, header_data, sizeof(header) - sizeof(header.crc_16));
    std::copy(header_data, header_data + sizeof(frame_header), buf);

    std::copy(data, data + size, buf + sizeof(frame_header));

    little_uint32_t crc32_val = FastCRC::crc32_calc(livox_crc32_init, buf, sizeof(header) + size);
    const uint8_t* crc32_data = reinterpret_cast<const uint8_t*>(&crc32_val);
    std::copy(crc32_data, crc32_data + sizeof(crc32_val), buf + sizeof(frame_header) + size);

    return length;
}

}
}