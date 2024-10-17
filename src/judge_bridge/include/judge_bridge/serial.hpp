#pragma once

#include "judge_bridge/protocol.hpp"

#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <queue>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

namespace JudgeBridge {
    const int BAUDRATE = 115200;

    class JudgeSerial
    {
    public:
        using JudgePair = std::pair<uint16_t, std::vector<uint8_t>>;
        /**
        * Constructor.
        * @param port device name, example "/dev/ttyUSB0" or "COM4"
        * @throws boost::system::system_error if cannot open the
        * serial device
        */
        JudgeSerial(const std::string& port, bool enable_recorder)
        : io(), serial(io,port)
        {
            serial.set_option(boost::asio::serial_port_base::baud_rate(BAUDRATE));
            if (enable_recorder)
                recorder.init();
        }

        /**
        * Write a string to the serial device.
        * @param s string to write
        * @throws boost::system::system_error on failure
        */
        void write(CMD_ID, uint8_t* data, uint16_t data_length);

        /**
        * Blocks until a line is received from the serial device.
        * Eventual '\n' or '\r\n' characters at the end of the string are removed.
        * @throws boost::system::system_error on failure
        */
        JudgePair read();

        // 拿到 frame 队列最前面的数据
        JudgePair get_front_frame();

        void pop_frame();


    private:
        boost::asio::io_service io;
        boost::asio::serial_port serial;
        std::queue<JudgePair> frame_queue {};
        uint8_t seq = 0;
        std::mutex write_lock;

        struct SerialData_ {
            frame_header_t header;
            uint16_t cmd_id {};
            uint16_t tail {};

            void clear();
        };

        struct Recorder {
            bool is_enabled = false;
            // std::fstream file;
            boost::optional<boost::iostreams::file_sink> file;
            boost::iostreams::filtering_ostream out;

            void init();
            void write(const uint8_t* buff, size_t len);
            void stop();
        };

        Recorder recorder = Recorder();
    };

}


