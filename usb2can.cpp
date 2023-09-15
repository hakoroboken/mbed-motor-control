#include <cstdint>
#include <vector>
#include <algorithm>
#include <cstring>

#include "mbed.h"
#include "BufferedSerial.h"
#include "CAN.h"

// Deserialize function
template <typename T>
T deserialize(const std::vector<uint8_t>& bytes) {
    static_assert(std::is_trivially_copyable<T>::value, "Data type is not trivially copyable");

    T data;
    std::memcpy(&data, bytes.data(), sizeof(data));
    return data;
}

struct motor_control_msg {
    std::uint8_t id;
    float motor_1;
    float motor_2;
    float motor_3;
    float motor_4;
};

inline auto set_motor(const float& power) -> int16_t {
    if (power > 1.0) {
        return 5000;
    }
    if (power < -1.0) {
        return -5000;
    }
    return static_cast<int16_t>(power * 5000.0f);
}

constexpr size_t MAXIMUM_BUFFER_SIZE = 128;

static BufferedSerial serial_port(USBTX, USBRX);
static CAN can(PB_8, PB_9, 1000 * 1000);

int main() {
    // Serial setup
    serial_port.set_baud(115200);
    serial_port.set_format(8, BufferedSerial::None, 1);
    uint8_t buf[MAXIMUM_BUFFER_SIZE] = {0};

    std::vector<uint8_t> data;

    while (1) {
        if (const ssize_t num = serial_port.read(buf, sizeof(buf))) {
            data.insert(data.end(), buf, buf + num);

            if (data.size() > 128 || 
                data.end() == std::find(data.begin(), data.end(), 's') ||
                data.end() == std::find(data.begin(), data.end(), 't')) {
                data.clear();
                continue;
            }

            if (data.end() == std::find(data.begin(), data.end(), 'e') ||
                data.end() == std::find(data.begin(), data.end(), 'n')) {
                continue;
            }

            data.erase(data.begin(), data.begin() + 2);
            data.erase(data.end() - 2, data.end());

            const auto mc_msg = deserialize<motor_control_msg>(data);

            CANMessage msg;
            msg.id = 0x199;
            msg.len = 8;
            msg.data[0] = (set_motor(mc_msg.motor_1) >> 8) & 0xff;
            msg.data[1] = set_motor(mc_msg.motor_1) & 0xff;
            msg.data[2] = (set_motor(mc_msg.motor_2) >> 8) & 0xff;
            msg.data[3] = set_motor(mc_msg.motor_2) & 0xff;
            msg.data[4] = (set_motor(mc_msg.motor_3) >> 8) & 0xff;
            msg.data[5] = set_motor(mc_msg.motor_3) & 0xff;
            msg.data[6] = (set_motor(mc_msg.motor_4) >> 8) & 0xff;
            msg.data[7] = set_motor(mc_msg.motor_4) & 0xff;
            
            can.write(msg);
            data.clear();
        }
    }
}
