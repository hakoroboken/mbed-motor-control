#include "InterfaceCAN.h"
#include "PinNames.h"
#include "mbed.h"
#include <cstdint>

#if !DEVICE_CAN
#error [NOT_SUPPORTED] CAN not supported for this target
#endif

CAN can(PB_8, PB_9, 1000 * 1000);

struct motor_param{
    int16_t SET_SPEED;
    int16_t speed;
    int16_t power;
    int16_t p_gain; // default: 50
    int16_t d_gain; // default: 2
    int16_t err_last;
} motors[4]; // Define an array for motors

// Constants and Variables
constexpr int16_t MAX_POWER = 16300;
constexpr int16_t MIN_POWER = -16300;

Ticker flipper;
volatile bool newCanMessage = false;

void calculatePID(motor_param& motor) {
    auto err = motor.SET_SPEED - motor.speed;
    // Calculate PD
    auto p_out = err / motor.p_gain;
    auto d_out = (err - motor.err_last) * motor.d_gain;
    motor.power += p_out + d_out;
    
    // Update error history
    motor.err_last = err;
    
    // Saturation Logic
    if (motor.power > MAX_POWER) motor.power = MAX_POWER;
    if (motor.power < MIN_POWER) motor.power = MIN_POWER;
}

void flip() {
    for(auto& motor : motors) {
        calculatePID(motor);
    }
    newCanMessage = true;
}

int main() {
    printf("CAN_MOTOR_CONTROL_COMMON\n");
    for (auto& motor : motors) {
        motor.p_gain = 50;
        motor.d_gain = 2;
    }

    flipper.attach(&flip, 2ms);

    while (1) {
        CANMessage rcv_msg;
        if (can.read(rcv_msg) == 1) {
            if (rcv_msg.id == 0x201) {
                motors[0].speed = static_cast<int16_t>((rcv_msg.data[2] << 8) | rcv_msg.data[3]);
            } else if (rcv_msg.id == 0x202) {
                motors[1].speed = static_cast<int16_t>((rcv_msg.data[2] << 8) | rcv_msg.data[3]);
            } else if (rcv_msg.id == 0x203) {
                motors[2].speed = static_cast<int16_t>((rcv_msg.data[2] << 8) | rcv_msg.data[3]);
            } else if (rcv_msg.id == 0x204) {
                motors[3].speed = static_cast<int16_t>((rcv_msg.data[2] << 8) | rcv_msg.data[3]);
            } else if (rcv_msg.id == 0x199) {
                for (auto i = 0; i < 4; i++) {
                    motors[i].SET_SPEED = static_cast<int16_t>((rcv_msg.data[i * 2] << 8) | rcv_msg.data[i * 2 + 1]);
                }
                printf("m1:%d,m2:%d,m3:%d,m4:%d\n", motors[0].SET_SPEED, motors[1].SET_SPEED, motors[2].SET_SPEED, motors[3].SET_SPEED);
            }
        }

        // Check flag and send CAN message outside of ISR
        if (newCanMessage) {
            auto msg = CANMessage();
            msg.id = 0x200;
            msg.len = 8;
            for (auto i = 0; i < 4; i++) {
                msg.data[i * 2] = (motors[i].power >> 8) & 0xff;
                msg.data[i * 2 + 1] = motors[i].power & 0xff;
            }
            can.write(msg);
            newCanMessage = false;
        }
    }
}
