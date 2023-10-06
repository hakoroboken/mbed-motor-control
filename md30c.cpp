#include "DigitalIn.h"
#include "DigitalOut.h"
#include "InterruptIn.h"
#include "PinNames.h"
#include "PinNamesTypes.h"
#include "PwmOut.h"
#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <vector>

#include "mbed_retarget.h"
#include "stm32f4xx.h"

// User settings

#define PWM_PIN_V1 PC_9
#define PWM_PIN_V2 PB_8 
#define PWM_PIN_V3 PB_9
#define PWM_PIN_V4 PA_6

#define DIGITAL_PIN_V1 PA_5
#define DIGITAL_PIN_V2 PA_7
#define DIGITAL_PIN_V3 PC_8
#define DIGITAL_PIN_V4 PC_5

// User settings end

#define PULSEWIDTH_US 2000
#define MAXIMUM_BUFFER_SIZE 128

static BufferedSerial serial_port(USBTX, USBRX);

// pin setup

PwmOut V1_PWM(PWM_PIN_V1);
PwmOut V2_PWM(PWM_PIN_V2);
PwmOut V3_PWM(PWM_PIN_V3);
PwmOut V4_PWM(PWM_PIN_V4);

DigitalOut V1_Digital(DIGITAL_PIN_V1);
DigitalOut V2_Digital(DIGITAL_PIN_V2);
DigitalOut V3_Digital(DIGITAL_PIN_V3);
DigitalOut V4_Digital(DIGITAL_PIN_V4);


Ticker safeTimer;
bool safeFlag = false;
int16_t safeCounter = 0;

// deserialize func
template <typename T>
T deserialize(std::vector<uint8_t>& bytes) {
    static_assert(std::is_trivially_copyable<T>::value, "Data type is not trivially copyable");

    T data;
    std::memcpy(&data, bytes.data(), sizeof(data));
    return data;
}

struct motor_control_msg{
    std::uint8_t id;
    float motor_1;
    float motor_2;
    float motor_3;
    float motor_4;
};


inline auto set_motor(const float & power, mbed::PwmOut & pwm, mbed::DigitalOut & digital){
    auto power_ = power;

    if(power > 1.0){
        power_ = 1.0;
    }

    if(power < -1.0){
        power_ = -1.0;
    }

    auto width = abs((int)(power_ * (float)2000));
    pwm.pulsewidth_us(width);
    if(power_ > 0.0){
        digital = 1;
    }else{
        digital = 0;
    }
}

auto safeCheck(){
    if(safeFlag){
        safeFlag = false;
        safeCounter = 0;
        return;
    }else{
        safeCounter++;
    }

    if(safeCounter > 1000){
        safeCounter = 0;
        safeFlag = false;

        set_motor(0.0, V1_PWM,V1_Digital);
        set_motor(0.0, V2_PWM,V2_Digital);
        set_motor(0.0, V3_PWM,V3_Digital);
        set_motor(0.0, V4_PWM,V4_Digital);
    }
}


int main() {
    safeTimer.attach(&safeCheck, 1ms);

    // Serial
    serial_port.set_baud(115200);
    serial_port.set_format(8, BufferedSerial::None, 1);
    uint8_t buf[MAXIMUM_BUFFER_SIZE] = {0};

    // PWM setup
    V1_PWM.period_us(PULSEWIDTH_US);
    V2_PWM.period_us(PULSEWIDTH_US);
    V3_PWM.period_us(PULSEWIDTH_US);
    V4_PWM.period_us(PULSEWIDTH_US);
    
    vector<uint8_t> data;

    while (1) {
        if (const ssize_t num = serial_port.read(buf, sizeof(buf))) {
            for(auto i = 0; i < num ; i++){
                data.push_back(buf[i]);
            }

            if(data.size() > 128){
                data.clear();
            }

            if(std::find(data.begin() , data.end() , 's') == data.end()){data.clear();}
            if(std::find(data.begin() , data.end() , 't') == data.end()){data.clear();}
            if(std::find(data.begin() , data.end() , 'e') == data.end()){continue;}
            if(std::find(data.begin() , data.end() , 'n') == data.end()){continue;}


            data.pop_back();
            data.pop_back();
            data.erase(data.begin());
            data.erase(data.begin());
            
            auto mc_msg = deserialize<motor_control_msg>(data);
            set_motor(mc_msg.motor_1, V1_PWM, V1_Digital);
            set_motor(mc_msg.motor_2, V2_PWM, V2_Digital);
            set_motor(mc_msg.motor_3, V3_PWM, V3_Digital);
            set_motor(mc_msg.motor_4, V4_PWM, V4_Digital);

            data.clear();
            safeFlag = true;
        }
    }
}
