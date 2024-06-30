#ifndef CYBERGEAR_DRIVER_H
#define CYBERGEAR_DRIVER_H

#include <Arduino.h>  // for micros and delay function
#include "cybergear_defs.h"
#include <FlexCAN_T4.h>

struct CyberGearStatus {
    float position;
    float speed;
    float torque;
    uint16_t temperature;
};

struct CyberGearMotionCommand {
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
};

class CyberGearDriver {
    public:
        CyberGearDriver();
        CyberGearDriver(uint8_t master_can_id, uint8_t cybergear_can_id);
        virtual ~CyberGearDriver();

        /**
         * @retval -1 Error
         * @retval 0 OK
         */
        int init_can();
        void init_motor(uint8_t mode);
        void enable_motor();
        void stop_motor();
        void set_run_mode(uint8_t mode);

        void set_limit_speed(float speed);
        void set_limit_current(float current);
        void set_limit_torque(float torque);

        // MODE MOTION
        void send_motion_control(CyberGearMotionCommand cmd);

        // MODE_CURRENT
        void set_current_kp(float kp);
        void set_current_ki(float ki);
        void set_current_filter_gain(float gain);
        void set_current(float current);

        // MODE_POSITION
        void set_position_kp(float kp);
        void set_position(float position);
        void request_mech_pos();
        void get_mech_pos();

        // MODE_SPEED
        void set_speed_kp(float kp);
        void set_speed_ki(float ki);
        void set_speed(float speed);

        uint8_t get_run_mode() const;
        uint8_t get_motor_can_id() const;
        void set_motor_can_id(uint8_t can_id);

        void request_status();
        void process_message(CAN_message_t& message);
        void request_VBUS();
        void get_VBUS();
        CyberGearStatus get_status();
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
        
    private:
        uint16_t _float_to_uint(float x, float x_min, float x_max, int bits);
        float _uint_to_float(uint16_t x, float x_min, float x_max);
        void _send_can_package(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data);
        void _send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max);

        uint8_t _cybergear_can_id;
        uint8_t _master_can_id;
        uint8_t _run_mode;
        bool _use_serial_debug;
        CyberGearStatus _status;
        
};

#endif