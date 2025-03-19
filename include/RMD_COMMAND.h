#ifndef RMD_COMMAND_H
#define RMD_COMMAND_H

#include <cstdint>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>

#define PI  3.14159265359
#define R2D 180./PI

#define Ratio 36


class RMD_COMMAND
{
    public:
        void RMD_COMMAND_setup();
    
        struct Motor_state_t
        {
            // float theta_org[2]; /// was 3 (joints) 8
            float theta[2];
            // float ptheta[2];
            float speed[2];
            float torque[2];
            float temp[2];
            int data_count[4] = {0, 0, 0, 0};
        };

        void READ_TORQUE_SPEED_ANGLE(int s, int canable, struct can_frame frame);
        void ENCODER_ZERO_POSITON(int s, int ID, struct can_frame frame);
        void TORQUE_CLOSED_LOOP_CONTROL(int s, int ID, int torque, struct can_frame frame);
        void RPM_CLOSED_LOOP_CONTROL(int s, int ID, int rpm, struct can_frame frame);
        void POSITION_CONTROL_A4(int s, int ID, float f_angle, int maxspeed, struct can_frame frame);
        void REQUEST_MOTOR_STATUS(int s, int ID, struct can_frame frame);
        void REQUEST_DATA(int command, int s, int ID, struct can_frame frame);
        // void READ_MULTITURN_ANGLE(int s, int ID, struct can_frame frame);
        // void READ_ENCODER_STATUS(int s, int ID, struct can_frame frame);
        // void WRITE_ENCODER_OFFSET(int s, int ID, int encoder_offset, struct can_frame frame);
        void WRITE_MOTOR_OFF(int s, int ID, struct can_frame frame);

        // Motor_state_t Get_motor_state();
    private:
        // Motor_state_t motor_state;
        int8_t temp;
        int16_t torque;
        int16_t speed;
        float degree;
        int16_t pulse; // encoder
        float output_degree;
        float pulse_to_degree;

        int command;
};

#endif
