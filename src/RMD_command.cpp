#include "RMD_COMMAND.h"

void RMD_COMMAND::RMD_COMMAND_setup()
{
    output_degree = 0.;
    degree = 0.;
    torque = 0.;
    speed = 0.;
    pulse = 0.;
    pulse_to_degree = 0.00001525878 * 360 / Ratio;
}

// ============================ RMD command ============================
void RMD_COMMAND::ENCODER_ZERO_POSITON(int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0x19;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::TORQUE_CLOSED_LOOP_CONTROL(int s, int ID, int torque, struct can_frame frame)
{ /// torque = tau = (float)(~~)
    int16_t torque_low_byte = 0;
    int16_t torque_high_byte = 0;

    /*if (s == 16)
    {
        if (ID == 1)
        {
            torque = torque;
        }
        else if (ID == 2)
        {
            torque = -1 * torque; // base joint
        }
    }

    else if (s == 17) // can11
    {
        if (ID == 1)
        {
            torque = torque;
        }
        else if (ID == 2)
        {
            torque = -1 * torque;
        }
    }
    else if (s == 18) // can12
    {
        if (ID == 1)
        {
            torque = -1 * torque;
        }
        else if (ID == 2)
        {
            torque = torque;
        }
    }
    else if (s == 19) // can13
    {
        if (ID == 1)
        {
            torque = -1 * torque;
        }
        else if (ID == 2)
        {
            torque = -1 * torque;
        }
    }
    */

    torque_low_byte = (torque >> 0) & (0x00FF);
    torque_high_byte = (torque >> 8) & (0x00FF);

    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0xA1;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = torque_low_byte;
    frame.data[5] = torque_high_byte;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    try
    {
        int ret = write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame);
        if (ret < 0)
        {
            if (ret == EAGAIN || ret == EWOULDBLOCK)
            {
                printf("No data available.\n");
                return;
            }
            else
            {
                perror("Write");
                return;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void RMD_COMMAND::RPM_CLOSED_LOOP_CONTROL(int s, int ID, int rpm, struct can_frame frame)
{
    int32_t rpm_low_byte = 0;
    int32_t rpm_2nd_byte = 0;
    int32_t rpm_3rd_byte = 0;
    int32_t rpm_high_byte = 0;

    rpm_low_byte = (rpm >> 0) & (0x00FF);
    rpm_2nd_byte = (rpm >> 8) & (0x00FF);
    rpm_3rd_byte = (rpm >> 16) & (0x00FF);
    rpm_high_byte = (rpm >> 24) & (0x00FF);

    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0xA2;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = rpm_low_byte;
    frame.data[5] = rpm_2nd_byte;
    frame.data[6] = rpm_3rd_byte;
    frame.data[7] = rpm_high_byte;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::POSITION_CONTROL_A4(int s, int ID, float f_angle, int maxspeed, struct can_frame frame)
{
    // 각도를 0.01도 단위로 변환 (0.01도 단위로 표현하기 위해 100을 곱함)
    int32_t angle = static_cast<int32_t>(f_angle * 100);

    uint16_t speed = static_cast<uint16_t>(maxspeed);

    uint8_t angle_low_byte = (angle >> 0) & 0xFF;
    uint8_t angle_mid_low_byte = (angle >> 8) & 0xFF;
    uint8_t angle_mid_high_byte = (angle >> 16) & 0xFF;
    uint8_t angle_high_byte = (angle >> 24) & 0xFF;

    // 속도를 2바이트로 분리
    uint8_t speed_low_byte = (speed >> 0) & 0xFF;
    uint8_t speed_high_byte = (speed >> 8) & 0xFF;

    // CAN 프레임 설정
    frame.can_id = 0x140 + ID; // CAN ID: 0x140 + 모터 ID
    frame.can_dlc = 8;         // 데이터 길이: 8바이트

    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = speed_low_byte;      // 최대 속도 낮은 바이트
    frame.data[3] = speed_high_byte;     // 최대 속도 높은 바이트
    frame.data[4] = angle_low_byte;      // 각도 낮은 바이트
    frame.data[5] = angle_mid_low_byte;  // 각도 중간 낮은 바이트
    frame.data[6] = angle_mid_high_byte; // 각도 중간 높은 바이트
    frame.data[7] = angle_high_byte;     // 각도 높은 바이트

    // CAN 프레임 전송
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::REQUEST_MOTOR_STATUS(int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0x9C;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::REQUEST_DATA(int command, int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = command;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::WRITE_MOTOR_OFF(int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0x80;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}