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
  /*
  Instruction description
  Turns off the motor output and also clears the motor running state, not in any closed loop mode.
  */
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
  // write(connected socket num, buffer(data saved), send data size,) : return(fail) -> -1
  if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  {
    perror("Write");
    return;
  }
}

void RMD_COMMAND::TORQUE_CLOSED_LOOP_CONTROL(int s, int ID, int torque, struct can_frame frame)
{
  int16_t torque_low_byte = 0;
  int16_t torque_high_byte = 0;
/*
  if (s == 16)
  {
    if (ID == 1)
    {
      torque = torque;
    }
    else if (ID == 2)
    {
      torque = torque;
    }
    else if (ID == 3)
    {
      torque = torque;
    }
  }
  else if (s == 17)
  {
    if (ID == 1)
    {
      torque = torque;
    }
    else if (ID == 2)
    {
      torque = -1 * torque;
    }
    else if (ID == 3)
    {
      torque = -1 * torque;
    }
  }
  else if (s == 18)
  {
    if (ID == 1)
    {
      torque = -1 * torque;
    }
    else if (ID == 2)
    {
      torque = torque;
    }
    else if (ID == 3)
    {
      torque = torque;
    }
  }
  else if (s == 19)
  {
    if (ID == 1)
    {
      torque = -1 * torque;
    }
    else if (ID == 2)
    {
      torque = -1 * torque;
    }
    else if (ID == 3)
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
        // 데이터가 없어서 블로킹이 발생하지 않은 경우
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
  uint8_t rpm_low_byte = 0;
  uint8_t rpm_2nd_byte = 0;
  uint8_t rpm_3rd_byte = 0;
  uint8_t rpm_high_byte = 0;

  rpm_low_byte = (rpm >> 0) & (0x00FF);
  rpm_2nd_byte = (rpm >> 8) & (0x00FF);
  rpm_3rd_byte = (rpm >> 16) & (0x00FF);
  rpm_high_byte = (rpm >> 24) & (0x00FF);

  frame.can_id = 0x140 + ID; // CAN ID 설정
  frame.can_dlc = 8; // 데이터 길이 설정

  frame.data[0] = 0xA2; // RPM 제어 명령
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
  /*
  try
  {
    int ret = write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame);
    if (ret < 0)
    {
      if (ret == EAGAIN || ret == EWOULDBLOCK)
      {
        // 데이터가 없어서 블로킹이 발생하지 않은 경우
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
  }*/
}

void RMD_COMMAND::POSITION_CONTROL_A4(int s, int ID, float f_angle, int maxspeed, struct can_frame frame) // output deg
{
  // std::cout<<"POSITION_CONTROL_A4"<<std::endl;
  int angle = 0;
  maxspeed = maxspeed * 9 * R2D;

  int32_t angle_low_byte = 0;
  int32_t angle_high_byte = 0;
  int32_t angle_mid_low_byte = 0;
  int32_t angle_mid_high_byte = 0;
  int16_t speed_low_byte = 0;
  int16_t speed_high_byte = 0;
/*
  if (s == 16)
  {
    // std::cout<<"s==0 "<<f_angle<<std::endl;
    if (ID == 1)
    {
      angle = (int)(f_angle * 100 * 9);
      // std::cout<<"f_angle of FLHR "<<f_angle<<std::endl;
    }
    else if (ID == 2)
    {
      angle = (int)((f_angle - 71.78) * 100 * 9);
    }
    else if (ID == 3)
    {
      angle = (int)((f_angle + 160.59) * 100 * 9);
    }
  }
  else if (s == 17)
  {
    if (ID == 1)
    {
      angle = (int)(f_angle * 100 * 9);
    }
    else if (ID == 2)
    {
      angle = -1 * (int)((f_angle - 72.4) * 100 * 9);
    }
    else if (ID == 3)
    {
      angle = -1 * (int)((f_angle + 160.59) * 100 * 9);
    }
  }
  else if (s == 18)
  {
    // std::cout<<"s==2"<<f_angle<<std::endl;
    if (ID == 1)
    {
      angle = -1 * (int)(f_angle * 100 * 9);
      // std::cout<<"f_angle of RLHR "<<f_angle<<std::endl;
    }
    else if (ID == 2)
    {
      angle = (int)((f_angle - 72.32) * 100 * 9);
    }
    else if (ID == 3)
    {
      angle = (int)((f_angle + 160.59) * 100 * 9);
    }
  }
  else if (s == 19)
  {
    if (ID == 1)
    {
      angle = -1 * (int)(f_angle * 100 * 9);
      // std::cout<<"f_angle of FLHR "<<f_angle<<std::endl;
    }
    else if (ID == 2)
    {
      angle = -1 * (int)((f_angle - 71.8) * 100 * 9);
    }
    else if (ID == 3)
    {
      angle = -1 * (int)((f_angle + 160.59) * 100 * 9);
    }
  }
*/
  angle_low_byte = (angle >> 0) & (0x00FF);
  angle_mid_low_byte = (angle >> 8) & (0x00FF);
  angle_mid_high_byte = (angle >> 16) & (0x00FF);
  angle_high_byte = (angle >> 24) & (0x00FF);

  speed_low_byte = (maxspeed >> 0) & (0x00FF);
  speed_high_byte = (maxspeed >> 8) & (0x00FF);

  frame.can_id = 0x140 + ID;
  frame.can_dlc = 8;

  frame.data[0] = 0xA4;
  frame.data[1] = 0x00;
  frame.data[2] = speed_low_byte;
  frame.data[3] = speed_high_byte;
  frame.data[4] = angle_low_byte;
  frame.data[5] = angle_mid_low_byte;
  frame.data[6] = angle_mid_high_byte;
  frame.data[7] = angle_high_byte;
  // std::cout << "FRAME.data"<< std::dec <<  frame.data[4] <<" ,"<< std::dec <<frame.data[5]<<" ,"<<std::dec <<frame.data[6]<<" ,"<<std::dec <<frame.data[7]<< std::endl;

  if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
  /*
  try
  {
    int ret = write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame);
    if (ret < 0)
    {
      if (ret == EAGAIN || ret == EWOULDBLOCK)
      {
        // 데이터가 없어서 블로킹이 발생하지 않은 경우
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
  }*/
  
  // if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  // {
  //   // perror("Write");
  //   std::cout << "write error " << std::endl;
  //   return;
  // }
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
  frame.can_dlc = 8; // data length

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

/*void RMD_COMMAND::READ_MULTITURN_ANGLE(int s, int ID, struct can_frame frame)
{
  frame.can_id = 0x140 + ID;
  frame.can_dlc = 8; // data length

  frame.data[0] = 0x92;
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
}*/

//read multi - turn encoder position data command(0x60)
/*void RMD_COMMAND::READ_ENCODER_STATUS(int s, int ID, struct can_frame frame)
{
  frame.can_id = 0x140 + ID;
  frame.can_dlc = 8; // data length

  frame.data[0] = 0x60;
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
}*/

/*void RMD_COMMAND::WRITE_ENCODER_OFFSET(int s, int ID, int encoder_offset, struct can_frame frame)
{
  int16_t encoder_offset_low_byte = 0;
  int16_t encoder_offset_high_byte = 0;

  encoder_offset_low_byte = (encoder_offset >> 0) & (0x00FF);
  encoder_offset_high_byte = (encoder_offset >> 8) & (0x00FF);

  frame.can_id = 0x140 + ID;
  frame.can_dlc = 8;

  frame.data[0] = 0xA1;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = encoder_offset_low_byte;
  frame.data[7] = encoder_offset_high_byte;

  if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  {
    perror("Write");
    return;
  }
}*/

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
