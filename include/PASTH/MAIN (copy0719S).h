#ifndef MAIN_ACTIVE_NODE_HPP
#define MAIN_ACTIVE_NODE_HPP

// CAN & Network Library
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// ---signal & Flie Library--------
#include <signal.h>
#include <fstream>

// Thread & Time Library
#include <pthread.h>
#include <sched.h>
#include <chrono>
#include <time.h>
#include <mutex>

// C++ Basic Library
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <cmath>
#include <functional>
#include <memory>
#include <climits>
#include <cstring>
#include <string>
#include <array>

// ETC
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>

// ROS2 Libraries
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp" // 

// User Headers
// #include "CRobot/CRobot.h"

#include "RMD_COMMAND.h"

#define PI 3.14159265359
#define PI2 6.28318530718
#define D2R PI / 180.

// Constants
#define TauMap 62.5
#define TauReadMap 62.061
#define ns_OneMilSec 1000000
#define ns_TwoMilSec 2000000
#define ns_FiveMilSec 5000000
#define ns_SixMilSec 6000000
#define ns_TenMilSec 10000000
#define ns_OneSec 1000000000
#define ns_TwoSec 2000000000
#define NUM_OF_CANABLE 6
#define NUM_OF_MOTOR 8 // 16
#define SERIAL 0
#define BLUETOOTH 1
#define JOINT_NUM 16

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr int NUM_MOTORS = 8; // 16;
constexpr int QUEUE_SIZE = 20;

volatile bool main_thread_exit = false;
volatile bool thread_flag = true;

// void cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg);
void MD_RPM(int32_t vesc_id, int32_t rpm);
int32_t left_rpm_command_;
int32_t right_rpm_command_;
int32_t left_wheel_rpm;
int32_t right_wheel_rpm;
// int32_t ID=1;
int32_t rpm = 0;
float wheel_base = 0.5;   // 너비
float wheel_radius = 0.1; // 바퀴 지름
float linear_velocity;
float angular_velocity;
struct can_frame frame;
int cansock[NUM_OF_CANABLE];


class MainActiveNode : public rclcpp::Node
{
public:
    MainActiveNode();
    ~MainActiveNode();

private:
    struct Motor_state_t
    {
        float theta_org[6];
        float theta[6];
        float speed[6];
        float torque[6];
        float temp[6];
        int data_count[6] = {0, 0, 0, 0, 0, 0}; // ?
    };

    struct period_info
    {
        struct timespec next_period;
        struct timespec current_time_1;
        struct timespec current_time_2;
        long period_ns;
    };

    typedef struct Joint
    {
        float InitPos;
        float InitVel;
        float InitAcc;

        float RefPos;
        float RefVel;
        float RefAcc;

        float RefPrePos;
        float RefPreVel;
        float RefPreAcc;

        float RefTorque;

        float RefTargetPos;
        float RefTargetVel;
        float RefTargetAcc;

        float CurrentPos;
        float CurrentVel;
        float CurrentAcc;

        float CurrentTorque;

        float FrictionCompensationTorque;

        float RefInputCurrent;
        float ActualInputCurrent;

        float KpGain;
        float KdGain;
    } JOINT;

    typedef enum
    {
        POSITION_COMMAND = 0,
        TORQUE_COMMAND,
        TEST,
        RPM_COMMAND
    } COMMAND_MODE;
typedef enum
    {
        NORMAL_MODE=0,
        FLIPPER_MODE=1,
        STAIR_MODE=2,
        STOP=3
    } DRIVING_MODE;
    struct Joint_j
    {
        float q;
        float qdot;
        float lpf_qdot;
        float torque;
        float prev_q;
        float calVel_by_qData;
    };

    // Variables
    rclcpp::TimerBase::SharedPtr Publish_timer;
    pthread_t main_thread;
    DRIVING_MODE drive_mode;
    std::mutex _mtx;
    RMD_COMMAND Rmd;

    std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, 4> flag_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4> time_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, JOINT_NUM> degree_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> tau_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> torque_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> motor_degree_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> temp_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> speed_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, 12> refvel_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, 12> curvel_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 12> traj_publishers;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>>cmd_vel_subscription_;

    int8_t temp = 0;
    int16_t torque = 0;
    int16_t speed = 0;
    int16_t pulse = 0;
    float pulse_to_degree = 0.00001525878 * 40.;
    float degree = 0.;
    float previous_degree[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float output_degree = 0.;
    float Kp_HR, Kp_HP, Kp_KN;
    float Kd_HR, Kd_HP, Kd_KN;
    float Kp_FR, Kp_FL, Kp_BR, Kp_BL;
    float Kd_FR, Kd_FL, Kd_BR, Kd_BL;
    int angle_switch[NUM_OF_CANABLE][NUM_OF_MOTOR];
    int flag[NUM_OF_CANABLE][NUM_OF_MOTOR];
    int tau[NUM_OF_CANABLE][NUM_OF_MOTOR];

    int cansock[NUM_OF_CANABLE];
    int command;
    int err_count = 0;

    FILE *drok6_data = NULL;
    int PrintDataIndex = 0;
    // float PrintData[SAVEDATA_TIME][SAVEDATA_LENGTH];
    float PrintDataTime = 0.0;

    float init_theta[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float desired_theta[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float traj_theta[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float calc_Vel[NUM_OF_CANABLE][NUM_OF_MOTOR];

    float runtime = 0.;
    float thread_timer = 0.;
    float Timedata[4] = {0, 0, 0, 0};

    struct can_frame frame[4];
    Motor_state_t motor_state[4];
    Motor_state_t motor_states[4];
    COMMAND_MODE command_mode;
    JOINT *joint;
    sensor_msgs::msg::Joy::SharedPtr joystick_data_;

    float vel_cmd[3] = {0, 0, 0};
    float mode_cmd[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    void publishMessage();
    int CAN_INIT(const std::string &can_interface_name);
    void READ_BUFFER();
    void READ_TORQUE_SPEED_ANGLE(int s, int canable, struct can_frame frame);
    void EMERGENCY_STOP();
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg); // 
    void Check_State();
    void ComputeTorque();

    float func_1_cos(float t, float init, float final, float T);
    float dt_func_1_cos(float t, float init, float final, float T);

    Motor_state_t Get_motor_state(int i) // num of canable : 3~8
    {
        return motor_state[i];
    }

    void GetState();

    pthread_t Thread_init(void *(*thread_func)(void *), void *arg, int core, int policy, int priority);

    void *pthread_main();
};

#endif // MAIN_ACTIVE_NODE_HPP
