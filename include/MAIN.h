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
#include <sstream>
#include <map>

// ROS2 Libraries
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
#define NUM_OF_CANABLE 4 // 8 // 6 //
#define NUM_OF_MOTOR 2   //
// #define SERIAL 0
// #define BLUETOOTH 1
#define JOINT_NUM 4 // 7 // 8

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr int NUM_MOTORS = 8; // 16;
constexpr int QUEUE_SIZE = 20;

volatile bool main_thread_exit = false;
// volatile bool arm_thread_exit = false;
volatile bool thread_flag = true;
volatile bool time_check = false; ///
// bool cmd_vel_flag = true; // Initially true, will be disabled after 9000 ms.

int32_t left_rpm_command_;
int32_t right_rpm_command_;
float wheel_base = 0.35;  // 너비
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

    pthread_t Thread_init(void *(*thread_func)(void *), void *arg, int core, int policy, int priority);
    void *pthread_main();
    int find_physical_core();

private:
    struct Motor_state_t
    {
        float theta_org[2];
        float theta[2];
        float speed[2];
        float torque[2];
        float temp[2];
        int data_count[4] = {0, 0, 0, 0};
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

        bool IsMoving; // Indicates whether the joint is currently moving
        float runtime; // Tracks the runtime for the 1-cos trajectory

        // float FrictionCompensationTorque;

        // float RefInputCurrent;
        // float ActualInputCurrent;

        // float KpGain;
        // float KdGain;
    } JOINT;

    typedef enum
    {
        POSITION_COMMAND = 0,
        TORQUE_COMMAND,
        TEST,
        RPM_COMMAND
    } COMMAND_MODE;

    // Variables
    rclcpp::TimerBase::SharedPtr Publish_timer;
    pthread_t main_thread;
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

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

    int16_t torque_command_[4] = {0, 0, 0, 0};
    float torque_command_tmp[4] = {0, 0, 0, 0};
    int8_t temp = 0; ///
    int16_t torque = 0;
    int16_t speed = 0;
    int16_t pulse = 0;
    float pulse_to_degree = 0.00001525878 * 10.;
    ////// 1:9 => 40.; // 1:6 => 60.; // 1:36 => 10.; .00015258789
    float pulse_to_degree_x4 = 0.00006103515 * 60.;
    // 1:9 => 40.; // 1:6 => 60.; // 1:36 => 10.; .00366210937 // 즉 24배 차이.
    float degree = 0.;
    float previous_degree[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float output_degree = 0.;
    float Kp_FR, Kp_FL, Kp_BR, Kp_BL;
    float Kd_FR, Kd_FL, Kd_BR, Kd_BL;
    int angle_switch[NUM_OF_CANABLE][NUM_OF_MOTOR];
    int flag[NUM_OF_CANABLE][NUM_OF_MOTOR];
    // int tau[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float tau[NUM_OF_CANABLE][NUM_OF_MOTOR];
    int cansock[NUM_OF_CANABLE];
    int command;
    int err_count = 0;

    FILE *drokX_data = NULL;
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

    std::chrono::time_point<std::chrono::steady_clock> last_joystick_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> before_thread_time; ///

    void publishMessage();
    int CAN_INIT(const std::string &can_interface_name);
    void READ_BUFFER();
    void READ_TORQUE_SPEED_ANGLE(int s, int canable, struct can_frame frame);
    void READ_TORQUE_SPEED_ANGLE_X4(int s, int canable, struct can_frame frame);

    void EMERGENCY_STOP();
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void Check_State();
    void ComputeTorque();

    float func_1_cos(float t, float init, float final, float T);
    float dt_func_1_cos(float t, float init, float final, float T);

    Motor_state_t Get_motor_state(int i) // num of canable : 8
    {
        return motor_state[i]; // for each canable -> 2개씩 받아옴 motor_state[i].theta[j:cansock-1]
    }

    void GetState();

    void *pthread_main();

};

#endif // MAIN_ACTIVE_NODE_HPP
