#include "MAIN.h"

int main(int argc, char **argv)
{
  printf("Init main ...\n");
  sleep(1);

  printf("System Initialization...\n");
  sleep(1);

  printf("Create Thread ...\n");
  for (int i = 3; i > 0; --i)
  {
    printf(" %d...\n", i);
    sleep(1);
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainActiveNode>()); // spin node
  rclcpp::shutdown();                               // ROS2 shutdown
  std::cout << "rclcpp::shutdown done" << std::endl;

  return 0;
}

MainActiveNode::MainActiveNode() : Node("Main_Active")
{
  int policy = SCHED_RR; // Set the desired scheduling policy : Round Robin
  int core = 2;          // Set the desired CPU core
  int priority = 98;     // Set the desired thread priority

  int arm_core = 1;      // Set the desired CPU core
  int arm_priority = 99; // Set the desired thread priority

  Publish_timer = this->create_wall_timer(10ms, std::bind(&MainActiveNode::publishMessage, this)); // publish Message 10ms interval
  joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MainActiveNode::joy_callback, this, std::placeholders::_1));

  // joint = static_cast<JOINT *>(std::malloc(JOINT_NUM * sizeof(JOINT))); // Dynamic memory allocation by multiplying memory size by the number of joints.
  joint = new JOINT[JOINT_NUM];

  // set can : 8 canables, 16 motors, 12 joints.
  cansock[0] = CAN_INIT("can4");
  cansock[1] = CAN_INIT("can5");
  cansock[2] = CAN_INIT("can6");
  cansock[3] = CAN_INIT("can7");

  cansock[4] = CAN_INIT("can10");
  cansock[5] = CAN_INIT("can11");
  cansock[6] = CAN_INIT("can12");
  cansock[7] = CAN_INIT("can13");

  Kp_X6 = 50.; // Flipper
  Kd_X6 = 1.;
  Kp_w1 = 0.5; // Manipulator wrist1~3, gripper
  Kd_w1 = 0.03;
  Kp_w2 = 1.;
  Kd_w2 = 0.03;
  Kp_w3 = 1.;
  Kd_w3 = 0.03;
  Kp_gr = 3.;
  Kd_gr = 0.03;

  // load angle, torque, speed once
  for (int i = 0; i < NUM_OF_CANABLE; i++)
  {
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      flag[i][j] = 1;
      previous_degree[i][j] = 0;
      angle_switch[i][j] = 0;

      std::string temp_topic = "/can" + std::to_string(i + 4) + "/motor" + std::to_string(j + 1) + "_temp";
      std::string tau_topic = "/can" + std::to_string(i + 4) + "/motor" + std::to_string(j + 1) + "_tau";
      std::string speed_topic = "/can" + std::to_string(i + 4) + "/motor" + std::to_string(j + 1) + "_speed";
      std::string torque_topic = "/can" + std::to_string(i + 4) + "/motor" + std::to_string(j + 1) + "_torque";
      std::string motor_degree_topic = "/can" + std::to_string(i + 4) + "/motor" + std::to_string(j + 1) + "_degree";

      temp_publishers[i][j] = this->create_publisher<std_msgs::msg::Int16>(temp_topic, QUEUE_SIZE);
      tau_publishers[i][j] = this->create_publisher<std_msgs::msg::Int16>(tau_topic, QUEUE_SIZE);
      speed_publishers[i][j] = this->create_publisher<std_msgs::msg::Int16>(speed_topic, QUEUE_SIZE);
      torque_publishers[i][j] = this->create_publisher<std_msgs::msg::Int16>(torque_topic, QUEUE_SIZE);
      motor_degree_publishers[i][j] = this->create_publisher<std_msgs::msg::Float32>(motor_degree_topic, QUEUE_SIZE);
    }
  }

  for (int i = 0; i < JOINT_NUM; i++) // JOINT_NUM = 6 -> 12
  {
    std::string degree_topic = "/JOINT_degree_" + std::to_string(i + 1);
    degree_publishers[i] = this->create_publisher<std_msgs::msg::Float32>(degree_topic, QUEUE_SIZE);
  }

  // Read encoder value
  // 차체 8개 팔 8개 NUM_OF_CANABLE = 8
  std::cout << "========== Current Q ==========" << std::endl;
  for (int i = 0; i < NUM_OF_CANABLE - 2; i++)
  {
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      Rmd.REQUEST_MOTOR_STATUS(cansock[i], j + 1, frame[i]);
      usleep(500); // 0.005 sec
      READ_TORQUE_SPEED_ANGLE(cansock[i], i, frame[i]);
    }
  }
  for (int i = 6; i < NUM_OF_CANABLE; i++) // NUM_OF_CANABLE = 8
  {
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      Rmd.REQUEST_MOTOR_STATUS(cansock[i], j + 1, frame[i]);
      usleep(500); // 0.005 sec
      READ_TORQUE_SPEED_ANGLE_X4(cansock[i], i, frame[i]);
    }
  }
  std::cout << "==============================" << std::endl;

  // Thread Init(Main)
  thread_flag = true;
  std::time_t t = std::time(0);
  std::tm *now = std::localtime(&t);
  std::string filename = "/home/kudos/ros2_ws/src/drokX/logs/log_" + std::to_string(now->tm_year + 1900) + std::to_string(now->tm_mon + 1) + std::to_string(now->tm_mday) + "__" + std::to_string(now->tm_hour) + std::to_string(now->tm_min) + std::to_string(now->tm_sec) + ".txt";
  std::cout << "filename : " << filename << std::endl;
  usleep(500);
  // start Main thread
  main_thread = Thread_init([](void *arg) -> void *
                            { return static_cast<MainActiveNode *>(arg)->pthread_main(); },
                            this, core, policy, priority);
  arm_thread = Thread_init([](void *arg) -> void *
                           { return static_cast<MainActiveNode *>(arg)->pthread_arm(); },
                           this, arm_core, policy, arm_priority);
  // write to txt file
  drokX_data = fopen(filename.c_str(), "w+");
}

void MainActiveNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!msg || msg->axes.empty() || msg->buttons.empty() || msg->axes.size() < 8 || msg->buttons.size() < 13)
  {
    RCLCPP_WARN(this->get_logger(), "Received empty or insufficient joy message.");
    return;
  }
  // Log the axes and buttons size for debugging
  // RCLCPP_INFO(this->get_logger(), "Received Joy message with axes size: %zu, buttons size: %zu", msg->axes.size(), msg->buttons.size());

  // === Wheel RPM Control ===
  // Joystick axes[1] (forward/backward) and axes[3] (left/right) are used for linear and angular velocity
  double wheel_base = 0.5;   // Distance between the wheels
  double wheel_radius = 0.1; // Radius of the wheel

  double linear_velocity = msg->axes[1];  // Forward/backward movement
  double angular_velocity = msg->axes[3]; // Left/right rotation

  // Compute individual wheel speeds
  double left_speed = (linear_velocity - (wheel_base / 2.0) * angular_velocity) / wheel_radius * 15000;
  double right_speed = (linear_velocity + (wheel_base / 2.0) * angular_velocity) / wheel_radius * 15000;

  // Set wheel RPM commands
  left_rpm_command_ = static_cast<int32_t>(left_speed);
  right_rpm_command_ = -static_cast<int32_t>(right_speed);

  // === Flipper Control ===
  // Map buttons to flipper indices
  int flipper_buttons_increase[4] = {0, 1, 2, 3};
  int flipper_axes_decrease[4][2] = {{7, -1}, {6, -1}, {7, 1}, {6, 1}};

  // Adjusted logic for buttons
  for (int i = 0; i < 4; i++)
  {
    if (msg->buttons[flipper_buttons_increase[i]] == 1)
    {
      if (i == 0 || i == 3) // For flippers 0 and 3, decrease instead of increase
      {
        joint[i].RefTargetPos -= 1.0 * D2R;          // Decrease by 1 degree
        if (joint[i].RefTargetPos < -90.0 * D2R * 2) // Limit to min angle (2:1 ratio)
          joint[i].RefTargetPos = -90.0 * D2R * 2;
      }
      else // For flippers 1 and 2, increase as usual
      {
        joint[i].RefTargetPos += 1.0 * D2R;         // Increase by 1 degree
        if (joint[i].RefTargetPos > 90.0 * D2R * 2) // Limit to max angle (2:1 ratio)
          joint[i].RefTargetPos = 90.0 * D2R * 2;
      }
      joint[i].IsMoving = true;
      joint[i].runtime = 0.0; // Reset trajectory time

      // Print the target angle to the terminal
      std::cout << "Flipper " << i << " Target Angle: " << joint[i].RefTargetPos * R2D << " degrees" << std::endl;
    }
  }

  // Adjusted logic for axes
  for (int i = 0; i < 4; i++)
  {
    int axis_index = flipper_axes_decrease[i][0];
    int axis_value = flipper_axes_decrease[i][1];
    if (static_cast<int>(msg->axes[axis_index]) == axis_value)
    {
      if (i == 0 || i == 3) // For flippers 0 and 3, increase instead of decrease
      {
        joint[i].RefTargetPos += 1.0 * D2R;         // Increase by 1 degree
        if (joint[i].RefTargetPos > 90.0 * D2R * 2) // Limit to max angle (2:1 ratio)
          joint[i].RefTargetPos = 90.0 * D2R * 2;
      }
      else // For flippers 1 and 2, decrease as usual
      {
        joint[i].RefTargetPos -= 1.0 * D2R;          // Decrease by 1 degree
        if (joint[i].RefTargetPos < -90.0 * D2R * 2) // Limit to min angle (2:1 ratio)
          joint[i].RefTargetPos = -90.0 * D2R * 2;
      }
      joint[i].IsMoving = true;
      joint[i].runtime = 0.0; // Reset trajectory time

      // Print the target angle to the terminal
      std::cout << "Flipper " << i << " Target Angle: " << joint[i].RefTargetPos * R2D << " degrees" << std::endl;
    }
  }

  // If no buttons are pressed, flippers stay in place
  for (int i = 0; i < 4; i++)
  {
    if (msg->buttons[flipper_buttons_increase[i]] == 0 &&
        static_cast<int>(msg->axes[flipper_axes_decrease[i][0]]) != flipper_axes_decrease[i][1])
    {
      // Do nothing, flippers hold their position
    }
  }

  // Manipulator control mode
  if (msg->buttons[4] == 1 && msg->buttons[5] == 1)
  {
    for (int i = 4; i < JOINT_NUM; i++) // JOINT_NUM = 12
    {
      joint[i].IsMoving = true;
      runtime = 0.0; // 팔 동작 runtime 리셋
    }
    // RCLCPP_INFO(this->get_logger(), ">>> Arm motion START triggered by joystick buttons.");
  }
  // Pick & Place when the buttons are pressed.
}

MainActiveNode::~MainActiveNode()
{
  // write log file done
  fclose(drokX_data);

  std::cout << "[drokX]destructor of MainActive Node " << std::endl;
  main_thread_exit = true;
  arm_thread_exit = true;

  // Waiting off the thread and join
  if (pthread_join(main_thread, NULL) != 0)
  {
    std::cerr << "Failed to join Main thread" << std::endl;
  }
  if (pthread_join(arm_thread, NULL) != 0)
  {
    std::cerr << "Failed to join Arm thread" << std::endl;
  }

  for (int i = 0; i < NUM_OF_CANABLE; i++)
  {
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j + 1, 0, frame[i]);
      std::cout << "[" << i + 3 << "-" << j + 1 << "]"
                << "Motor Stop... " << std::endl;
      Rmd.WRITE_MOTOR_OFF(cansock[i], j + 1, frame[i]);

      while (read(cansock[i], &frame[i], sizeof(struct can_frame)) > 0) // clear the Canable buffer
      {
        std::cout << "clear buffer ..." << std::endl;
      }
    }
  }
  // close socket can
  close(cansock[0]);
  close(cansock[1]);
  close(cansock[2]);
  close(cansock[3]);

  close(cansock[4]);
  close(cansock[5]);
  close(cansock[6]);
  close(cansock[7]);

  // 할당된 메모리 해제
  // std::free(joint);
  delete[] joint;
  joint = nullptr; // 선택 사항: 댕글링 포인터 방지

  std::cout << "Close Socket CAN " << std::endl;
}

int MainActiveNode::CAN_INIT(const std::string &can_interface_name)
{
  int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket: %s", std::strerror(errno));
    perror("Socket");
    return 1; // return 1: success or warn, -1 : failed
  }
  struct ifreq ifr;                               // InterFace REQuest
  std::string ifname = can_interface_name;        // can4~7, can10~13
  std::strcpy(ifr.ifr_name, ifname.c_str());      // STRing CoPY
  if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) == -1) // Input Output ConTroL
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to retrieve CAN interface index: %s", std::strerror(errno));
    perror("IOCTL");
    return 1;
  }
  struct sockaddr_can addr; // socket binding
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket: %s", std::strerror(errno));
    perror("Bind");
    return 1;
  }
  return socket_fd; // File Descriptor
}

pthread_t MainActiveNode::Thread_init(void *(*thread_func)(void *), void *arg, int core, int policy, int priority)
{
  struct sched_param param;
  pthread_attr_t attr;
  pthread_t thread;
  int ret;

  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
  {
    std::cerr << "mlockall failed" << std::endl;
    exit(-2);
  }
  /* Initialize pthread attributes (default values) */
  ret = pthread_attr_init(&attr);
  if (ret)
  {
    std::cerr << "init pthread attributes failed" << std::endl;
    return 0;
  }
  /* Set a specific stack size */
  // size_t stack_size = 1024 * 1024 * 4;
  ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN); // stack_size
  if (ret)
  {
    std::cerr << "pthread setstacksize failed" << std::endl;
    return 0;
  }
  /* Create a pthread with specified attributes */
  ret = pthread_create(&thread, &attr, thread_func, arg);
  if (ret)
  {
    std::cerr << "create pthread failed" << std::endl;
    return 0;
  }
  param.sched_priority = priority;
  if (pthread_setschedparam(thread, policy, &param))
  {
    std::lock_guard<std::mutex> lock(_mtx); // unlocks when lock destroyed
    return 0;
  }
  else
  {
    std::lock_guard<std::mutex> lock(_mtx);
    std::cout << "Thread priority scheduling set to " << priority << std::endl;
  }
  cpu_set_t _cpuset; // Define CPU set variable
  int nproc = sysconf(_SC_NPROCESSORS_ONLN);
  if (nproc < core)
  {
    core = nproc;
  }
  CPU_ZERO(&_cpuset);      // Initialize CPU set
  CPU_SET(core, &_cpuset); // Add 'core' to the set
  ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &_cpuset);
  if (ret != 0)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    std::cerr << "Failed to set Thread Affinity : " << std::strerror(errno) << std::endl;
    exit(0);
    return 0;
  }
  else
  {
    std::lock_guard<std::mutex> lock(_mtx);
    std::cout << "Thread Affinity set to CPU " << core << std::endl;
  }
  return thread;
}

void *MainActiveNode::pthread_main()
{
  struct period_info pinfo;
  pinfo.period_ns = ns_TenMilSec; // 10ms
  usleep(500);
  clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
  auto thread_start = std::chrono::high_resolution_clock::now();

  //------------------------------------------------------------//
  while ((!main_thread_exit))
  {
    pthread_testcancel();
    clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_1);

    // === Read Motor States === NUM_OF_CANABLE - 4 == 8 - 4 == 4;
    for (int i = 0; i < 4; i++) // For vehicle control: CAN IDs 4, 5, 6, 7
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++) // NUM_OF_MOTOR => 2
      {
        READ_TORQUE_SPEED_ANGLE(cansock[i], i, frame[i]);
      }
    }
    GetState();

    // === Wheel RPM Control ===
    for (int i = 1; i <= 2; i++) // CAN IDs 5, 6 correspond to index 1~2
    {
      int left_motorID = 1;  // motorID1 controls left RPM
      int right_motorID = 2; // motorID2 controls right RPM
      // Control the right wheel RPM on the current CANable
      Rmd.RPM_CLOSED_LOOP_CONTROL(cansock[i], right_motorID, right_rpm_command_, frame[i]);
      // Control the left wheel RPM on the current CANable
      Rmd.RPM_CLOSED_LOOP_CONTROL(cansock[i], left_motorID, left_rpm_command_, frame[i]);
    }

    // === Flipper Control ===
    // Update current positions and velocities
    joint[0].CurrentPos = motor_state[0].theta[0];
    joint[0].CurrentVel = motor_state[0].speed[0];
    joint[1].CurrentPos = motor_state[0].theta[1];
    joint[1].CurrentVel = motor_state[0].speed[1];
    joint[2].CurrentPos = motor_state[3].theta[0];
    joint[2].CurrentVel = motor_state[3].speed[0];
    joint[3].CurrentPos = motor_state[3].theta[1];
    joint[3].CurrentVel = motor_state[3].speed[1];

    // Control each flipper independently
    for (int i = 0; i < 4; i++)
    {
      if (joint[i].IsMoving)
      {
        if (joint[i].runtime == 0.0)
        {
          joint[i].InitPos = joint[i].CurrentPos;
          joint[i].InitVel = joint[i].CurrentVel;
        }
        // 1-cos trajectory over 3000 ms
        if (joint[i].runtime < 3000.0)
        {
          joint[i].RefPos = func_1_cos(joint[i].runtime, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
          joint[i].RefVel = dt_func_1_cos(joint[i].runtime, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
          joint[i].runtime += (pinfo.period_ns / 1000000.);
        }
        else
        {
          joint[i].RefPos = joint[i].RefTargetPos;
          joint[i].RefVel = 0.0;
          joint[i].IsMoving = false; // Movement complete
        }
      }
      else
      {
        // Hold target position
        joint[i].RefPos = joint[i].RefTargetPos;
        joint[i].RefVel = 0.0;
      }
    }

    // Compute torques using PD control
    ComputeTorque();

    // Apply torque commands to flippers
    // For front flippers (CAN 4)
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[0], j + 1, tau[0][j], frame[0]);
    }
    // For rear flippers (CAN 7)
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[3], j + 1, tau[3][j], frame[3]);
    }

    // Wait for the next control cycle
    pinfo.next_period.tv_nsec += pinfo.period_ns;
    while (pinfo.next_period.tv_nsec >= ns_OneSec)
    {
      pinfo.next_period.tv_sec++;
      pinfo.next_period.tv_nsec -= ns_OneSec;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo.next_period, NULL);
    clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_2);

    auto thread_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> thread_time = thread_end - thread_start;
    thread_timer = thread_time.count();
    thread_start = std::chrono::high_resolution_clock::now();
  }
  return NULL;
}

void *MainActiveNode::pthread_arm()
{
  struct period_info pinfo;
  //* Set Main Thread Period 주기 설정
  pinfo.period_ns = ns_TenMilSec; // 2ms ns_TwoMilSec ns_FiveMilSec
  usleep(500);
  clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
  auto thread_start = std::chrono::high_resolution_clock::now();

  //* -------------------------------------------------------------------------Thread Loop-------------------------------------------------------
  while ((!arm_thread_exit))
  {
    pthread_testcancel();                                  // make thread cancel point to saftey exit thread
    clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_1); // time1 load
    // input loop function
    // Read buffer - motor state data : torque, speed, angle
    for (int i = NUM_OF_CANABLE - 4; i < NUM_OF_CANABLE; i++) // 10~13
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        if (i < NUM_OF_CANABLE - 2)
          READ_TORQUE_SPEED_ANGLE(cansock[i], i, frame[i]); // Read for first two CAN channels
        else
          READ_TORQUE_SPEED_ANGLE_X4(cansock[i], i, frame[i]); // Read for remaining CAN channels
      }
    }
    GetState();

    { // Lock the mutex only when accessing shared resources
      std::lock_guard<std::mutex> lock(_mtx);
      // Update joint positions and velocities // joint4~7, joint8~11
      joint[4].CurrentPos = motor_state[4].theta[0]; // base
      joint[4].CurrentVel = motor_state[4].speed[0];

      joint[5].CurrentPos = motor_state[5].theta[0]; // shoulder
      joint[5].CurrentVel = motor_state[5].speed[0];
      joint[6].CurrentPos = motor_state[5].theta[1]; // -shoulder
      joint[6].CurrentVel = motor_state[5].speed[1];

      joint[7].CurrentPos = motor_state[4].theta[1]; // elbow
      joint[7].CurrentVel = motor_state[4].speed[1];

      // X6 X4

      joint[8].CurrentPos = motor_state[6].theta[0]; // wrist1
      joint[8].CurrentVel = motor_state[6].speed[0];
      joint[9].CurrentPos = motor_state[6].theta[1]; // wrist2
      joint[9].CurrentVel = motor_state[6].speed[1];
      joint[10].CurrentPos = motor_state[7].theta[0]; // wrist3
      joint[10].CurrentVel = motor_state[7].speed[0];
      joint[11].CurrentPos = motor_state[7].theta[1]; // gripper
      joint[11].CurrentVel = motor_state[7].speed[1];
      //=============2025. 1. 21. Tue.=============//

      if (joint[4].IsMoving || joint[5].IsMoving || joint[6].IsMoving || joint[7].IsMoving ||
          joint[10].IsMoving || joint[11].IsMoving)
      {
        if (runtime == 0.0)
        {
          joint[4].RefTargetPos = 0.; // can10 motor 1 : base
          joint[4].InitPos = joint[4].CurrentPos;
          joint[4].InitVel = joint[4].CurrentVel;

          joint[5].RefTargetPos = -30 * D2R; // can11 motor 1 2
          joint[5].InitPos = joint[5].CurrentPos;
          joint[5].InitVel = joint[5].CurrentVel;
          joint[6].RefTargetPos = 30 * D2R;
          joint[6].InitPos = joint[6].CurrentPos;
          joint[6].InitVel = joint[6].CurrentVel;

          joint[7].RefTargetPos = 90 * D2R; // for yaw to pitch of joint[7]
          joint[7].InitPos = motor_state[4].theta[1];
          joint[7].InitVel = motor_state[4].speed[1];

          joint[10].RefTargetPos = -15 * D2R; // pitch
          joint[10].InitPos = motor_state[7].theta[0];
          joint[10].InitVel = motor_state[7].speed[0];

          joint[11].RefTargetPos = -300 * D2R; // opening
          joint[11].InitPos = motor_state[7].theta[1];
          joint[11].InitVel = motor_state[7].speed[1];
        }

        if (runtime < 3000.0)
        {
          for (int i = 4; i <= 7; i++)
          {
            joint[i].RefPos = func_1_cos(runtime, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
            joint[i].RefVel = dt_func_1_cos(runtime, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
          }
          joint[10].RefPos = func_1_cos(runtime, joint[10].InitPos, joint[10].RefTargetPos, 3000.0);
          joint[10].RefVel = dt_func_1_cos(runtime, joint[10].InitPos, joint[10].RefTargetPos, 3000.0);

          joint[11].RefPos = func_1_cos(runtime, joint[11].InitPos, joint[11].RefTargetPos, 3000.0);
          joint[11].RefVel = dt_func_1_cos(runtime, joint[11].InitPos, joint[11].RefTargetPos, 3000.0);
        }
        else if (runtime == 3000.0)
        {
          for (int i = 4; i < 11; i++)
          {
            joint[i].RefPos = joint[i].RefTargetPos;
            joint[i].RefVel = 0.0;
          }
          joint[11].RefTargetPos = 300 * D2R; // gripping motion
          joint[11].InitPos = motor_state[7].theta[1];
          joint[11].InitVel = motor_state[7].speed[1];
        }
        else if (3000.0 < runtime && runtime < 6000.0)
        {
          joint[11].RefPos = func_1_cos(runtime - 3000, joint[11].InitPos, joint[11].RefTargetPos, 3000.0);
          joint[11].RefVel = dt_func_1_cos(runtime - 3000, joint[11].InitPos, joint[11].RefTargetPos, 3000.0);
        }
        else if (runtime == 6000.0)
        {
          joint[4].RefTargetPos = -0.;
          joint[4].InitPos = joint[4].CurrentPos;
          joint[4].InitVel = joint[4].CurrentVel;

          joint[5].RefTargetPos = 30 * D2R;
          joint[5].InitPos = joint[5].CurrentPos;
          joint[5].InitVel = joint[5].CurrentVel;
          joint[6].RefTargetPos = -30 * D2R;
          joint[6].InitPos = joint[6].CurrentPos;
          joint[6].InitVel = joint[6].CurrentVel;

          joint[7].RefTargetPos = -90 * D2R;
          joint[7].InitPos = motor_state[4].theta[1];
          joint[7].InitVel = motor_state[4].speed[1];

          joint[8].RefTargetPos = 15 * D2R;
          joint[8].InitPos = motor_state[6].theta[0];
          joint[8].InitVel = motor_state[6].speed[0];
        }
        else if (6000.0 < runtime && runtime < 9000.0)
        {
          for (int i = 4; i < 9; i++)
          {
            joint[i].RefPos = func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
            joint[i].RefVel = dt_func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
          }
        }
        if (runtime > 9000.0)
        {
          for (int i = 4; i < 12; i++)
          {
            joint[i].RefPos = joint[i].RefTargetPos;
            joint[i].RefVel = 0.;      // = joint[i].RefVel;
            joint[i].IsMoving = false; // Disable after 9000ms
          }
        }
      }
      // else
      // {
      //   // IsMoving = false이면, 아무런 동작을 하지 않음
      // }

      // std::cout << "Joint_Ref_POS" << joint[6].RefPos * R2D << std::endl;
      // std::cout << "Joint_Cur_POS" << joint[6].CurrentPos * R2D << std::endl;
      // std::cout << "Joint_RefTarget_POS" << joint[6].RefTargetPos * R2D << std::endl;
      // std::cout << "Joint_Init_POS" << joint[6].InitPos * R2D << std::endl;
      // std::cout << "RUNTIME" << runtime << std::endl;

      // std::cout << "Runtime: " << runtime << " ms" << std::endl;
      // for (int i = 4; i < 12; i++)
      // {
      //   std::cout << "Joint " << i << " RefPos: " << joint[i].RefPos * R2D
      //             << ", CurrentPos: " << joint[i].CurrentPos * R2D
      //             << ", RefTargetPos: " << joint[i].RefTargetPos * R2D
      //             << ", InitPos: " << joint[i].InitPos * R2D << std::endl;
      // }
    }
    // calc torque(real range: -30~30A) Mapping to -2000~2000
    ComputeTorque();
    for (int i = 4; i < NUM_OF_CANABLE; i++) // NUM_OF_CANABLE = 8
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j + 1, tau[i][j], frame[i]);
      }
    }

    runtime = (runtime + (pinfo.period_ns / 1000000.)); //-> runtime += thread period
    pinfo.next_period.tv_nsec += pinfo.period_ns;
    while (pinfo.next_period.tv_nsec >= ns_OneSec) // 1000000000 nsec = 1sec
    {
      /* timespec nsec overflow */
      pinfo.next_period.tv_sec++;
      pinfo.next_period.tv_nsec -= ns_OneSec; // clear nanosec
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo.next_period, NULL);
    clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_2); // tim2 load

    // std::cout << " ============================= " << std::endl;
    // std::cout << "Pthread_time: "<< (double)(pinfo.current_time_2.tv_nsec - pinfo.current_time_1.tv_nsec) / ns_OneMilSec << " ms " << std::endl;
    // std::cout << " ============================= " << std::endl;
    auto thread_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> thread_time = thread_end - thread_start;
    thread_timer = thread_time.count();
    // std::cout << "end-Thread Time: " << thread_time.count() << " ms" << std::endl;
    // printf("end-Thread Time: %.5f ms\n", thread_time.count());
    thread_start = std::chrono::high_resolution_clock::now();

    // // -----------logging part-----------//
    // fprintf(drokX_data, "%f \t", runtime);
    // for (int i = 0; i < NUM_OF_CANABLE; i++)
    // {
    //   for (int j = 0; j < NUM_OF_MOTOR; j++)
    //   {
    //     fprintf(drokX_data, "%f \t", motor_state[i].theta[j]);
    //   }
    // }
    // // std::cout<<"mt done"<<std::endl;
    // for (int i = 0; i < NUM_OF_CANABLE; i++)
    // {
    //   for (int j = 0; j < NUM_OF_MOTOR; j++)
    //   {
    //     fprintf(drokX_data, "%f \t", motor_state[i].speed[j]);
    //   }
    // }
    // // std::cout<<"ms done"<<std::endl;
    // for (int i = 0; i < NUM_OF_CANABLE; i++)
    // {
    //   for (int j = 0; j < NUM_OF_MOTOR; j++)
    //   {
    //     fprintf(drokX_data, "%f \t", (float)(tau[i][j]));
    //   }
    // }
    // // std::cout<<"tq done"<<std::endl;
    // for (int i = 0; i < NUM_OF_CANABLE; i++)
    // {
    //   for (int j = 0; j < NUM_OF_MOTOR; j++)
    //   {
    //     fprintf(drokX_data, "%f \t", motor_state[i].torque[j]);
    //   }
    // }
    // fprintf(drokX_data, "%f \t", thread_timer);
    // // fprintf temperature
    // for (int i = 0; i < NUM_OF_CANABLE; i++)
    // {
    //   for (int j = 0; j < NUM_OF_MOTOR; j++)
    //   {
    //     fprintf(drokX_data, "%f \t", motor_state[i].temp[j]);
    //   }
    // }
    // fprintf(drokX_data, "\n");
    // //-----------logging part-----------//

  } // end of while loop

  return NULL;
}

void MainActiveNode::GetState()
{
  for (int i = 0; i < NUM_OF_CANABLE; i++)
  {
    motor_states[i] = Get_motor_state(i);
  }
}

float MainActiveNode::func_1_cos(float t, float init, float final, float T)
{
  float des;
  des = init + 0.5 * (final - init) * (1.0 - cos(PI * t / T));
  return des;
}

float MainActiveNode::dt_func_1_cos(float t, float init, float final, float T)
{
  float des_v;
  des_v = 0.5 * (final - init) * (PI / T) * (sin(PI * t / T));
  return des_v;
}

void MainActiveNode::ComputeTorque()
{
  // For front flippers (CAN 4)
  for (int j = 0; j < 2; j++)
  {
    tau[0][j] = (float)((Kp_X6 * (joint[j].RefPos - joint[j].CurrentPos) + Kd_X6 * (joint[j].RefVel - joint[j].CurrentVel)) * TauMap);
    if (tau[0][j] > 62.5)
      tau[0][j] = 62.5;
    if (tau[0][j] < -62.5)
      tau[0][j] = -62.5;
  }
  // For rear flippers (CAN 7)
  for (int j = 0; j < 2; j++)
  {
    int idx = j + 2; // Indexing joint[2] and joint[3]
    tau[3][j] = (float)((Kp_X6 * (joint[idx].RefPos - joint[idx].CurrentPos) + Kd_X6 * (joint[idx].RefVel - joint[idx].CurrentVel)) * TauMap);
    if (tau[3][j] > 62.5)
      tau[3][j] = 62.5;
    if (tau[3][j] < -62.5)
      tau[3][j] = -62.5;
  }
  // For manipulator & gripper
  for (int j = 0; j < 2; j++) // base and elbow (CAN 10)
  {
    int idx = j + 4; // Indexing joint[4] and joint[5]
    tau[4][j] = (float)((Kp_X6 * (joint[idx].RefPos - joint[idx].CurrentPos) + Kd_X6 * (joint[idx].RefVel - joint[idx].CurrentVel)) * TauMap);
    // Saturate torque
    if (tau[4][j] > 62.5)
      tau[4][j] = 62.5;
    if (tau[4][j] < -62.5)
      tau[4][j] = -62.5;
  }
  for (int j = 0; j < 2; j++) // shoulder1, 2 (CAN 11)
  {
    int idx = j + 6; // Indexing joint[6] and joint[7]
    tau[5][j] = (float)((Kp_X6 * (joint[idx].RefPos - joint[idx].CurrentPos) + Kd_X6 * (joint[idx].RefVel - joint[idx].CurrentVel)) * TauMap);
    // Saturate torque
    if (tau[5][j] > 62.5)
      tau[5][j] = 62.5;
    else if (tau[5][j] < -62.5)
      tau[5][j] = -62.5;
  }

  // Indexing joint[8]~joint[11] (CAN12, 13)
  tau[6][0] = (Kp_w1 * (joint[8].RefPos - joint[8].CurrentPos) + Kd_w1 * (joint[8].RefVel - joint[8].CurrentVel)) * (TauMap / 14); // TauMap_X4 needed
  tau[6][1] = (Kp_w2 * (joint[9].RefPos - joint[9].CurrentPos) + Kd_w2 * (joint[9].RefVel - joint[9].CurrentVel)) * (TauMap / 14);
  tau[7][0] = (Kp_w3 * (joint[10].RefPos - joint[10].CurrentPos) + Kd_w3 * (joint[10].RefVel - joint[10].CurrentVel)) * (TauMap / 14);
  tau[7][1] = (Kp_gr * (joint[11].RefPos - joint[11].CurrentPos) + Kd_gr * (joint[11].RefVel - joint[11].CurrentVel)) * (TauMap / 14);

  if (tau[6][0] > 5.) // 62.5
    tau[6][0] = 5.;
  else if (tau[6][0] < -5.)
    tau[6][0] = -5.;
  if (tau[6][1] > 5.) // 62.5
    tau[6][1] = 5.;
  else if (tau[6][1] < -5.)
    tau[6][1] = -5.;
  if (tau[7][0] > 5.) // 62.5
    tau[7][0] = 5.;
  else if (tau[7][0] < -5.)
    tau[7][0] = -5.;
  if (tau[7][1] > 5.) // 62.5
    tau[7][1] = 5.;
  else if (tau[7][1] < -5.)
    tau[7][1] = -5.;
}

void MainActiveNode::READ_TORQUE_SPEED_ANGLE(int s, int canable, struct can_frame frame)
{ // for X-6 motor 8EA -> 4 for flippers, 4 for manipulator
  fd_set read_fds;
  struct timeval timeout;
  int ret;

  int flags = fcntl(s, F_GETFL, 0);
  fcntl(s, F_SETFL, flags | O_NONBLOCK);

  FD_ZERO(&read_fds);
  FD_SET(s, &read_fds);

  timeout.tv_sec = 0;
  timeout.tv_usec = 500;

  ret = select(s + 1, &read_fds, NULL, NULL, &timeout);
  if (ret < 0)
  {
    perror("Error in select");
    return;
  }
  if (FD_ISSET(s, &read_fds))
  {
    int nbytes = read(s, &frame, sizeof(struct can_frame));
    if (nbytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        // printf("No data available.\n");
        return;
      }
      else
      {
        perror("Read");
        return;
      }
    }
    else if (nbytes == sizeof(struct can_frame))
    {
      int motor_id = frame.can_id - 0x140;

      temp = (frame.data[1]);
      torque = (frame.data[2] << 0) | (frame.data[3] << 8);
      speed = (frame.data[4] << 0) | (frame.data[5] << 8);
      pulse = (frame.data[6] << 0) | (frame.data[7] << 8);
      degree = (float)pulse * pulse_to_degree; // pulse to degree : 2 ^-16
      motor_state[canable].temp[motor_id - 1] = temp;

      if (flag[canable][motor_id - 1] == 1)
      {
        previous_degree[canable][motor_id - 1] = degree;
        flag[canable][motor_id - 1] = 0;
      }

      if (degree - previous_degree[canable][motor_id - 1] < -5)
        angle_switch[canable][motor_id - 1] += 1;
      else if (degree - previous_degree[canable][motor_id - 1] > 5)
        angle_switch[canable][motor_id - 1] -= 1;

      output_degree = angle_switch[canable][motor_id - 1] * 10. + degree; // +-20

      if (output_degree > 360)
      {
        angle_switch[canable][motor_id - 1] = 0;
      }
      else if (output_degree < -360)
      {
        angle_switch[canable][motor_id - 1] = 0;
      }

      previous_degree[canable][motor_id - 1] = degree;

      if (frame.can_id == 0x141)
      {
        if (canable == 0) // FL
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 36;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
        else if (canable == 3) // RL
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 36;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
        else if (canable == 4) // base
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 36;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
        else if (canable == 5) // shoulder1
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 36;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
      }
      else if (frame.can_id == 0x142)
      {
        if (canable == 0) // FR
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 36;
          motor_state[canable].theta[1] = (output_degree)*D2R;
        }
        else if (canable == 3) // RR
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 36;
          motor_state[canable].theta[1] = (output_degree)*D2R;
        }
        else if (canable == 4) // elbow
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 36;
          motor_state[canable].theta[1] = (output_degree)*D2R;
        }
        else if (canable == 5) // shoulder2
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 36;
          motor_state[canable].theta[1] = (output_degree)*D2R;
        }
      }
    }
  }
}

void MainActiveNode::READ_TORQUE_SPEED_ANGLE_X4(int s, int canable, struct can_frame frame)
{
  fd_set read_fds;
  struct timeval timeout;
  int ret;

  // Set the socket to non-blocking mode
  int flags = fcntl(s, F_GETFL, 0);
  fcntl(s, F_SETFL, flags | O_NONBLOCK);

  // 파일 디스크립터 셋 초기화
  FD_ZERO(&read_fds);   // init read_fds
  FD_SET(s, &read_fds); // set fd 1

  // 타임아웃 설정
  timeout.tv_sec = 0;
  timeout.tv_usec = 500;

  // select 함수를 사용하여 비블로킹으로 대기
  ret = select(s + 1, &read_fds, NULL, NULL, &timeout);
  if (ret < 0)
  {
    perror("Error in select");
    return;
  }
  if (FD_ISSET(s, &read_fds))
  {
    int nbytes = read(s, &frame, sizeof(struct can_frame));
    if (nbytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        // 데이터가 없어서 블로킹이 발생하지 않은 경우
        printf("No data available.\n");
        return;
      }
      else
      {
        perror("Read");
        return;
      }
    }
    else if (nbytes == sizeof(struct can_frame))
    {
      int motor_id = frame.can_id - 0x140;

      // std::cout << "CANABLE : " << canable << std::hex << "0x" << frame.can_id << std::endl;
      temp = (frame.data[1]);

      torque = (frame.data[2] << 0) | (frame.data[3] << 8);

      speed = (frame.data[4] << 0) | (frame.data[5] << 8);

      pulse = (frame.data[6] << 0) | (frame.data[7] << 8);
      degree = (float)pulse * pulse_to_degree_x4; // pulse to degree by accuracy : 2 ^-14
      motor_state[canable].temp[motor_id - 1] = temp;
      // std::cout << "temp : " << std::dec << temp << std::endl;
      // std::cout << "torque : " << std::dec << torque << std::endl;
      // std::cout << "speed : " << std::dec << speed << std::endl;
      // std::cout << "Motor ID : " << motor_id << std::endl;
      // std::cout << "PULSE    : " << pulse << std::endl;
      // std::cout << "degree   : " << std::dec << degree << std::endl;
      // std::cout << "previous_degree : " << std::dec << previous_degree[canable][motor_id - 1] << std::endl;
      // std::cout << "angle_switch   : " << std::dec << angle_switch[canable][motor_id - 1] << std::endl;

      ///========================= Single turn : -360 ~ 360 ===============================///

      if (flag[canable][motor_id - 1] == 1)
      {
        previous_degree[canable][motor_id - 1] = degree;
        flag[canable][motor_id - 1] = 0;
      }

      if (degree - previous_degree[canable][motor_id - 1] < -30)
        angle_switch[canable][motor_id - 1] += 1;
      else if (degree - previous_degree[canable][motor_id - 1] > 30)
        angle_switch[canable][motor_id - 1] -= 1;

      /*if (angle_switch[canable][motor_id - 1] > 9)
        angle_switch[canable][motor_id - 1] -= 10;
      else if (angle_switch[canable][motor_id - 1] < -9)
        angle_switch[canable][motor_id - 1] += 10;*/

      output_degree = angle_switch[canable][motor_id - 1] * 60. + degree; // +-30

      if (output_degree > 360)
      {
        angle_switch[canable][motor_id - 1] = 0;
      }
      else if (output_degree < -360)
      {
        angle_switch[canable][motor_id - 1] = 0;
      }

      previous_degree[canable][motor_id - 1] = degree;

      ///========================= Single turn  : -360 ~ 360===============================///

      if (frame.can_id == 0x141)
      {
        if (canable == 6)
        {
          motor_state[canable].torque[0] = -torque;
          motor_state[canable].speed[0] = -speed * D2R / 6;
          motor_state[canable].theta[0] = -(output_degree)*D2R;
        }
        // RR
        else if (canable == 7)
        {
          motor_state[canable].torque[0] = -torque;
          motor_state[canable].speed[0] = -speed * D2R / 6;
          motor_state[canable].theta[0] = -(output_degree)*D2R;
        }
      }

      // LEFT  : 0,2
      // RIGHT : 1,3

      else if (frame.can_id == 0x142) // HIP Pitch
      {
        if (canable == 6)
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 6;
          motor_state[canable].theta[1] = (output_degree + 0) * D2R; //+74.25
        }
        // RR
        else if (canable == 7)
        {
          motor_state[canable].torque[1] = -torque;
          motor_state[canable].speed[1] = -speed * D2R / 6;
          motor_state[canable].theta[1] = -(output_degree - 0) * D2R; //-74.5
        }
      }

      previous_degree[canable][motor_id - 1] = degree;
      // std::cout << "Motor Theta[1] : " << motor_state[0].theta[1] * R2D << std::endl;
      // std::cout << "Motor Theta[2] : " << motor_state[1].theta[0] * R2D << std::endl;
      // std::cout << "=================================" << std::endl;
      // std::cout << "endangle_switch   : " << std::dec << angle_switch[canable][motor_id - 1] << std::endl;
    }
  }
}

void MainActiveNode::publishMessage()
{
  // Publish motor states
  for (int i = 0; i < NUM_OF_CANABLE; i++)
  {
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      auto msg_torque = std::make_unique<std_msgs::msg::Int16>();
      msg_torque->data = motor_state[i].torque[j];
      torque_publishers[i][j]->publish(std::move(msg_torque));

      auto msg_tau = std::make_unique<std_msgs::msg::Int16>();
      msg_tau->data = tau[i][j];
      tau_publishers[i][j]->publish(std::move(msg_tau));

      auto msg_speed = std::make_unique<std_msgs::msg::Int16>();
      msg_speed->data = motor_state[i].speed[j];
      speed_publishers[i][j]->publish(std::move(msg_speed));

      auto msg_degree = std::make_unique<std_msgs::msg::Float32>();
      msg_degree->data = motor_state[i].theta[j] * R2D;
      motor_degree_publishers[i][j]->publish(std::move(msg_degree));

      auto msg_temp = std::make_unique<std_msgs::msg::Int16>();
      msg_temp->data = motor_state[i].temp[j];
      temp_publishers[i][j]->publish(std::move(msg_temp));
    }
  }
}
