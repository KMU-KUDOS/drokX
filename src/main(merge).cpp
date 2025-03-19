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
  yolo_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/yolo/coordinates_3D", 10, std::bind(&MainActiveNode::yolo_callback, this, std::placeholders::_1));
  cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MainActiveNode::cmd_vel_callback, this, std::placeholders::_1));
  torque_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("flipper_torque", 10, std::bind(&MainActiveNode::torque_callback, this, std::placeholders::_1));

  joint = static_cast<JOINT *>(std::malloc(JOINT_NUM * sizeof(JOINT))); // Dynamic memory allocation by multiplying memory size by the number of joints.

  // set can 4 5 6
  cansock[0] = CAN_INIT("can4");
  cansock[1] = CAN_INIT("can5");
  cansock[2] = CAN_INIT("can6");
  // cansock[3] = CAN_INIT("can7");

  cansock[3] = CAN_INIT("can10");
  cansock[4] = CAN_INIT("can11");
  cansock[5] = CAN_INIT("can12");
  cansock[6] = CAN_INIT("can13");
  // 7 canables, 13 motors

  Kp_FL = 50.;
  Kd_FL = 1.;

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

  for (int i = 0; i < JOINT_NUM; i++)
  {
    std::string degree_topic = "/JOINT_degree_" + std::to_string(i + 1);
    degree_publishers[i] = this->create_publisher<std_msgs::msg::Float32>(degree_topic, QUEUE_SIZE);
  }

  // Read encoder value 차체 6개(뒷 플리퍼 ㅂㅂ) 팔 7개(baseㅂㅂ) 캐너블 7개
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
  for (int i = NUM_OF_CANABLE - 2; i < NUM_OF_CANABLE; i++)
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

void MainActiveNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Convert cmd_vel to left and right RPM commands
  double wheel_base = 0.5;   // Set your robot's wheel base (distance between the two wheels)
  double wheel_radius = 0.1; // Set your robot's wheel radius

  double linear_velocity = msg->linear.x;
  double angular_velocity = msg->angular.z;

  double left_speed = (linear_velocity - (wheel_base / 2.0) * angular_velocity) / wheel_radius * 10000;
  double right_speed = (linear_velocity + (wheel_base / 2.0) * angular_velocity) / wheel_radius * 10000;

  left_rpm_command_ = static_cast<int32_t>(left_speed);
  right_rpm_command_ = -static_cast<int32_t>(right_speed);
}

void MainActiveNode::torque_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  for (int i = 0; i < 4; i++)
  {
    torque_command_[i] = static_cast<int32_t>(msg->data[i]);
  }
}

void MainActiveNode::yolo_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_mtx);
  x_pixel = msg->data[0];
  y_pixel = msg->data[1];
  z_depth = msg->data[2];

  if (0.00001f < z_depth && z_depth < 0.9f) // Check if z_depth is greater than 0 and x_pixel is within or outside the threshold
  {
    if (!global_planar_flag)
    {                            // set rpm zero here. stop when z_depth == certain_value;
      global_planar_flag = true; // If x_pixel is within the threshold, activate planar_flag directly
      runtime = 0.0;             // Reset runtime when planar_flag is activated
    }
  }
  else
  {
    std::cerr << "Invalid z_depth: " << z_depth << std::endl;
  }
  std::cout << "YOLO callback executed: x_pixel = " << x_pixel << ", y_pixel = " << y_pixel << ", z_depth = " << z_depth << std::endl;
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
  // close(cansock[3]);

  close(cansock[3]);
  close(cansock[4]);
  close(cansock[5]);
  close(cansock[6]);

  // 할당된 메모리 해제
  std::free(joint);

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
  std::string ifname = can_interface_name;        // can4~7
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
  size_t stack_size = 1024 * 1024 * 4; ///
  ret = pthread_attr_setstacksize(&attr, stack_size);
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

    // Read buffer
    for (int i = 0; i < 3; i++) // For vehicle control: CAN IDs 4, 5, 6
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++) // NUM_OF_MOTOR => 2
      {
        READ_TORQUE_SPEED_ANGLE(cansock[i], j, frame[i]);
      }
    }
    GetState(); ///

    // Check joystick timeout
    // auto current_time = std::chrono::steady_clock::now();
    // auto time_since_last_joystick = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_joystick_time_).count();
    // if (time_since_last_joystick > 300)
    // {
    //   // Apply 0 RPM to all motors if no joystick input for 0.3 seconds
    //   for (int i = 0; i < 4; i++)
    //   {
    //     for (int j = 0; j < 2; j++)
    //     {
    //       Rmd.RPM_CLOSED_LOOP_CONTROL(cansock[i], j, 0, frame[i]);
    //     }
    //   }
    // }

    // Control logic for wheels
    for (int i = 1; i < 3; i++) // CAN IDs 4, 5 correspond to index 1~2
    {
      int left_motorID = 1;  // motorID1 controls left RPM
      int right_motorID = 2; // motorID2 controls right RPM

      // Control the right wheel RPM on the current CANable
      Rmd.RPM_CLOSED_LOOP_CONTROL(cansock[i], right_motorID, right_rpm_command_, frame[i]);

      // Control the left wheel RPM on the current CANable
      Rmd.RPM_CLOSED_LOOP_CONTROL(cansock[i], left_motorID, left_rpm_command_, frame[i]);
    }

    // Handle flipper torque control
    for (int i = 0; i < 4; i++)
    {
      // if(i==2)i++;
      int canable = (i < 2) ? 0 : 3; // Front flippers use can4, Back flippers use can7
      int motor = (i % 2) + 1;       // Left flippers use motor 1, Right flippers use motor 2
      Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[canable], motor, torque_command_[i], frame[canable]);
    }

    // i = 0: canable = 0(can4), motor = 1 (앞쪽 왼쪽 플리퍼)
    // i = 1: canable = 0, motor = 2 (앞쪽 오른쪽 플리퍼)
    // i = 2: canable = 3(can7), motor = 1 (뒤쪽 왼쪽 플리퍼)
    // i = 3: canable = 3, motor = 2 (뒤쪽 오른쪽 플리퍼)

    runtime = (runtime + (pinfo.period_ns / 1000000.));
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
  pinfo.period_ns = ns_TwoMilSec; // 2ms ns_TwoMilSec ns_FiveMilSec
  usleep(500);
  clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
  auto thread_start = std::chrono::high_resolution_clock::now();

  {
    std::lock_guard<std::mutex> lock(_mtx);
    planar_flag = global_planar_flag;
  }

  //* -------------------------------------------------------------------------Thread Loop-------------------------------------------------------
  while ((!arm_thread_exit))
  {
    pthread_testcancel();                                  // make thread cancel point to saftey exit thread
    clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_1); // time1 load
    // input loop function
    // Read buffer - motor state data : torque, speed, angle
    for (int i = NUM_OF_CANABLE - 4; i < NUM_OF_CANABLE; i++)
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
    // Lock the mutex only when accessing shared resources
    {
      std::lock_guard<std::mutex> lock(_mtx);
      // Update joint positions and velocities
      for (int n = 0; n < 4; n++)
      {
        joint[2 * n].CurrentPos = motor_state[n+3].theta[0];
        joint[2 * n].CurrentVel = motor_state[n+3].speed[0];
        joint[2 * n + 1].CurrentPos = motor_state[n+3].theta[1];
        joint[2 * n + 1].CurrentVel = motor_state[n+3].speed[1];
      }

      if (std::abs(x_pixel) <= 0.005f && !planar_flag)
      {
        planar_flag = true;
        runtime = 0.0; // Reset runtime for planar motion
      }

      // 이 코드에서 base_flag 부분을 빼고 planar 만 남기기(x_pixel==0으로 가정). 활성화 시 rpm = 0.;
      // 이후 joint[2] joint[3](shoulder) joint[4](elbow) joint[6](wrist=90*D2R) joint[5](wrist) IK
      // runtime=0.; 으로 초기화, 모터 구동
      // j8(gripper) grip motion->이후 다시 복귀하는 알고리즘
      // 이야기를 듣고 마음을 알아보는 것 -- 경청

      if (planar_flag) // Planar motion control
      {
        if (runtime == 0.0)
        {
          // float th_y = std::asin(y_pixel / z_depth);
          // float th_z = std::acos((L1 * L1 + z_depth * z_depth - L2 * L2) / (2 * L1 * z_depth));
          // float th_shoulder = PI - (th_z - th_y);
          // float th_elbow = std::acos((L1 * L1 - z_depth * z_depth + L2 * L2) / (2 * L1 * L2));

          // float th_y = std::asin(y_pixel / z_depth);
          // float th_z = std::acos((L1 * L1 + z_depth * z_depth - L2 * L2) / (2 * L1 * z_depth));
          float th_shoulder = 150 * D2R;
          float th_elbow = 45 * D2R;
          // j3 <---> th3 //

          // joint[0].RefTargetPos = ; // pitch
          joint[0].InitPos = motor_state[3].theta[0]; // can10 motor 1
          joint[0].InitVel = motor_state[0].speed[0];
          // joint[0].RefTargetPos = joint[0].InitPos; // motor is not in use.

          joint[1].RefTargetPos = th_elbow; // can10 motor 2
          joint[1].InitPos = joint[1].CurrentPos;
          joint[1].InitVel = joint[1].CurrentVel;

          joint[2].RefTargetPos = -th_shoulder; // can11 motor 1 2
          joint[2].InitPos = joint[2].CurrentPos;
          joint[2].InitVel = joint[2].CurrentVel;
          joint[3].RefTargetPos = th_shoulder;
          joint[3].InitPos = joint[3].CurrentPos;
          joint[3].InitVel = joint[3].CurrentVel;

          joint[4].RefTargetPos = 90 * D2R; // for yaw to pitch of joint[5]
          joint[4].InitPos = motor_state[2].theta[0];
          joint[4].InitVel = motor_state[2].speed[0];

          joint[5].RefTargetPos = -30 * D2R; // pitch
          joint[5].InitPos = motor_state[2].theta[1];
          joint[5].InitVel = motor_state[2].speed[1];

          joint[6].InitPos = motor_state[3].theta[0];
          joint[6].InitVel = motor_state[3].speed[0];
          joint[6].RefTargetPos = joint[6].InitPos; // motor of Mr. ChanWoo is not in use.

          joint[7].RefTargetPos = -300 * D2R; // opening
          joint[7].InitPos = motor_state[3].theta[1];
          joint[7].InitVel = motor_state[3].speed[1];
        }

        if (runtime < 3000.0)
        {
          joint[0].RefPos = joint[0].InitPos;
          joint[0].RefVel = joint[0].InitVel;

          joint[1].RefPos = func_1_cos(runtime, joint[1].InitPos, joint[1].RefTargetPos, 3000.0);
          joint[1].RefVel = dt_func_1_cos(runtime, joint[1].InitPos, joint[1].RefTargetPos, 3000.0);

          joint[2].RefPos = func_1_cos(runtime, joint[2].InitPos, joint[2].RefTargetPos, 3000.0);
          joint[2].RefVel = dt_func_1_cos(runtime, joint[2].InitPos, joint[2].RefTargetPos, 3000.0);

          joint[3].RefPos = func_1_cos(runtime, joint[3].InitPos, joint[3].RefTargetPos, 3000.0);
          joint[3].RefVel = dt_func_1_cos(runtime, joint[3].InitPos, joint[3].RefTargetPos, 3000.0);

          joint[4].RefPos = func_1_cos(runtime, joint[4].InitPos, joint[4].RefTargetPos, 3000.0);
          joint[4].RefVel = dt_func_1_cos(runtime, joint[4].InitPos, joint[4].RefTargetPos, 3000.0);

          joint[6].RefPos = joint[6].InitPos;
          joint[6].RefVel = joint[6].InitVel;

          joint[5].RefPos = func_1_cos(runtime, joint[5].InitPos, joint[5].RefTargetPos, 3000.0);
          joint[5].RefVel = dt_func_1_cos(runtime, joint[5].InitPos, joint[5].RefTargetPos, 3000.0);

          joint[7].RefPos = func_1_cos(runtime, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
          joint[7].RefVel = dt_func_1_cos(runtime, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
        }
        else if (runtime == 3000.0)
        {
          for (int i = 0; i < 7; i++)
          {
            joint[i].RefPos = joint[i].RefPos;
            joint[i].RefVel = joint[i].RefVel;
          }
          joint[7].RefTargetPos = 300 * D2R; // gripping motion
          joint[7].InitPos = motor_state[3].theta[1];
          joint[7].InitVel = motor_state[3].speed[1];
        }
        else if (3000.0 < runtime && runtime < 6000.0)
        {
          joint[7].RefPos = func_1_cos(runtime - 3000, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
          joint[7].RefVel = dt_func_1_cos(runtime - 3000, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
        }
        else if (runtime == 6000.0)
        {
          joint[1].RefTargetPos = -th_elbow;
          joint[1].InitPos = joint[1].CurrentPos;
          joint[1].InitVel = joint[1].CurrentVel;

          joint[2].RefTargetPos = th_shoulder;
          joint[2].InitPos = joint[2].CurrentPos;
          joint[2].InitVel = joint[2].CurrentVel;
          joint[3].RefTargetPos = -th_shoulder;
          joint[3].InitPos = joint[3].CurrentPos;
          joint[3].InitVel = joint[3].CurrentVel;

          joint[4].RefTargetPos = -90 * D2R;
          joint[4].InitPos = motor_state[2].theta[0];
          joint[4].InitVel = motor_state[2].speed[0];

          joint[5].RefTargetPos = 30 * D2R;
          joint[5].InitPos = motor_state[2].theta[1];
          joint[5].InitVel = motor_state[2].speed[1];

          // joint[6].RefTargetPos = -90 * D2R;
          // joint[6].InitPos = motor_state[3].theta[0];
          // joint[6].InitVel = motor_state[3].speed[0];

          // joint[7].RefTargetPos = 300 * D2R; // gripping motion
          // joint[7].InitPos = motor_state[3].theta[1];
          // joint[7].InitVel = motor_state[3].speed[1];
        }
        else if (6000.0 < runtime && runtime < 9000.0)
        {
          for (int i = 1; i < 8; i++)
          {
            joint[i].RefPos = func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
            joint[i].RefVel = dt_func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
          }
        }
        else
        {
          for (int i = 0; i < 8; i++)
          {
            joint[i].RefPos = joint[i].RefPos;
            joint[i].RefVel = joint[i].RefVel;
          }
        }
        // else
        // {
        //   planar_flag = false;
        // }
      }

      std::cout << "Joint_Ref_POS" << joint[2].RefPos * R2D << std::endl;
      std::cout << "Joint_Cur_POS" << joint[2].CurrentPos * R2D << std::endl;
      std::cout << "Joint_RefTarget_POS" << joint[2].RefTargetPos * R2D << std::endl;
      std::cout << "Joint_Init_POS" << joint[2].InitPos * R2D << std::endl;
      std::cout << "RUNTIME" << runtime << std::endl;
    }
    // calc torque(real range: -30~30A) Mapping to -2000~2000
    ComputeTorque();
    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j + 1, tau[i][j], frame[i]);
        // Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j+1, tau[i][j],frame[i]);
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
    //-----------logging part-----------//
    fprintf(drokX_data, "%f \t", runtime);

    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drokX_data, "%f \t", motor_state[i].theta[j]);
      }
    }
    // std::cout<<"mt done"<<std::endl;
    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drokX_data, "%f \t", motor_state[i].speed[j]);
      }
    }
    // std::cout<<"ms done"<<std::endl;

    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drokX_data, "%f \t", (float)(tau[i][j]));
      }
    }
    // std::cout<<"tq done"<<std::endl;
    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drokX_data, "%f \t", motor_state[i].torque[j]);
      }
    }

    fprintf(drokX_data, "%f \t", thread_timer);

    // fprintf temperature
    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drokX_data, "%f \t", motor_state[i].temp[j]);
      }
    }

    fprintf(drokX_data, "\n");
    //-----------logging part-----------//
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
  // Calcualte tau(ref torque) and Map it to current to send commands to RMDMotor
  // tau[0][0] = (float)((Kp_HR * (joint[0].RefPos - joint[0].CurrentPos) + Kd_HR * (joint[0].RefVel - joint[0].CurrentVel)) * TauMap);
  tau[0][0] = 0.;
  tau[0][1] = (float)((200 * (joint[1].RefPos - joint[1].CurrentPos) + 2 * (joint[1].RefVel - joint[1].CurrentVel)) * TauMap);

  tau[1][0] = (float)((200. * (joint[2].RefPos - joint[2].CurrentPos) + 2 * (joint[2].RefVel - joint[2].CurrentVel)) * TauMap);
  tau[1][1] = (float)((200. * (joint[3].RefPos - joint[3].CurrentPos) + 2 * (joint[3].RefVel - joint[3].CurrentVel)) * TauMap);

  tau[2][0] = (float)((400 * (joint[4].RefPos - joint[4].CurrentPos) + 2 * (joint[4].RefVel - joint[4].CurrentVel)) * TauMap); //
  tau[2][1] = (float)((400 * (joint[5].RefPos - joint[5].CurrentPos) + 8 * (joint[5].RefVel - joint[5].CurrentVel)) * TauMap);
  tau[3][0] = (float)((200 * (joint[6].RefPos - joint[6].CurrentPos) + 8 * (joint[6].RefVel - joint[6].CurrentVel)) * TauMap); //
  tau[3][1] = (float)((400. * (joint[7].RefPos - joint[7].CurrentPos) + 2 * (joint[7].RefVel - joint[7].CurrentVel)) * TauMap);
  // tau[0][1] [][] [][] [][] [][] [][] [][] 합격.

  // for (int i = 0; i < 4; i++)
  // {
  //   for (int j = 0; j < 2; j++)
  //   {
  //     if (tau[i][j] > 62.5)
  //     {
  //       tau[i][j] = 62.5;
  //     }
  //     if (tau[i][j] < -62.5)
  //     {
  //       tau[i][j] = -62.5;
  //     }
  //   }
  // }

  // if (tau[0][0] > 62.5)
  // {
  //   tau[0][0] = 62.5;
  // }
  // if (tau[0][0] < -62.5)
  // {
  //   tau[0][0] = -62.5;
  // }
  if (tau[0][1] > 135.5)
  {
    tau[0][1] = 135.5;
  }
  if (tau[0][1] < -135.5)
  {
    tau[0][1] = -135.5;
  }
  if (tau[1][0] > 250.5)
  {
    tau[1][0] = 250.5;
  }
  if (tau[1][0] < -250.5)
  {
    tau[1][0] = -250.5;
  }
  if (tau[1][1] > 250.5)
  {
    tau[1][1] = 250.5;
  }
  if (tau[1][1] < -250.5)
  {
    tau[1][1] = -250.5;
  }
  // ////////////////////////
  if (tau[2][0] > 135.5)
  {
    tau[2][0] = 135.5;
  }
  if (tau[2][0] < -135.5)
  {
    tau[2][0] = -135.5;
  }
  if (tau[2][1] > 135.5)
  {
    tau[2][1] = 135.5;
  }
  if (tau[2][1] < -135.5)
  {
    tau[2][1] = -135.5;
  }
  // if (tau[3][0] > 62.5)
  // {
  //   tau[3][0] = 62.5;
  // }
  // if (tau[3][0] < -62.5)
  // {
  //   tau[3][0] = -62.5;
  // }
  if (tau[3][1] > 187.5)
  {
    tau[3][1] = 187.5;
  }
  if (tau[3][1] < -187.5)
  {
    tau[3][1] = -187.5;
  }
  // tau[0][0] = (int) ((Kp_HR*(joint[FLHR].RefPos - joint[FLHR].CurrentPos)  + Kd_HR*(joint[FLHR].RefVel- joint[FLHR].CurrentVel))  *TauMap);
  // tau[0][1] = (int) ((Kp_HP*(joint[FLHP].RefPos - joint[FLHP].CurrentPos)  + Kd_HP*(joint[FLHP].RefVel- joint[FLHP].CurrentVel))  *TauMap);
  // tau[0][2] = (int) ((Kp_KN*(joint[FLKN].RefPos - joint[FLKN].CurrentPos)  + Kd_KN*(joint[FLKN].RefVel- joint[FLKN].CurrentVel))  *TauMap);
  // tau[1][0] = (int) ((Kp_HR*(joint[FRHR].RefPos - joint[FRHR].CurrentPos)  + Kd_HR*(joint[FRHR].RefVel- joint[FRHR].CurrentVel))  *TauMap);
  // tau[1][1] = (int) ((Kp_HP*(joint[FRHP].RefPos - joint[FRHP].CurrentPos)  + Kd_HP*(joint[FRHP].RefVel- joint[FRHP].CurrentVel))  *TauMap);
  // tau[1][2] = (int) ((Kp_KN*(joint[FRKN].RefPos - joint[FRKN].CurrentPos)  + Kd_KN*(joint[FRKN].RefVel- joint[FRKN].CurrentVel))  *TauMap);
  // tau[2][0] = (int) ((Kp_HR*(joint[RLHR].RefPos - joint[RLHR].CurrentPos)  + Kd_HR*(joint[RLHR].RefVel- joint[RLHR].CurrentVel))  *TauMap);
  // tau[2][1] = (int) ((Kp_HP*(joint[RLHP].RefPos - joint[RLHP].CurrentPos)  + Kd_HP*(joint[RLHP].RefVel- joint[RLHP].CurrentVel))  *TauMap);
  // tau[2][2] = (int) ((Kp_KN*(joint[RLKN].RefPos - joint[RLKN].CurrentPos)  + Kd_KN*(joint[RLKN].RefVel- joint[RLKN].CurrentVel))  *TauMap);
  // tau[3][0] = (int) ((Kp_HR*(joint[RRHR].RefPos - joint[RRHR].CurrentPos)  + Kd_HR*(joint[RRHR].RefVel- joint[RRHR].CurrentVel))  *TauMap);
  // tau[3][1] = (int) ((Kp_HP*(joint[RRHP].RefPos - joint[RRHP].CurrentPos)  + Kd_HP*(joint[RRHP].RefVel- joint[RRHP].CurrentVel))  *TauMap);
  // tau[3][2] = (int) ((Kp_KN*(joint[RRKN].RefPos - joint[RRKN].CurrentPos)  + Kd_KN*(joint[RRKN].RefVel- joint[RRKN].CurrentVel))  *TauMap);
}

//*Read buffer func -> save HR,HP,KN data to one buffer(frame), save type is que
void MainActiveNode::READ_TORQUE_SPEED_ANGLE(int s, int canable, struct can_frame frame)
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
  timeout.tv_usec = 0.5; // 0.5 us

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
      degree = (float)pulse * pulse_to_degree; // pulse to degree : 2 ^-16
      motor_state[canable].temp[motor_id - 1] = temp;
      // std::cout << "temp : " << std::dec << temp << std::endl;
      // std::cout << "torque : " << std::dec << torque << std::endl;
      // std::cout << "speed : " << std::dec << speed << std::endl;
      std::cout << "Motor ID : " << motor_id << std::endl;
      // std::cout << "PULSE    : " << pulse << std::endl;
      std::cout << "degree   : " << std::dec << degree << std::endl;
      std::cout << "previous_degree : " << std::dec << previous_degree[canable][motor_id - 1] << std::endl;
      std::cout << "angle_switch   : " << std::dec << angle_switch[canable][motor_id - 1] << std::endl;

      ///========================= Single turn : -360 ~ 360 ===============================///

      if (flag[canable][motor_id - 1] == 1)
      {
        previous_degree[canable][motor_id - 1] = degree;
        flag[canable][motor_id - 1] = 0;
      }

      if (degree - previous_degree[canable][motor_id - 1] < -5)
        angle_switch[canable][motor_id - 1] += 1;
      else if (degree - previous_degree[canable][motor_id - 1] > 5)
        angle_switch[canable][motor_id - 1] -= 1;

      /*if (angle_switch[canable][motor_id - 1] > 9)
        angle_switch[canable][motor_id - 1] -= 10;
      else if (angle_switch[canable][motor_id - 1] < -9)
        angle_switch[canable][motor_id - 1] += 10;*/

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

      ///========================= Single turn  : -360 ~ 360===============================///

      if (frame.can_id == 0x141)
      {
        // FL
        if (canable == 0)
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 36;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
        // FR
        else if (canable == 1)
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 36;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
      }

      // LEFT  : 0,2
      // RIGHT : 1,3

      else if (frame.can_id == 0x142) // HIP Pitch
      {
        // FL
        if (canable == 0)
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 36;
          motor_state[canable].theta[1] = (output_degree + 0.0) * D2R; // 71.78 is offset -> must fix
        }
        // FR
        else if (canable == 1)
        {
          motor_state[canable].torque[1] = -torque;
          motor_state[canable].speed[1] = -speed * D2R / 36;
          motor_state[canable].theta[1] = -(output_degree - 0) * D2R; // -74.3
        }
      }

      previous_degree[canable][motor_id - 1] = degree;
      std::cout << "Motor Theta[0] : " << motor_state[0].theta[0] * R2D << std::endl;
      std::cout << "Motor Theta[1] : " << motor_state[0].theta[1] * R2D << std::endl;
      std::cout << "=================================" << std::endl;
      // std::cout << "endangle_switch   : " << std::dec << angle_switch[canable][motor_id - 1] << std::endl;
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
  timeout.tv_usec = 0.5; // 0.5 us

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
      std::cout << "Motor ID : " << motor_id << std::endl;
      // std::cout << "PULSE    : " << pulse << std::endl;
      std::cout << "degree   : " << std::dec << degree << std::endl;
      std::cout << "previous_degree : " << std::dec << previous_degree[canable][motor_id - 1] << std::endl;
      std::cout << "angle_switch   : " << std::dec << angle_switch[canable][motor_id - 1] << std::endl;

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
        if (canable == 2)
        {
          motor_state[canable].torque[0] = -torque;
          motor_state[canable].speed[0] = -speed * D2R / 6;
          motor_state[canable].theta[0] = -(output_degree)*D2R;
        }
        // RR
        else if (canable == 3)
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
        if (canable == 2)
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 6;
          motor_state[canable].theta[1] = (output_degree + 0) * D2R; //+74.25
        }
        // RR
        else if (canable == 3)
        {
          motor_state[canable].torque[1] = -torque;
          motor_state[canable].speed[1] = -speed * D2R / 6;
          motor_state[canable].theta[1] = -(output_degree - 0) * D2R; //-74.5
        }
      }

      previous_degree[canable][motor_id - 1] = degree;
      std::cout << "Motor Theta[1] : " << motor_state[0].theta[1] * R2D << std::endl;
      std::cout << "Motor Theta[2] : " << motor_state[1].theta[0] * R2D << std::endl;
      std::cout << "=================================" << std::endl;
      // std::cout << "endangle_switch   : " << std::dec << angle_switch[canable][motor_id - 1] << std::endl;
    }
  }
}

void MainActiveNode::publishMessage()
{
  std::cout << "publish : start " << std::endl;
  switch (err_count)
  {
  case 1:
    std::cout << "Error Count : " << err_count << std::endl;
    break;
  case 2:
    std::cout << "\033[1;31m" << "OVER SPEED!! ( " << err_count << ")" << "\033[0m" << std::endl;
    err_count = 0;
    break;
  default:
    break;
  }
  for (int i = 0; i < NUM_OF_CANABLE; ++i)
  {
    for (int j = 0; j < NUM_OF_MOTOR; ++j)
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

      std::cout << "publish : end " << std::endl;
    }
  }
}
