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
  int physical_core = find_physical_core(); // Get available physical core instead of hard-coded value
  int priority = 98;     // Set the desired thread priority

  Publish_timer = this->create_wall_timer(10ms, std::bind(&MainActiveNode::publishMessage, this)); // publish Message 10ms interval
  joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MainActiveNode::joy_callback, this, std::placeholders::_1));

  command_mode = RPM_COMMAND; // TEST TORQUE_COMMAND POSITION_COMMAND RPM_COMMAND

  joint = static_cast<JOINT *>(std::malloc(JOINT_NUM * sizeof(JOINT))); // Dynamic memory allocation by multiplying memory size by the number of joints.

  // set can 4 5 6 7
  cansock[0] = CAN_INIT("can4");
  cansock[1] = CAN_INIT("can5");
  cansock[2] = CAN_INIT("can6");
  cansock[3] = CAN_INIT("can7");

  Kp_FL = 10.; // PD control gains
  Kd_FL = 1.1;

  // Initialize variables
  runtime = 0.0;
  for (int i = 0; i < JOINT_NUM; i++)
  {
    joint[i].RefPos = 0.0;
    joint[i].RefVel = 0.0;
    joint[i].RefTargetPos = 0.0;
    joint[i].IsMoving = false;
  }

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

  // Read encoder value
  std::cout << "========== Current Q ==========" << std::endl;
  for (int i = 0; i < NUM_OF_CANABLE; i++)
  {
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      Rmd.REQUEST_MOTOR_STATUS(cansock[i], j + 1, frame[i]);
      usleep(500); // 0.005 sec
      READ_TORQUE_SPEED_ANGLE(cansock[i], i, frame[i]);
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
                            this, physical_core, policy, priority);
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
        joint[i].RefTargetPos -= 1.0 * D2R;       // Decrease by 1 degree
        if (joint[i].RefTargetPos < -100.0 * D2R) // Limit to min angle
          joint[i].RefTargetPos = -100.0 * D2R;
      }
      else // For flippers 1 and 2, increase as usual
      {
        joint[i].RefTargetPos += 1.0 * D2R;      // Increase by 1 degree
        if (joint[i].RefTargetPos > 100.0 * D2R) // Limit to max angle
          joint[i].RefTargetPos = 100.0 * D2R;
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
        joint[i].RefTargetPos += 1.0 * D2R;      // Increase by 1 degree
        if (joint[i].RefTargetPos > 100.0 * D2R) // Limit to max angle
          joint[i].RefTargetPos = 100.0 * D2R;
      }
      else // For flippers 1 and 2, decrease as usual
      {
        joint[i].RefTargetPos -= 1.0 * D2R;       // Decrease by 1 degree
        if (joint[i].RefTargetPos < -100.0 * D2R) // Limit to min angle
          joint[i].RefTargetPos = -100.0 * D2R;
      }
      joint[i].IsMoving = true;
      joint[i].runtime = 0.0; // Reset trajectory time

      // Print the target angle to the terminal
      std::cout << "Flipper " << i << " Target Angle: " << joint[i].RefTargetPos * R2D << " degrees" << std::endl;
    }
  }

  /*
    // === Flipper Control ===
    // Map buttons to flipper indices
    // Buttons 0,1,2,3 correspond to front left, front right, rear left, rear right flippers
    int flipper_buttons_increase[4] = {0, 1, 2, 3};
    int flipper_axes_decrease[4][2] = {{7, -1}, {6, -1}, {7, 1}, {6, 1}}; // axes[7] == -1, axes[6] == -1, etc.

    // Increase flipper target angles when buttons are pressed
    for (int i = 0; i < 4; i++)
    {
      if (msg->buttons[flipper_buttons_increase[i]] == 1)
      {
        joint[i].RefTargetPos += 1.0 * D2R;     // Increase by 1 degrees
        if (joint[i].RefTargetPos > 590.0 * D2R) // Limit to max angle
          joint[i].RefTargetPos = 590.0 * D2R;
        joint[i].IsMoving = true;
        joint[i].runtime = 0.0; // Reset trajectory time

        // Print the target angle to the terminal
        std::cout << "Flipper " << i << " Target Angle: " << joint[i].RefTargetPos * R2D << " degrees" << std::endl;
      }
    }

    // Decrease flipper target angles when axes are triggered
    for (int i = 0; i < 4; i++)
    {
      int axis_index = flipper_axes_decrease[i][0];
      int axis_value = flipper_axes_decrease[i][1];
      if (static_cast<int>(msg->axes[axis_index]) == axis_value)
      {
        joint[i].RefTargetPos -= 1.0 * D2R;      // Decrease by 5 degrees
        if (joint[i].RefTargetPos < -590.0 * D2R) // Limit to min angle
          joint[i].RefTargetPos = -590.0 * D2R;
        joint[i].IsMoving = true;
        joint[i].runtime = 0.0; // Reset trajectory time

        // Print the target angle to the terminal
        std::cout << "Flipper " << i << " Target Angle: " << joint[i].RefTargetPos * R2D << " degrees" << std::endl;
      }
    }
  */

  // If no buttons are pressed, flippers stay in place
  for (int i = 0; i < 4; i++)
  {
    if (msg->buttons[flipper_buttons_increase[i]] == 0 &&
        static_cast<int>(msg->axes[flipper_axes_decrease[i][0]]) != flipper_axes_decrease[i][1])
    {
      // Do nothing, flippers hold their position
    }
  }
}

MainActiveNode::~MainActiveNode()
{
  // write log file done
  fclose(drokX_data);

  std::cout << "[drokX]destructor of MainActive Node " << std::endl;
  main_thread_exit = true;

  // Waiting off the thread and join
  if (pthread_join(main_thread, NULL) != 0)
  {
    std::cerr << "Failed to join Main thread" << std::endl;
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

  // Free allocated memory
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
    std::lock_guard<std::mutex> lock(_mtx);
    std::cout << "Requested core " << core << " exceeds available cores, using core 0 instead" << std::endl;
    core = 0;
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

    // === Read Motor States ===
    for (int i = 0; i < NUM_OF_CANABLE; i++) // For vehicle control: CAN IDs 4, 5, 6, 7
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
    tau[0][j] = (float)((Kp_FL * (joint[j].RefPos - joint[j].CurrentPos) + Kd_FL * (joint[j].RefVel - joint[j].CurrentVel)) * TauMap);
    // Saturate torque
    if (tau[0][j] > 62.5)
      tau[0][j] = 62.5;
    if (tau[0][j] < -62.5)
      tau[0][j] = -62.5;
  }
  // For rear flippers (CAN 7)
  for (int j = 0; j < 2; j++)
  {
    int idx = j + 2; // Indexing joint[2] and joint[3]
    tau[3][j] = (float)((Kp_FL * (joint[idx].RefPos - joint[idx].CurrentPos) + Kd_FL * (joint[idx].RefVel - joint[idx].CurrentVel)) * TauMap);
    // Saturate torque
    if (tau[3][j] > 62.5)
      tau[3][j] = 62.5;
    if (tau[3][j] < -62.5)
      tau[3][j] = -62.5;
  }
}

void MainActiveNode::READ_TORQUE_SPEED_ANGLE(int s, int canable, struct can_frame frame)
{
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
      }
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

int MainActiveNode::find_physical_core()
{
  int selected_core = 0; // 기본값
  std::ifstream cpuinfo("/proc/cpuinfo");
  
  if (!cpuinfo.is_open()) {
    RCLCPP_WARN(this->get_logger(), "Could not open /proc/cpuinfo. Using default core 0.");
    return selected_core;
  }
  
  std::map<int, bool> is_physical;
  std::map<int, int> core_to_processor;
  std::string line;
  int processor = -1;
  int core_id = -1;
  bool has_siblings = false;
  
  // /proc/cpuinfo 파일 분석
  while (std::getline(cpuinfo, line)) {
    if (line.find("processor") == 0) {
      std::istringstream iss(line.substr(line.find(":") + 1));
      iss >> processor;
      // 새 프로세서를 발견할 때마다 변수 초기화
      core_id = -1;
      has_siblings = false;
    } 
    else if (line.find("core id") == 0) {
      std::istringstream iss(line.substr(line.find(":") + 1));
      iss >> core_id;
      if (core_id >= 0) {
        core_to_processor[core_id] = processor;
      }
    } 
    else if (line.find("siblings") == 0) {
      int siblings;
      std::istringstream iss(line.substr(line.find(":") + 1));
      iss >> siblings;
      if (siblings > 1) {
        has_siblings = true;
      }
    } 
    else if (line.find("cpu cores") == 0) {
      int cores;
      std::istringstream iss(line.substr(line.find(":") + 1));
      iss >> cores;
      if (cores == 1 && has_siblings && core_id >= 0) {
        // 하이퍼스레딩이 활성화된 경우, 실제 물리 코어만 표시
        is_physical[processor] = true;
      } else if (cores > 0 && core_id >= 0) {
        is_physical[processor] = true;
      }
    }
    else if (line.empty() && processor >= 0 && core_id >= 0) {
      // 엔트리 종료
      if (!is_physical.count(processor)) {
        is_physical[processor] = true; // 정보가 불충분한 경우 기본적으로 물리적인 것으로 간주
      }
    }
  }
  
  // 사용 가능한 물리 코어를 찾음
  for (const auto& pair : is_physical) {
    if (pair.second) {
      // 물리 코어인 경우
      selected_core = pair.first;
      RCLCPP_INFO(this->get_logger(), "Selected physical core: %d", selected_core);
      break;
    }
  }
  
  // 코어를 찾지 못한 경우 기본값 사용
  if (selected_core < 0) {
    selected_core = 0;
    RCLCPP_WARN(this->get_logger(), "Could not identify physical cores. Using default core 0.");
  }
  
  return selected_core;
}
