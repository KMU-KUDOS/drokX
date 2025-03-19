#include "MAIN.h"
#include <time.h>

bool stop = false;
bool step = false;
bool flipper_mode = false;
int selected_flipper = -1;

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
  int core = 1;          // Set the desired CPU core
  int priority = 99;     // Set the desired thread priority

  Publish_timer = this->create_wall_timer(10ms, std::bind(&MainActiveNode::publishMessage, this)); // publish Message 10ms interval
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MainActiveNode::joy_callback, this, std::placeholders::_1));
  cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MainActiveNode::cmd_vel_callback, this, std::placeholders::_1));

  command_mode = RPM_COMMAND; // TEST TORQUE_COMMAND POSITION_COMMAND RPM_COMMAND

  joint = static_cast<JOINT *>(std::malloc(JOINT_NUM * sizeof(JOINT))); // Dynamic memory allocation by multiplying memory size by the number of joints.

  // set can
  cansock[0] = CAN_INIT("can4");
  //cansock[1] = CAN_INIT("can5");

  Kp_FR = 50.; // Front Right
  Kp_FL = 50.;
  Kp_BR = 50.; // Back Left
  Kp_BL = 50.;
  Kd_FR = 1.;
  Kd_FL = 1.;
  Kd_BR = 1.;
  Kd_BL = 1.;

  // load angle, torque, speed once
  for (int i = 0; i < NUM_OF_CANABLE; i++)
  {
    for (int j = 0; j < NUM_OF_MOTOR; j++)
    {
      flag[i][j] = 1; // 플래그로 상태 표시
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
  std::string filename = "/home/kudos/ros2_ws/src/drok6/logs/log_" + std::to_string(now->tm_year + 1900) + std::to_string(now->tm_mon + 1) + std::to_string(now->tm_mday) + "__" + std::to_string(now->tm_hour) + std::to_string(now->tm_min) + std::to_string(now->tm_sec) + ".txt";
  std::cout << "filename  " << filename << std::endl;
  usleep(500);
  // start Main thread
  main_thread = Thread_init([](void *arg) -> void *
                            { return static_cast<MainActiveNode *>(arg)->pthread_main(); },
                            this, core, policy, priority);
  // write to txt file
  drok6_data = fopen(filename.c_str(), "w+");
}

void MainActiveNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linear_velocity = msg->linear.x * 1000.0;
  angular_velocity = msg->angular.z * 1000.0;

  left_wheel_rpm = ((linear_velocity + (wheel_base / 2.0) * angular_velocity) / (2.0 * M_PI * wheel_radius)) * 10.0;
  right_wheel_rpm = ((linear_velocity - (wheel_base / 2.0) * angular_velocity) / (2.0 * M_PI * wheel_radius)) * 10.0;

  left_rpm_command_ = static_cast<int32_t>(left_wheel_rpm);   // 왼쪽 바퀴 (반대 방향)
  right_rpm_command_ = -static_cast<int32_t>(right_wheel_rpm); // 오른쪽 바퀴
}

void MainActiveNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joystick_data_ = msg;

  // Toggle flipper mode
  if (joystick_data_->buttons[6])
  {
    flipper_mode = !flipper_mode;
  }

  // Select flipper for adjustment
  if (flipper_mode)
  {
    for (int i = 0; i < 4; i++)
    {
      if (joystick_data_->buttons[i])
      {
        selected_flipper = i;
        break;
      }
    }

    // Adjust torque for selected flipper
    if (selected_flipper != -1)
    {
      float torque_adjustment = joystick_data_->axes[1];
      tau[1][selected_flipper] = static_cast<int>(torque_adjustment * 1000); // Scale as necessary
    }
  }
}

MainActiveNode::~MainActiveNode()
{
  // write log file done
  fclose(drok6_data);

  std::cout << "[drok6]destructor of MainActive Node " << std::endl;
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
      std::cout << "[" << i << "-" << j << "]"
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
  std::string ifname = can_interface_name;        // can4~5
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
  ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
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
  pinfo.period_ns = ns_TwoMilSec; // 2ms
  usleep(500);
  clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
  auto thread_start = std::chrono::high_resolution_clock::now();

  while ((!main_thread_exit))
  {
    pthread_testcancel();
    clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_1);

    // Read buffer 
    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        READ_TORQUE_SPEED_ANGLE(cansock[i], i, frame[i]);
      }
    }
    GetState();

    // Control logic for wheels and flippers
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        if (i == 0)
        {
          if (j < 2)
          {
            Rmd.RPM_CLOSED_LOOP_CONTROL(cansock[i], j + 1, left_rpm_command_, frame[i]);
          }
          else
          {
            Rmd.RPM_CLOSED_LOOP_CONTROL(cansock[i], j + 1, right_rpm_command_, frame[i]);
          }
        }
        else
        {
          if (joystick_data_ && joystick_data_->buttons[4])
          {
            float torque_adjustment = joystick_data_->axes[1];
            tau[1][j] = static_cast<int>(torque_adjustment * 1000);
          }

          float current_angle = motor_state[1].theta[j] * R2D;
          if (current_angle > 40.0)
          {
            Rmd.POSITION_CONTROL_A4(cansock[1], j + 1, 40.0 * D2R, 4, frame[1]);
          }
          else if (current_angle < -40.0)
          {
            Rmd.POSITION_CONTROL_A4(cansock[1], j + 1, -40.0 * D2R, 4, frame[1]);
          }
          else
          {
            Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[1], j + 1, tau[1][j], frame[1]);
          }
        }
      }
    }

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
    fprintf(drok6_data, "%f \t", runtime);

    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drok6_data, "%f \t", motor_state[i].theta[j]);
      }
    }

    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drok6_data, "%f \t", motor_state[i].speed[j]);
      }
    }

    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drok6_data, "%d \t", (int)(tau[i][j])); // was %f
      }
    }

    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drok6_data, "%f \t", motor_state[i].torque[j]);
      }
    }

    fprintf(drok6_data, "%f \t", thread_timer);

    for (int i = 0; i < NUM_OF_CANABLE; i++)
    {
      for (int j = 0; j < NUM_OF_MOTOR; j++)
      {
        fprintf(drok6_data, "%f \t", motor_state[i].temp[j]);
      }
    }
    fprintf(drok6_data, "\n");
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
  tau[0][0] = (int)((Kp_FL * (joint[0].RefPos - joint[0].CurrentPos) + Kd_FL * (joint[0].RefVel - joint[0].CurrentVel)) * TauMap);

  if (tau[0][0] > 62.5)
  {
    tau[0][0] = 62.5;
  }
  if (tau[0][0] < -62.5)
  {
    tau[0][0] = -62.5;
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

      temp = (frame.data[1]);
      torque = (frame.data[2] << 0) | (frame.data[3] << 8);
      speed = (frame.data[4] << 0) | (frame.data[5] << 8);
      pulse = (frame.data[6] << 0) | (frame.data[7] << 8);
      degree = (float)pulse * pulse_to_degree;
      motor_state[canable].temp[motor_id - 1] = temp;

      if (flag[canable][motor_id - 1] == 1)
      {
        previous_degree[canable][motor_id - 1] = degree;
        flag[canable][motor_id - 1] = 0;
      }

      if (degree - previous_degree[canable][motor_id - 1] < -20)
        angle_switch[canable][motor_id - 1] += 1;
      else if (degree - previous_degree[canable][motor_id - 1] > 20)
        angle_switch[canable][motor_id - 1] -= 1;

      output_degree = angle_switch[canable][motor_id - 1] * 40. + degree;

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
        if (canable == 0)
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 9;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
        else if (canable == 1)
        {
          motor_state[canable].torque[0] = torque;
          motor_state[canable].speed[0] = speed * D2R / 9;
          motor_state[canable].theta[0] = (output_degree)*D2R;
        }
        else if (canable == 2)
        {
          motor_state[canable].torque[0] = -torque;
          motor_state[canable].speed[0] = -speed * D2R / 9;
          motor_state[canable].theta[0] = -(output_degree)*D2R;
        }
        else if (canable == 3)
        {
          motor_state[canable].torque[0] = -torque;
          motor_state[canable].speed[0] = -speed * D2R / 9;
          motor_state[canable].theta[0] = -(output_degree)*D2R;
        }
      }
      else if (frame.can_id == 0x142)
      {
        if (canable == 0)
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 9;
          motor_state[canable].theta[1] = (output_degree + 71.78) * D2R;
        }
        else if (canable == 1)
        {
          motor_state[canable].torque[1] = -torque;
          motor_state[canable].speed[1] = -speed * D2R / 9;
          motor_state[canable].theta[1] = -(output_degree - 72.4) * D2R;
        }
        else if (canable == 2)
        {
          motor_state[canable].torque[1] = torque;
          motor_state[canable].speed[1] = speed * D2R / 9;
          motor_state[canable].theta[1] = (output_degree + 72.32) * D2R;
        }
        else if (canable == 3)
        {
          motor_state[canable].torque[1] = -torque;
          motor_state[canable].speed[1] = -speed * D2R / 9;
          motor_state[canable].theta[1] = -(output_degree - 71.8) * D2R;
        }
      }
      else if (frame.can_id == 0x143)
      {
        if (canable == 0)
        {
          motor_state[canable].torque[2] = torque;
          motor_state[canable].speed[2] = speed * D2R / 9;
          motor_state[canable].theta[2] = (output_degree - 0.) * D2R;
        }
        else if (canable == 1)
        {
          motor_state[canable].torque[2] = -torque;
          motor_state[canable].speed[2] = -speed * D2R / 9;
          motor_state[canable].theta[2] = -(output_degree + 160.59) * D2R;
        }
        else if (canable == 2)
        {
          motor_state[canable].torque[2] = -torque;
          motor_state[canable].speed[2] = -speed * D2R / 9;
          motor_state[canable].theta[2] = -(output_degree + 160.59) * D2R;
        }
        else if (canable == 3)
        {
          motor_state[canable].torque[2] = -torque;
          motor_state[canable].speed[2] = -speed * D2R / 9;
          motor_state[canable].theta[2] = -(output_degree + 160.59) * D2R;
        }
      }
      previous_degree[canable][motor_id - 1] = degree;
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
    std::cout << "\033[1;31m"
              << "OVER SPEED!! ( " << err_count << ")"
              << "\033[0m" << std::endl;
    err_count = 0;
    break;
  default:
    break;
  }
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

      std::cout << "publish : end !!!" << std::endl;
    }
  }
}

