/*
 * Copyright (c) 2024-2025 Pony
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_d1.hpp"

// SDK joint order: FL[0], FR[1], RL[2], RR[3] for each joint type
// rl_sar joint order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
// Mapping: rl_sar_idx -> sdk_leg_idx
static const int SDK_LEG_MAP[4] = {1, 0, 3, 2}; // FR->1, FL->0, RR->3, RL->2

RL_Real::RL_Real(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_axis = "world";
    this->robot_name = "d1";
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // Network init for D1
    int local_port = 43988;
    std::string local_ip = "192.168.234.2";  // PC IP
    std::string robot_ip = "192.168.234.1";  // D1 default IP

    // Parse command line arguments for network config
    if (argc >= 2)
    {
        local_ip = argv[1];
    }
    if (argc >= 3)
    {
        robot_ip = argv[2];
    }

    std::cout << LOGGER::INFO << "Connecting to D1 robot at " << robot_ip << " from " << local_ip << std::endl;

    // Initialize D1 SDK (both LowLevel and HighLevel)
    this->d1_lowlevel.initRobot(local_ip, local_port, robot_ip);
    this->d1_highlevel.initRobot(local_ip, local_port + 1, robot_ip);

    // Wait for LowLevel connection
    int retry_count = 0;
    while (!this->d1_lowlevel.checkConnect() && retry_count < 50)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
    }

    if (!this->d1_lowlevel.checkConnect())
    {
        std::cout << LOGGER::ERROR << "Failed to connect to D1 robot (LowLevel)!" << std::endl;
    }
    else
    {
        std::cout << LOGGER::INFO << "Connected to D1 robot (LowLevel) successfully!" << std::endl;
    }

    // Wait for HighLevel connection
    retry_count = 0;
    while (!this->d1_highlevel.checkConnect() && retry_count < 50)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
    }

    if (!this->d1_highlevel.checkConnect())
    {
        std::cout << LOGGER::WARNING << "Failed to connect to D1 robot (HighLevel)!" << std::endl;
    }
    else
    {
        std::cout << LOGGER::INFO << "Connected to D1 robot (HighLevel) successfully!" << std::endl;
        // Print battery power
        uint32_t battery = this->d1_highlevel.getBatteryPower();
        std::cout << LOGGER::INFO << "Battery power: " << battery << "%" << std::endl;
    }

    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // Initialize mapped vectors
    this->mapped_joint_positions.resize(12, 0.0f);
    this->mapped_joint_velocities.resize(12, 0.0f);
    this->mapped_joint_torques.resize(12, 0.0f);

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    // Send zero command before exit
    mc_sdk::motorCmd zero_cmd;
    for (int i = 0; i < 4; i++)
    {
        zero_cmd.kp_abad[i] = 0.0f;
        zero_cmd.kp_hip[i] = 0.0f;
        zero_cmd.kp_knee[i] = 0.0f;
        zero_cmd.kd_abad[i] = 3.0f;
        zero_cmd.kd_hip[i] = 3.0f;
        zero_cmd.kd_knee[i] = 3.0f;
    }
    for (int i = 0; i < 100; i++)
    {
        this->d1_lowlevel.sendMotorCmd(zero_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<float> *state)
{
    // Get motor state from SDK
    this->d1_motor_state = this->d1_lowlevel.getMotorState();

    // Get IMU data
    std::vector<float> quat = this->d1_lowlevel.getQuaternion();
    std::vector<float> gyro = this->d1_lowlevel.getBodyGyro();

    if (quat.size() >= 4)
    {
        state->imu.quaternion[0] = quat[0]; // w
        state->imu.quaternion[1] = quat[1]; // x
        state->imu.quaternion[2] = quat[2]; // y
        state->imu.quaternion[3] = quat[3]; // z
    }

    if (gyro.size() >= 3)
    {
        state->imu.gyroscope[0] = gyro[0];
        state->imu.gyroscope[1] = gyro[1];
        state->imu.gyroscope[2] = gyro[2];
    }

    // Map joint states from SDK to rl_sar order
    // SDK order: FL[0], FR[1], RL[2], RR[3]
    // rl_sar order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
    if (this->d1_motor_state != nullptr && this->d1_lowlevel.haveMotorData())
    {
        for (int leg = 0; leg < 4; ++leg)
        {
            int sdk_leg = SDK_LEG_MAP[leg];
            int base_idx = leg * 3;

            state->motor_state.q[base_idx + 0] = this->d1_motor_state->q_abad[sdk_leg];
            state->motor_state.q[base_idx + 1] = this->d1_motor_state->q_hip[sdk_leg];
            state->motor_state.q[base_idx + 2] = this->d1_motor_state->q_knee[sdk_leg];

            state->motor_state.dq[base_idx + 0] = this->d1_motor_state->qd_abad[sdk_leg];
            state->motor_state.dq[base_idx + 1] = this->d1_motor_state->qd_hip[sdk_leg];
            state->motor_state.dq[base_idx + 2] = this->d1_motor_state->qd_knee[sdk_leg];

            state->motor_state.tau_est[base_idx + 0] = this->d1_motor_state->tau_abad_fb[sdk_leg];
            state->motor_state.tau_est[base_idx + 1] = this->d1_motor_state->tau_hip_fb[sdk_leg];
            state->motor_state.tau_est[base_idx + 2] = this->d1_motor_state->tau_knee_fb[sdk_leg];
        }
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    // Map commands from rl_sar order to SDK order
    // rl_sar order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
    // SDK order: FL[0], FR[1], RL[2], RR[3]
    for (int leg = 0; leg < 4; ++leg)
    {
        int sdk_leg = SDK_LEG_MAP[leg];
        int base_idx = leg * 3;

        this->d1_motor_cmd.q_des_abad[sdk_leg] = command->motor_command.q[base_idx + 0];
        this->d1_motor_cmd.q_des_hip[sdk_leg] = command->motor_command.q[base_idx + 1];
        this->d1_motor_cmd.q_des_knee[sdk_leg] = command->motor_command.q[base_idx + 2];

        this->d1_motor_cmd.qd_des_abad[sdk_leg] = command->motor_command.dq[base_idx + 0];
        this->d1_motor_cmd.qd_des_hip[sdk_leg] = command->motor_command.dq[base_idx + 1];
        this->d1_motor_cmd.qd_des_knee[sdk_leg] = command->motor_command.dq[base_idx + 2];

        this->d1_motor_cmd.kp_abad[sdk_leg] = command->motor_command.kp[base_idx + 0];
        this->d1_motor_cmd.kp_hip[sdk_leg] = command->motor_command.kp[base_idx + 1];
        this->d1_motor_cmd.kp_knee[sdk_leg] = command->motor_command.kp[base_idx + 2];

        this->d1_motor_cmd.kd_abad[sdk_leg] = command->motor_command.kd[base_idx + 0];
        this->d1_motor_cmd.kd_hip[sdk_leg] = command->motor_command.kd[base_idx + 1];
        this->d1_motor_cmd.kd_knee[sdk_leg] = command->motor_command.kd[base_idx + 2];

        this->d1_motor_cmd.tau_abad_ff[sdk_leg] = command->motor_command.tau[base_idx + 0];
        this->d1_motor_cmd.tau_hip_ff[sdk_leg] = command->motor_command.tau[base_idx + 1];
        this->d1_motor_cmd.tau_knee_ff[sdk_leg] = command->motor_command.tau[base_idx + 2];
    }

    // Send command to robot
    int ret = this->d1_lowlevel.sendMotorCmd(this->d1_motor_cmd);
    if (ret < 0)
    {
        std::cout << LOGGER::WARNING << "Failed to send motor command" << std::endl;
    }
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
        !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"),
                     this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state.motor_state.q[i]);
        this->plot_target_joint_pos[i].push_back(this->robot_command.motor_command.q[i]);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}

void RL_Real::SwitchToLowLevel()
{
    if (!this->lowlevel_mode)
    {
        // Use HighLevel passive() to release control
        this->d1_highlevel.passive();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        this->lowlevel_mode = true;
        std::cout << LOGGER::INFO << "Switched to LowLevel control mode" << std::endl;
    }
}

void RL_Real::SwitchToHighLevel()
{
    if (this->lowlevel_mode)
    {
        this->lowlevel_mode = false;
        std::cout << LOGGER::INFO << "Switched to HighLevel control mode" << std::endl;
    }
}

uint32_t RL_Real::GetBatteryPower()
{
    return this->d1_highlevel.getBatteryPower();
}

uint32_t RL_Real::GetCurrentCtrlMode()
{
    return this->d1_highlevel.getCurrentCtrlmode();
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    RL_Real rl_sar(argc, argv);
    while (1) { sleep(10); }
#endif
    return 0;
}
