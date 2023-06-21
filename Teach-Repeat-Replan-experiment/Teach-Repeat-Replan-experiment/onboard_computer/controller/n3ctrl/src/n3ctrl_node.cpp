#include <ros/ros.h>
#include "N3CtrlFSM.h"

#include <quadrotor_msgs/SO3Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <signal.h>

#include <n3ctrl/ControllerDebug.h>
N3CtrlFSM* pFSM;

//将终端被中断时候的状态进行消息发送，可视化
void mySigintHandler(int sig) {
    pFSM->stateVisualizer.publish_led_vis(ros::Time::now(), "null");
    ROS_ERROR("[N3Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "n3ctrl");
    ros::NodeHandle nh("~");
    // signal(SIGINT, sig_int);param1:处理的信号类型，如SIGINT  param2:处理函数
    // SIGINT信号由InterruptKey产生，通常是CTRL +C 或者是DELETE 产生，发送给所有ForeGround Group的进程。
    signal(SIGINT, mySigintHandler);
    // 延时1s  Sleep for one second while Rate requires the sleep duration in Hz (1/s)
    ros::Duration(1.0).sleep();

    // 参数类，里面存放的是无人机悬停时增益参数，轨迹跟踪时增益参数，怠速状态时（无人机解锁了，但是没起飞）的限制参数
    // 摇杆值的缩放参数以及其它一些参数，具体在参数部分讲解，Parameter_t 类的定义见N3CtrlParam.h 和N3CtrlParam.cpp
    // 具体的参数值看 config文件夹下的 ctrl_param_fpv.yaml.
    Parameter_t param;
    // 控制类，控制的具体实现，我们重点讲解部分，类的定义及实现 controller.cpp 和 controller.h 
    Controller controller(param);
    // 悬停 油门 kalman滤波  
    HovThrKF hov_thr_kf(param);
    // 这个对应就是总的控制，大图 fsm中各种控制模式的切换就是在这个类里实现
    // N3CtrlFSM.h 是类的定义，在 N3CtrlFSM.cpp , N3CtrlFSM_control.cpp,N3CtrlFSM_state.cpp  分别对类做了不同的实现
    N3CtrlFSM fsm(param, controller, hov_thr_kf);
    pFSM = &fsm;

    //在launch 启动时已经把ctrl_param_fpv.yaml文件中的所有参数都加载到ros空间了
    //这里把参数传给 param 类中对应参数
    param.config_from_ros_handle(nh);
    //这里初始化只做了一件事，就是计算全推力，hov_percent 表示悬停百分比
    // full_thrust = mass * gra / hov_percent;  
    // 很奇怪的是这里计算的 full_thrust 是75.41，但是在 .yaml 中是 60，
    // 看完所有代码再来解答这个问题
    param.init();
    
    // 悬停油门的卡尔曼滤波
    fsm.hov_thr_kf.init();
    fsm.hov_thr_kf.set_hov_thr(param.hov_percent);
    // 检查悬停油门 百分比是否为 0 
    if (param.hover.set_hov_percent_to_zero) {
        // set to zero for debug
        fsm.hov_thr_kf.set_hov_thr(0.0);
    }
    // 这里就是比较参数 人为设计的 full_thrust 和  
    //根据 m*g/hov_percent计算的full_thrust 是否相同，在这个参数里面不同
    ROS_INFO("Initial value for hov_thr set to %.2f/%.2f",
             fsm.hov_thr_kf.get_hov_thr(),
             param.mass * param.gra / param.full_thrust);
    // 这个是 bool值， use_hov_percent_kf 在yaml文件中是true，是否使用悬停
    ROS_INFO("Hovering thrust kalman filter is %s.",
             param.hover.use_hov_percent_kf ? "used" : "NOT used");
    // 是否等待跳过等待rc摇杆值
    bool skip_wait_for_rc = false;
    //如果是在跑仿真，
    // yaml文件中定义了workmode为realtime  FSM的N3CtrlFSM.cpp文件中只有下面三个模式 SIMULATION，SIM_WITHOUT_RC，REALTIME
    if (param.work_mode.compare("simulation") == 0) { 
        fsm.set_work_mode(N3CtrlFSM::SIMULATION); //在fsm中选择模式   simulation模式，就跳过等待
        skip_wait_for_rc = true;
    } else if (param.work_mode.compare("sim_without_rc") == 0) {
        fsm.set_work_mode(N3CtrlFSM::SIM_WITHOUT_RC); //simulation 不用摇杆，也跳过等待
        skip_wait_for_rc = true;
    } else {  
        fsm.set_work_mode(N3CtrlFSM::REALTIME);  //其它情况都要等待获取摇杆值
    }

    if (param.js_ctrl_mode.compare("raw") == 0) {
        fsm.set_js_ctrl_mode(N3CtrlFSM::JS_CTRL_MODE_RAW);
    } else {
        fsm.set_js_ctrl_mode(N3CtrlFSM::JS_CTRL_MODE_FEEDBACK);
    }

    fsm.rc_data.set_default_mode(std::string("manual"));
    fsm.rc_data.set_default_mode(std::string("noapi"));
    fsm.controller.config();

    ros::Subscriber joy_sub =
        nh.subscribe<sensor_msgs::Joy>("joy",
                                       1000,
                                       boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         1000,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("imu",
                                       1000,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "cmd",
        1000,
        boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber idle_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(
        "idling",
        1000,
        boost::bind(&Idling_Data_t::feed, &fsm.idling_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber enter_js_sub =
        nh.subscribe<std_msgs::Header>("enter_js",
                                       1000,
                                       boost::bind(&Trigger_Data_t::feed, &fsm.trigger_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    fsm.controller.ctrl_pub = nh.advertise<sensor_msgs::Joy>("ctrl", 10);

    // fsm.controller.ctrl_so3_pub	=
    // 	nh.advertise<quadrotor_msgs::SO3Command>("ctrl_so3", 10);

    fsm.controller.ctrl_so3_attitude_pub =
        nh.advertise<geometry_msgs::QuaternionStamped>("ctrl_so3/attitude", 10);

    fsm.controller.ctrl_so3_thrust_pub =
        nh.advertise<geometry_msgs::WrenchStamped>("ctrl_so3/thrust", 10);

    fsm.controller.ctrl_vis_pub = nh.advertise<sensor_msgs::Imu>("ctrl_vis", 10);

    fsm.controller.ctrl_dbg_pub = nh.advertise<std_msgs::Header>("ctrl_dbg/info", 10);

    fsm.controller.ctrl_val_dbg_pub = nh.advertise<n3ctrl::ControllerDebug>("ctrl_dbg/value", 10);

    fsm.controller.ctrl_dbg_p_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/p", 10);

    fsm.controller.ctrl_dbg_v_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/v", 10);

    fsm.controller.ctrl_dbg_a_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/a", 10);

    fsm.controller.ctrl_dbg_att_des_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/att_des", 10);

    fsm.controller.ctrl_dbg_att_real_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/att_real", 10);

    fsm.hov_thr_kf.hov_thr_pub = nh.advertise<std_msgs::Float64>("hov_thr", 10);

    fsm.des_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("desire_pose", 10);

    fsm.fsm_dbg_pub = nh.advertise<std_msgs::Header>("fsm_dbg", 10);

    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 10);

    fsm.stateVisualizer.led_pub = nh.advertise<visualization_msgs::Marker>("state_led", 10);

    // essential for publishers and subscribers to get ready
    ros::Duration(0.5).sleep();

    if (skip_wait_for_rc) {
        ROS_INFO("[N3CTRL] Simulation, skip rc.");
    } else {
        ROS_INFO("[N3CTRL] Waiting for rc");
        while (ros::ok()) {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now())) {
                ROS_INFO("[N3CTRL] rc received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    // ros::Timer timer = nh.createTimer(ros::Duration(1.0/1000.0),
    // 	boost::bind(&N3CtrlFSM::process, &fsm, _1));

    ros::Rate r(2000.0);
    fsm.last_ctrl_time = ros::Time::now();
    // ---- process ----
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        fsm.process();
    }

    return 0;
}
