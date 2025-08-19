#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

// #define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    void Init();
    void Start();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
    int queryMotionStatus();
    std::string queryServiceName(std::string form,std::string name);
 
private:
    // float Kp = 70.0;
    // float Kd = 5.0;
    // double time_consume = 0;
    // int rate_count = 0;
    // int sin_count = 0;
    // int motiontime = 0;
    // float dt = 0.002; // 0.001~0.01

    // MotionSwitcherClient msc;

    // unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
    // ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    // ThreadPtr lowCmdWriteThreadPtr;

    // float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
    //                           -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    // float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
    //                           0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    // float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
    //                           -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

    // float _startPos[12];
    // float _duration_1 = 500;   
    // float _duration_2 = 500; 
    // float _duration_3 = 2000;   
    // float _duration_4 = 900;   
    // float _percent_1 = 0;    
    // float _percent_2 = 0;    
    // float _percent_3 = 0;    
    // float _percent_4 = 0;    

    // bool firstRun = true;
    // bool done = false;
};



void Custom::Init()
{
    // InitLowCmd();

    /*create publisher*/
    // lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    // lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*init MotionSwitcherClient*/
    // msc.SetTimeout(10.0f); 
    // msc.Init();

    /*Shut down motion control-related service*/
    // while(queryMotionStatus())
    // {
    //     std::cout << "Try to deactivate the motion control-related service." << std::endl;
    //     int32_t ret = msc.ReleaseMode(); 
    //     if (ret == 0) {
    //         std::cout << "ReleaseMode succeeded." << std::endl;
    //     } else {
    //         std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    //     }
    //     sleep(5);
    // }
}



void Custom::Start()
{
    /*loop publishing thread*/
    // lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Custom::LowCmdWrite, this);
    std::cout << "Start to read sensor data example: " << std::endl;
}

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
    std::cout << "Start to read sensor data example: " << std::endl;
    std::cout<<"Imu accelerometer : "<<"x: "<<low_state.imu_state().accelerometer()[0]<<" y: "<<low_state.imu_state().accelerometer()[1]<<" z: "<<low_state.imu_state().accelerometer()[2]<<std::endl;
    std::cout<<"Imu orientation : " << "w: " << low_state.imu_state().quaternion()[0] <<" x: "<<low_state.imu_state().quaternion()[1]<<" y: "<<low_state.imu_state().quaternion()[2]<<" z: "<<low_state.imu_state().quaternion()[3]<<std::endl;
    std::cout<<"Imu angular velocity : "<<"x: "<<low_state.imu_state().gyroscope()[0]<<" y: "<<low_state.imu_state().gyroscope()[1]<<" z: "<<low_state.imu_state().gyroscope()[2]<<std::endl;
}


int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.Init();
    custom.Start();
  
    while (1)
    {
        sleep(10);
    }

    return 0;
}
