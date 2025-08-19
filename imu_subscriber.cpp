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

#define TOPIC_LOWSTATE "rt/lowstate"

class Custom{
public:
    explicit Custom(){}
    ~Custom(){}
    void Run(); 
private:
    void LowStateMessageHandler(const void* messages);
private:
    unitree_go::msg::dds_::LowState_ low_state{};  // default init
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
};

void Custom::Run(){
    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);
}

void Custom::LowStateMessageHandler(const void* message){
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
    std::cout << "Start to read sensor data example: " << std::endl;
    std::cout<<"Imu accelerometer : "<<"x: "<<low_state.imu_state().accelerometer()[0]<<" y: "<<low_state.imu_state().accelerometer()[1]<<" z: "<<low_state.imu_state().accelerometer()[2]<<std::endl;
    std::cout<<"Imu orientation : " << "w: " << low_state.imu_state().quaternion()[0] <<" x: "<<low_state.imu_state().quaternion()[1]<<" y: "<<low_state.imu_state().quaternion()[2]<<" z: "<<low_state.imu_state().quaternion()[3]<<std::endl;
    std::cout<<"Imu angular velocity : "<<"x: "<<low_state.imu_state().gyroscope()[0]<<" y: "<<low_state.imu_state().gyroscope()[1]<<" z: "<<low_state.imu_state().gyroscope()[2]<<std::endl;
}


int main(int argc, const char** argv){
    if (argc < 2){
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ChannelFactory::Instance()->Init(0, argv[1]);
    Custom custom;
    custom.Run();
  
    while (1){
        sleep(10);
    }
    return 0;
}
