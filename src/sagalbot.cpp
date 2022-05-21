#include "./sagalbot.h"

using namespace sagalbot;
MyRobot::MyRobot(const size_t& _num_joints){
    /*Generate joint names automatically*/
    ROS_INFO("Hello, sagal-bot with %ld joints", _num_joints);
    /*Make joint state handles*/
    std::stringstream ss;
    for(size_t i=0; i<_num_joints; ++i){
        ss.clear();
        ss << "Joint_"<<i;
        joint_list.push_back(ss.str());
    }
    MyRobot_init();
}
MyRobot::MyRobot(const std::string& _joint_list_file){
    /*Get joint list from file*/
    ROS_INFO("Hello, sagal-bot with joint list file: %s", _joint_list_file.c_str());
    std::ifstream joint_list_stream(_joint_list_file);
    std::string st_joint_name;
    while(joint_list_stream){
        getline(joint_list_stream, st_joint_name);
        joint_list.push_back(st_joint_name);
    }
    MyRobot_init();
}
MyRobot::MyRobot(const std::vector<std::string>& _joint_list_vector){
    /*Get joint list from vector*/
    ROS_INFO("Hello, sagal-bot with joint list vector. Loading joint names...");
    std::for_each(_joint_list_vector.begin(), _joint_list_vector.end(), [&](const std::string& _st_jnt_name)->
    void{
        joint_list.push_back(_st_jnt_name);
    });
    MyRobot_init();
}

MyRobot::~MyRobot(){

}

void MyRobot::MyRobot_init(){
    size_t num_joints = joint_list.size();
    cmd.clear();
    pos.clear();
    vel.clear();
    eff.clear();
    ROS_INFO("Joint name loaded: ");
    std::for_each(joint_list.begin(), joint_list.end(), [](const std::string& _st_jnt_name)->void{
        ROS_INFO("Joint name: %s", _st_jnt_name.c_str());
    });
    for(size_t i=0; i<num_joints; ++i){
        cmd.push_back(0);
        pos.push_back(0);
        vel.push_back(0);
        eff.push_back(0);
        hardware_interface::JointStateHandle state_handle_init(joint_list[i],&pos[i],&vel[i],&eff[i]);
        jnt_state_handles.push_back(std::move(state_handle_init));
    }
    /*Register state handles to interface*/
    std::for_each(jnt_state_handles.begin(), jnt_state_handles.end(),
        [&](hardware_interface::JointStateHandle& _to_add)->void{jnt_state_interface.registerHandle(_to_add);});
    registerInterface(&jnt_state_interface);

    /*Make joint position handles*/
    for(size_t i=0;i<num_joints;++i){
        hardware_interface::JointHandle posi_handle_init(jnt_state_handles[i],&cmd[i]);
        jnt_pos_handles.push_back(std::move(posi_handle_init));
    }
    /*Register handles to interface*/
    std::for_each(jnt_pos_handles.begin(), jnt_pos_handles.end(),
        [&](hardware_interface::JointHandle& _to_add)->void{jnt_pos_interface.registerHandle(_to_add);});
    registerInterface(&jnt_pos_interface);
}

void MyRobot::read(){
    for(size_t i=0;i<joint_list.size();++i){
        pos[i] = 0.0;
        vel[i] = 0.0;
        eff[i] = 0.0;
    }
}

void MyRobot::write(){
    std::stringstream ss;
    for(size_t i=0; i<joint_list.size(); ++i){
        ss<<joint_list[i]<<" cmd: "<<cmd[i]<<std::endl;
    }
    std::cout<<ss.str();
}