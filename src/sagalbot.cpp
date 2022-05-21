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
    delete[] cmd;
    delete[] pos;
    delete[] vel;
    delete[] eff;
    delete[] jnt_state_handles;
    delete[] jnt_pos_handles;
}

void MyRobot::MyRobot_init(){
    const size_t num_joints = joint_list.size();
    cmd = new double[num_joints];
    pos = new double[num_joints];
    vel = new double[num_joints];
    eff = new double[num_joints];
    jnt_state_handles = new hardware_interface::JointStateHandle[num_joints];
    jnt_pos_handles = new hardware_interface::JointHandle[num_joints];
    
    ROS_INFO("Joint name loaded: ");
    std::for_each(joint_list.begin(), joint_list.end(), [](const std::string& _st_jnt_name)->void{
        ROS_INFO("Joint name: %s", _st_jnt_name.c_str());
    });
    
    for(size_t i=0; i<num_joints; ++i){
        cmd[i] = 0;
        pos[i] = 0;
        vel[i] = 0;
        eff[i] = 0;
        /*Register state handles to interface*/
        jnt_state_handles[i] = hardware_interface::JointStateHandle(joint_list[i],&pos[i],&vel[i],&eff[i]);
        jnt_state_interface.registerHandle(jnt_state_handles[i]);
    }
    registerInterface(&jnt_state_interface);

    /*Make joint position handles and register it to interface*/
    for(size_t i=0;i<num_joints;++i){
        jnt_pos_handles[i] = hardware_interface::JointHandle(jnt_state_handles[i],&cmd[i]);
        jnt_pos_interface.registerHandle(jnt_pos_handles[i]);
    }
    registerInterface(&jnt_pos_interface);
}

void MyRobot::read(){
    for(size_t i=0;i<joint_list.size();++i){
        pos[i] += 1.0;
        if(pos[i] > 10) pos[i] = 0;
        vel[i] += 1.0;
        if(vel[i] > 20) vel[i] = 10;
        eff[i] += 1.0;
        if(eff[i] > 30) eff[i] = 20;
    }
}

void MyRobot::write(){
    std::stringstream ss;
    for(size_t i=0; i<joint_list.size(); ++i){
        ss<<joint_list[i]<<" cmd: "<<cmd[i]<<std::endl;
    }
    std::cout<<ss.str();
}