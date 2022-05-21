#include "./sagalbot.h"
#include "controller_manager/controller_manager.h"
#include "ros/ros.h"
#include <string>

int main(int argc, char **argv){
    ros::init(argc, argv, "sagal_bot_main");
    /*Create a node handle*/
    ros::NodeHandle node_obj;


    std::vector<std::string> sagal_controller_names;
    sagal_controller_names.push_back("snake_joint");
    sagal_controller_names.push_back("scorpion_joint");
    auto sagal_robot = std::make_unique<sagalbot::MyRobot>(sagal_controller_names);
    controller_manager::ControllerManager cm(sagal_robot.get());

    ROS_INFO("sagal robot has generated");
    std::vector<std::string> interface_names;
    interface_names = sagal_robot->getNames();
    std::for_each(interface_names.begin(), interface_names.end(),[](const std::string& _name)->void{
        ROS_INFO("Interface %s",_name.c_str());
    });
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
 
    ros::Time t = ros::Time::now();
    ros::Rate rate(10.0);
 
    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - t;
        ros::Time t = ros::Time::now();
        sagal_robot->read();
        cm.update(t, d);
        sagal_robot->write();
        rate.sleep();
    }
    return 0;
}