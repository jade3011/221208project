#ifndef CMD_VEL_PUB_H
#define CMD_VEL_PUB_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <string>
#include <cmath>
#include <vector>

#include <std_msgs/String.h>

//#define SATURATION_VEL 0.5
//#define SATURATION_ANG 0.2
#define pi 3.14159265358979323846

class controller{
private:
    ros::NodeHandle *nh;
    geometry_msgs::Twist cmd_vel;

    ros::Publisher cmd_vel_publisher;
    ros::Subscriber odom_subscriber;

    ros::Subscriber goal_pose_stamped;
    ros::Subscriber lidar_subscriber;
    geometry_msgs::TransformStamped trans;
    geometry_msgs::PoseWithCovarianceStamped amcl_pose;

    // 초기 위치와 목표 위치 값의 입력이 들어왔는지
//    bool is_init = false;
    bool is_goal;
    bool flag;


    double vel_x, ang_z;
    float pos_x, pos_y, pos_yaw;

    // 목표 위치와 현재 위치의 오차
    double error_x, error_y;
    // 제어 gain값
    double gain_x, gain_y;

    double goal_x, goal_y, goal_yaw;

   double pre_velocity, pre_steer_angle;


    double c;


    double velocity;
    double steer_angle;
    std::vector<double> temp_x;
    std::vector<double>temp_y;
    std::vector<double>compare_object;
    double distance;
    double compare_tmp;
    int select_object;

    //arduino
    ros::Subscriber sstate_sub;
    ros::Publisher lstate_pub;

    std_msgs::String sstate;
    std_msgs::String lstate;



public:
    controller(ros::NodeHandle *nh_);

    void messageCallback(const nav_msgs::Odometry& msg);
    void lidarCallback(const sensor_msgs::PointCloud2 &msg);
    void process();

    ~controller();
};


#endif
