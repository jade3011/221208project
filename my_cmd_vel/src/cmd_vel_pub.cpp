#include "my_cmd_vel/cmd_vel_pub.h"


controller::controller(ros::NodeHandle *nh_):nh(nh_){

    odom_subscriber = nh->subscribe("/odom", 1, &controller::messageCallback, this);
    lidar_subscriber=nh->subscribe("/lidar_object",10,&controller::lidarCallback,this);
    cmd_vel_publisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);

    lstate_pub = nh->advertise<std_msgs::String>("/led_state",10);

    gain_x = 0.5;
    gain_y =0.2;
    c = 0.214;
    flag = false;
    is_goal=false;
    velocity = 0;
    steer_angle = 0;
    distance = 0;

    pre_velocity = 0;
    pre_steer_angle = 0;
    lstate.data="RED";

}

// lidar call back
void controller::lidarCallback(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
    pcl::fromROSMsg(msg,output_pointcloud);
    if(!(output_pointcloud.points.size()==0)){

     goal_x =  output_pointcloud.points.at(0).x;
     goal_y = output_pointcloud.points.at(0).y;
     flag =true;



    distance = sqrt((goal_x*goal_x)+(goal_y*goal_y));
    // distance more than 1.3 -> stop
    if (distance <= 1.3)
     {
         is_goal = true;
        // std::cout<<"distance to goal"<<distance<<std::endl;
         lstate.data="RED";
         lstate_pub.publish(lstate);

     }
     else
     {
         is_goal=false;
       //  std::cout<<"distance to goal"<<distance<<std::endl;
         lstate.data="GREEN";
         lstate_pub.publish(lstate);
     }
    }
}

void controller::messageCallback(const nav_msgs::Odometry &msg){

}

void controller :: process(){

    ang_z = 0;
    pos_x = 0;
    pos_y =0;


    if (flag == true && is_goal == false)
    {
        geometry_msgs::Twist cmd_vel; // 속도 파라미터


        error_x = goal_x - pos_x;
        error_y = goal_y - pos_y;

        velocity = cos(ang_z) * gain_x * error_x + sin(ang_z) * gain_y * error_y;
        steer_angle = 1/c*(-sin(ang_z) * gain_x * error_x + cos(ang_z) * gain_y * error_y);
        if(velocity > pre_velocity+0.3)
        {
            velocity = pre_velocity+0.3;
        }
        else if(velocity <pre_velocity-0.3)
        {
            velocity = pre_velocity -0.3;
        }

        if(steer_angle>pre_steer_angle+0.1)
        {
            steer_angle = pre_steer_angle +0.1;
        }
        else if(steer_angle<pre_steer_angle-0.1)
        {
            steer_angle = pre_steer_angle - 0.1;
        }


        if(velocity>1.5)
        {
            velocity = 1.5;
        }
        else if(velocity<0)
        {
            velocity = 0;
        }
        if(steer_angle>0.60)
        {
            steer_angle = 0.60;
        }
        else if(steer_angle<-0.60)
        {
             steer_angle = -0.60;
        }


//        if(distance <= 1.3)
//        {
//            velocity = 0;
//            std::cout<<" to close"<<std::endl;
//        }

        pre_velocity = velocity;
        pre_steer_angle = steer_angle;

        cmd_vel.linear.x = velocity;
        cmd_vel.angular.z = steer_angle;
        ROS_INFO("object angle : %2f", atan2(error_y, error_x)*180/3.14);
        ROS_INFO("vel : %2f", cmd_vel.linear.x);
        ROS_INFO("ang_vel : %f\n", cmd_vel.angular.z);


        cmd_vel_publisher.publish(cmd_vel);
        flag =false;
    }
    else
    {
       if(flag == false)
       {
           //std::cout<<"no goal point data"<<std::endl;

       }


    }
}
controller::~controller()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_pub");
    ros::NodeHandle nh;

    controller kine(&nh);

    while(ros::ok())
    {
       ros::spinOnce();
       kine.process();
    }

   // ros::spin();

    return 0;
}

