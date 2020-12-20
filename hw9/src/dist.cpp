#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::Publisher chatter_pub;
ros::Publisher pose_pub;
ros::Publisher twist_pub;
double dist = 10000;
double learning_rate = 4;

geometry_msgs::Pose start_pose;
geometry_msgs::Twist start_twist;
geometry_msgs::Pose box_pose;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    // set if -> exit
}

void publishNewPositions()
{
    geometry_msgs::Pose tmp_pose = start_pose;
    geometry_msgs::Twist tmp_twist = start_twist;
    
    
    while ((pow((tmp_pose.position.x - box_pose.position.x), 2.0) - pow((start_pose.position.x - box_pose.position.x), 2.0)) > 0.99) {
            if (tmp_pose.position.x > box_pose.position.x)
                tmp_pose.position.x -= learning_rate * abs(tmp_pose.position.x);
            else
                tmp_pose.position.x += learning_rate * abs(tmp_pose.position.x);
    }
                
    while ((pow((tmp_pose.position.x - box_pose.position.y), 2.0) - pow((start_pose.position.y - box_pose.position.y), 2.0)) > 0.99) {
            if (tmp_pose.position.y > box_pose.position.y)
                tmp_pose.position.y -= learning_rate * abs(tmp_pose.position.y);
            else
                tmp_pose.position.y += learning_rate * abs(tmp_pose.position.y);
    }
    start_pose.position.x, start_pose.position.y, start_pose.position.z = tmp_pose.position.x, tmp_pose.position.y, tmp_pose.position.z;
    
    pose_pub.publish(start_pose);
    twist_pub.publish(start_twist);
}

void posCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    start_pose.position.x = msg->pose[1].position.x;
    start_pose.position.y = msg->pose[1].position.y;
    start_pose.position.z = msg->pose[1].position.z;
    
    start_pose.orientation.x = msg->pose[1].orientation.x;
    start_pose.orientation.y = msg->pose[1].orientation.y;
    start_pose.orientation.z = msg->pose[1].orientation.z;
    start_pose.orientation.w = msg->pose[1].orientation.w;
    
    start_twist.linear.x = msg->twist[1].linear.x;
    start_twist.linear.y = msg->twist[1].linear.y;
    start_twist.linear.z = msg->twist[1].linear.z;
    
    start_twist.angular.x = msg->twist[1].angular.x;
    start_twist.angular.y = msg->twist[1].angular.y;
    start_twist.angular.z = msg->twist[1].angular.z;
    
    ROS_INFO("Robot position: [%lf, %lf, %lf]", start_pose.position.x, start_pose.position.y, start_pose.position.z);
    ROS_INFO("Robot twist: [%lf, %lf, %lf]", start_twist.linear.x, start_twist.linear.y, start_twist.linear.z);
    
    if ((msg->pose).size() == 3) {
        box_pose.position.x = msg->pose[2].position.x;
        box_pose.position.y = msg->pose[2].position.y;
        box_pose.position.z = msg->pose[2].position.z;
        
        box_pose.orientation.x = msg->pose[2].orientation.x;
        box_pose.orientation.y = msg->pose[2].orientation.y;
        box_pose.orientation.z = msg->pose[2].orientation.z;
        box_pose.orientation.w = msg->pose[2].orientation.w;
        
        ROS_INFO("Box position: [%lf, %lf, %lf]", box_pose.position.x, box_pose.position.y, box_pose.position.z);
        
        dist = sqrt(pow((start_pose.position.x - box_pose.position.x), 2.0) + 
        pow((start_pose.position.y - box_pose.position.y), 2.0) + 
        pow((start_pose.position.z - box_pose.position.z), 2.0));
        ROS_INFO("Distance: [%lf]", dist);
    }
    
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::NodeHandle n_pose;
    ros::NodeHandle n_twist;
    
    chatter_pub = n.advertise<std_msgs::String>("speed_chatter", 1000);
    pose_pub = n_pose.advertise<geometry_msgs::Pose>("cmd_pose", 1000);
    twist_pub = n_twist.advertise<geometry_msgs::Twist>("cmd_twist", 1000);
    
    ros::init(argc, argv, "listener");
    ros::Subscriber models_states_sub = n.subscribe("/gazebo/model_states", 1000, posCallback);
    ros::Subscriber pose_sub = n_pose.subscribe("cmd_pose", 1000, chatterCallback);
    ros::Subscriber twist_sub = n_twist.subscribe("cmd_twist", 1000, chatterCallback);
    
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        if (dist > 1) {
            std_msgs::String msg;
            int speed;
            
            if (n.getParam("speed", speed))
            {
                std::stringstream ss;
                ss << "Speed: " << speed;
                msg.data = ss.str();
                ROS_INFO("Msg data: %s", msg.data.c_str());
            }
            else
            {
                ROS_ERROR("Failed to get param 'speed'");
            }
    
            chatter_pub.publish(msg);
            
            publishNewPositions();
            
        } else {
            ROS_INFO("The distance is small: %lf", dist);
            break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ros::spin();
    return 0;
}