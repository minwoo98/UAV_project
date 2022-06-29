#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//----this code should be written in theh header file later----//
mavros_msgs::State current_state;
nav_msgs::Odometry odom;

int set_goal_cnt = 0;
int count = 0;

double pre_err_lin_x;
double pre_err_lin_y;
double pre_err_lin_z;

const double kp = 0.45;
const double kd = 0.15;

double dist(nav_msgs::Odometry now_pose, geometry_msgs::PoseStamped goal);

//------------------------------------------------------------//

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){

    //ROS_INFO("Pos: [x] : %.2f |, [y] : %.2f |, [z] : %.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    //get current position(odom)
    odom.pose.pose.position.x = msg->pose.pose.position.x;
    odom.pose.pose.position.y = msg->pose.pose.position.y;
    odom.pose.pose.position.z = msg->pose.pose.position.z;
}
double dist(nav_msgs::Odometry now, geometry_msgs::PoseStamped goal)
{
    double dist = sqrt(
                        pow(now.pose.pose.position.x - goal.pose.position.x,2) + 
                        pow(now.pose.pose.position.y - goal.pose.position.y,2) +
                        pow(now.pose.pose.position.z - goal.pose.position.z,2)
                      );

    return dist;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, odom_cb);

    //publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    //serviceclient
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.x = 0;
    goal_pose.pose.position.y = 0;
    goal_pose.pose.position.z = 4;

    geometry_msgs::TwistStamped goal_vel;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        /*
        //velocity_control square path
        if(dist(odom, goal_pose) < 0.18)
        {
            //waypoint에 도착하면 다음 goal_pose setting
            //ROS_INFO("arrived at [%d]", set_goal_cnt);
            if(set_goal_cnt == 0)
            {
                goal_pose.pose.position.x = 0;
                goal_pose.pose.position.y = 4;
                goal_pose.pose.position.z = 4;
                set_goal_cnt = 1;
            }
            else if(set_goal_cnt == 1)
            {
                goal_pose.pose.position.x = 4;
                goal_pose.pose.position.y = 4;
                goal_pose.pose.position.z = 4;
                set_goal_cnt = 2;
            }
            
            else if(set_goal_cnt == 2)
            {
                goal_pose.pose.position.x = 4;
                goal_pose.pose.position.y = 0;
                goal_pose.pose.position.z = 4;
                set_goal_cnt = 3;
            }
            else if(set_goal_cnt == 3)
            {
                goal_pose.pose.position.x = 0;
                goal_pose.pose.position.y = 0;
                goal_pose.pose.position.z = 4;
                set_goal_cnt = 0;
            }
        }
        */
        //local_pos_pub.publish(goal_pose); //postion control

        //velocity_control circle path
        const int r = 7;
        double theta;

        theta = count*0.01;

        goal_pose.pose.position.x = r*cos(theta);
        goal_pose.pose.position.y = r*sin(theta);
        goal_pose.pose.position.z = 4;

        count += 1;
     
        double err_lin_x = goal_pose.pose.position.x - odom.pose.pose.position.x;
        double err_lin_y = goal_pose.pose.position.y - odom.pose.pose.position.y;
        double err_lin_z = goal_pose.pose.position.z - odom.pose.pose.position.z;
       

        goal_vel.twist.linear.x = kp*err_lin_x + kd*(err_lin_x - pre_err_lin_x)*dt;
        goal_vel.twist.linear.y = kp*err_lin_y + kd*(err_lin_y - pre_err_lin_y)*dt;
        goal_vel.twist.linear.z = kp*err_lin_z + kd*(err_lin_z - pre_err_lin_z)*dt;

        if(goal_vel.twist.linear.x > 3)   goal_vel.twist.linear.x = 3;
        if(goal_vel.twist.linear.x < -3)  goal_vel.twist.linear.x = -3;
        if(goal_vel.twist.linear.y > 3)   goal_vel.twist.linear.y = 3;
        if(goal_vel.twist.linear.y < -3)   goal_vel.twist.linear.y = -3;
        if(goal_vel.twist.linear.z > 3)   goal_vel.twist.linear.z = 3;
        if(goal_vel.twist.linear.z < -3)   goal_vel.twist.linear.z = -3;

        local_vel_pub.publish(goal_vel);
        
        //ROS_INFO(" target_x : %.2f | target_y : %.2f | target_z : %.2f | ", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
        //ROS_INFO(" vx : %.2f | vy : %.2f | vz : %.2f | ", goal_vel.twist.linear.x, goal_vel.twist.linear.y, goal_vel.twist.linear.z);

        pre_err_lin_x = err_lin_x;
        pre_err_lin_y = err_lin_y;
        pre_err_lin_z = err_lin_z;

        last_time = current_time;
        
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}