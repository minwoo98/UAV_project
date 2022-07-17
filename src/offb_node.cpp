#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

//----this code should be written in theh header file later----//
mavros_msgs::State current_state;
nav_msgs::Odometry odom;

#define pi 3.141592

double roll, pitch, yaw;
double target_roll,target_pitch,target_yaw;

int set_goal_cnt = 0;
int count = 0;


double vth;

double pre_err_lin_x;
double pre_err_lin_y;
double pre_err_lin_z;

double pre_err_roll;
double pre_err_pitch;
double pre_err_yaw;
double pre_err_thrust;

const double kp_lin_vel = 0.45;
const double kd_lin_vel = 0.15;
/*
const double kp_roll_dist = 0.15;
const double kp_pitch_dist = 0.3;
const double kp_yaw_dist = 0.3;
const double kp_roll = 0.3;
const double kp_pitch = 0.3;
const double kp_yaw = 0.3;
*/
double pre_thrust = 0.7;
const double kp_thrust = 0.33;

 double deg = 90;


double dist(nav_msgs::Odometry now_pose, geometry_msgs::PoseStamped goal);

//------------------------------------------------------------//

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    vth = msg->angular_velocity.z;
    //ROS_INFO("%.3f", vth);
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){

    odom.pose.pose.position.x = msg->pose.pose.position.x;  
    odom.pose.pose.position.y = msg->pose.pose.position.y;
    odom.pose.pose.position.z = msg->pose.pose.position.z;

    odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    odom.twist.twist.linear.x = msg->twist.twist.linear.x;
    odom.twist.twist.linear.y = msg->twist.twist.linear.y;
    odom.twist.twist.linear.z = msg->twist.twist.linear.z;

    odom.twist.twist.angular.x = msg->twist.twist.angular.x;
    odom.twist.twist.angular.y = msg->twist.twist.angular.y;
    odom.twist.twist.angular.z = msg->twist.twist.angular.z;

    tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(q);

    mat.getRPY(roll,pitch,yaw);
    //ROS_INFO("%.2f | %.2f | %.2f ", roll,pitch,yaw);
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
geometry_msgs::Quaternion rpy_to_quat(double roll, double pitch, double yaw)
{
    tf::Quaternion q;
    geometry_msgs::Quaternion att_quat;

    q.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,att_quat);

    att_quat.x *= -1;
    att_quat.y *= -1;
    att_quat.z *= -1;
    att_quat.w *= -1;

    /*
    ROS_INFO("ORI %.2f | %.2f | %.2f | %.2f ", odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w);
    ROS_INFO("ORI %.2f | %.2f | %.2f | %.2f ", att_quat.x,att_quat.y,att_quat.z,att_quat.w);
    ROS_INFO("----------------------");
    */
    return att_quat;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, odom_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 20, imu_cb);

    //publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    //serviceclient
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

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
    mavros_msgs::AttitudeTarget goal_att;

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
    //
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double th = 0.0;

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


        //postition control - square
        #if 0

        geometry_msgs::Quaternion quat;
        //ROS_INFO("%.3f", dist(odom, goal_pose));
        ROS_INFO("%.3f | %.3f ", odom.pose.pose.position.x, odom.pose.pose.position.y);

        if(dist(odom, goal_pose) < 0.3)
        {
            ROS_INFO("arrived at [%d]", set_goal_cnt);
            if(set_goal_cnt == 0)
            {
                target_yaw = 90*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 0;
                    goal_pose.pose.position.y = 4;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 1;
                }
            }
            else if(set_goal_cnt == 1)
            {
                target_yaw = 0;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 4;
                    goal_pose.pose.position.y = 4;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 2;
                }
            }
            
            else if(set_goal_cnt == 2)
            {
                target_yaw = (-90)*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 4;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 3;
                }
            }
            else if(set_goal_cnt == 3)
            {
                target_yaw = (-180)*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 0;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 0;
                }
            }
        }

        quat = rpy_to_quat(0,0,target_yaw);
        goal_pose.pose.orientation = quat;
    
        local_pos_pub.publish(goal_pose); 

        #endif

        //position_control - circle
        #if 0
        int r = 0;
        double target_yaw = 0;
        double theta;
        geometry_msgs::Quaternion quat;

        
        if( (dist(odom, goal_pose) < 0.2) && set_goal_cnt == 0)
        {
            //initial pose setting
            goal_pose.pose.position.x = 4;
            goal_pose.pose.position.y = 0;
            goal_pose.pose.position.z = 4;

            target_yaw = 90*M_PI/180;

            if(abs(yaw - target_yaw) < 0.1)
            {
                set_goal_cnt = 1;
            }
        }
        else if(set_goal_cnt == 1)
        {
            target_yaw = deg*M_PI/180;
            theta = count*0.01;
            if(deg >= 180)
            {
                deg = -180;
            }
       
            deg += 0.01*180/M_PI;
            count += 1;     

            goal_pose.pose.position.x = 4*cos(theta);
            goal_pose.pose.position.y = 4*sin(theta);
            goal_pose.pose.position.z = 4;
            
        }

        //ROS_INFO("deg: %.2f | theta: %.2f", deg, theta);
        target_yaw = deg*M_PI/180;

        quat = rpy_to_quat(0,0,target_yaw);
        goal_pose.pose.orientation = quat;
        local_pos_pub.publish(goal_pose);

        #endif

        //velocity_control - circle
        #if 0
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
        #endif

        //attitude control - square
        #if 0

        if(dist(odom, goal_pose) < 0.3)
        {
            ROS_INFO("arrived at [%d]", set_goal_cnt);
            if(set_goal_cnt == 0)
            {   
                //target_yaw =  90*M_PI/180;
                target_yaw = 0;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 0;
                    goal_pose.pose.position.y = 4;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 1;
                }   
            }
         
            else if(set_goal_cnt == 1)
            {  
                target_yaw = 0;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 0;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 0;
                }   
            }
            
            else if(set_goal_cnt == 2)
            {
                target_yaw = (-90)*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 4;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 3;
                }    
            }
            else if(set_goal_cnt == 3)
            {
                target_yaw = (-180)*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 0;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 0;
                }   
            }  
        }
        //ROS_INFO("%.2f | %.2f | %.2f ", goal_pose.pose.position.x -odom.pose.pose.position.x, goal_pose.pose.position.y - odom.pose.pose.position.y, goal_pose.pose.position.z - odom.pose.pose.position.z);

        geometry_msgs::Quaternion quat;

        if(set_goal_cnt == 0 || set_goal_cnt == 2)
        {
            goal_vel.twist.linear.x = 0.2*(goal_pose.pose.position.x - odom.pose.pose.position.x); 
            goal_vel.twist.linear.y = 0.2*(goal_pose.pose.position.y - odom.pose.pose.position.y);
            goal_vel.twist.linear.z = 0.2*(goal_pose.pose.position.z - odom.pose.pose.position.z);
        }
        else
        {
            goal_vel.twist.linear.x = 0.2*(goal_pose.pose.position.y - odom.pose.pose.position.y); 
            goal_vel.twist.linear.y = (-0.2)*(goal_pose.pose.position.x - odom.pose.pose.position.x);
            goal_vel.twist.linear.z = 0.2*(goal_pose.pose.position.z - odom.pose.pose.position.z);
        }
       /*
        if(set_goal_cnt == 0 || set_goal_cnt == 1)
        {
            roll = (-1)*( 0.1*(goal_pose.pose.position.y - odom.pose.pose.position.y) + 0.1*( goal_vel.twist.linear.y - odom.twist.twist.linear.y) );
            pitch =   1*( 0.1*(goal_pose.pose.position.x - odom.pose.pose.position.x) + 0.1*( goal_vel.twist.linear.x - odom.twist.twist.linear.x) );
            
            roll = (-1)*( 0.1*(goal_pose.pose.position.y - odom.pose.pose.position.y))+ 0.1*odom.twist.twist.linear.y;
            pitch =  0.1*(goal_pose.pose.position.x - odom.pose.pose.position.x) - 0.1*odom.twist.twist.linear.x;
        }
        
        else if(set_goal_cnt == 1)
        {
            roll = 1*( 0.1*(goal_pose.pose.position.x - odom.pose.pose.position.x) + (-0.1)*( goal_vel.twist.linear.y - odom.twist.twist.linear.y));
            pitch =  1*( 0.1*(goal_pose.pose.position.y - odom.pose.pose.position.y) + 0.1*( goal_vel.twist.linear.x - odom.twist.twist.linear.x)) ;  
        }*/

        roll = 0.1*(sin(yaw)*(goal_pose.pose.position.x - odom.pose.pose.position.x) - cos(yaw)*(goal_pose.pose.position.y - odom.pose.pose.position.y) + odom.twist.twist.linear.y);
        pitch = 0.1*(cos(yaw)*(goal_pose.pose.position.x - odom.pose.pose.position.x) + sin(yaw)*(goal_pose.pose.position.y - odom.pose.pose.position.y) - odom.twist.twist.linear.x);

        //ROS_INFO("%.2f | %.2f ", odom.twist.twist.linear.x, odom.twist.twist.linear.y);  

        if(roll > 0.5)  roll = 0.5;
        else if(roll < -0.5) roll = -0.5;
        if(pitch > 0.5)  pitch = 0.5;
        else if(pitch < -0.5) pitch = -0.5;
       
        if(goal_vel.twist.linear.x > 1) goal_vel.twist.linear.x = 1;
        else if(goal_vel.twist.linear.x < -1) goal_vel.twist.linear.x = -1;
        if(goal_vel.twist.linear.y > 1) goal_vel.twist.linear.y = 1;
        else if(goal_vel.twist.linear.y < -1) goal_vel.twist.linear.y = -1;
        if(goal_vel.twist.linear.z > 1) goal_vel.twist.linear.z = 1;

        quat = rpy_to_quat(roll,pitch,target_yaw);
     
        double thrust = pre_thrust + 0.075*(goal_pose.pose.position.z - odom.pose.pose.position.z) + 0.2*(goal_vel.twist.linear.z-odom.twist.twist.linear.z); //0.075, 0.2
       
        //ROS_INFO("%.2f", thrust);

        //hovering 0.68 0.72
        if(thrust < 0.6575)   thrust = 0.6575; 
        if(thrust > 0.73)   thrust = 0.725 + 0.03*abs(goal_pose.pose.position.y - odom.pose.pose.position.y) + 0.03*abs(goal_pose.pose.position.x - odom.pose.pose.position.x);
        
        goal_att.type_mask = 7;
        goal_att.thrust = thrust;
        goal_att.orientation = quat;

        att_pub.publish(goal_att);

        pre_thrust = thrust;

        last_time = current_time;
      
        #endif

        //attitude control - circle
        #if 0
        
        double theta;
        double target_yaw = 0;
        geometry_msgs::Quaternion quat;

        theta = count*0.01;

        if( (dist(odom, goal_pose) < 0.2) && set_goal_cnt == 0)
        {
             target_yaw = 90*M_PI/180;

            if(abs(yaw - target_yaw) < 0.1)
            {
                set_goal_cnt = 1;
            }
        }
        else if(set_goal_cnt == 1)
        { 
            target_yaw = deg*M_PI/180;
            theta = count*0.01;
            if(deg >= 180)
            {
                deg = -180;
            }
       
            deg += 0.01*180/M_PI;
            count += 1;     

            goal_pose.pose.position.x = 4*cos(theta);
            goal_pose.pose.position.y = 4*sin(theta);
            goal_pose.pose.position.z = 4;

        }

        roll = 0.1*(sin(yaw)*(goal_pose.pose.position.x - odom.pose.pose.position.x) - cos(yaw)*(goal_pose.pose.position.y - odom.pose.pose.position.y) + odom.twist.twist.linear.y);
        pitch = 0.1*(cos(yaw)*(goal_pose.pose.position.x - odom.pose.pose.position.x) + sin(yaw)*(goal_pose.pose.position.y - odom.pose.pose.position.y) - odom.twist.twist.linear.x);

        target_yaw = deg*M_PI/180;

        if(roll > 0.5)  roll = 0.5;
        else if(roll < -0.5) roll = -0.5;
        if(pitch > 0.5)  pitch = 0.5;
        else if(pitch < -0.5) pitch = -0.5;

        quat = rpy_to_quat(roll,pitch,target_yaw);
     
        double thrust = pre_thrust + 0.075*(goal_pose.pose.position.z - odom.pose.pose.position.z) + 0.2*(goal_vel.twist.linear.z-odom.twist.twist.linear.z); //0.075, 0.2
       
        //ROS_INFO("%.2f", thrust);

        //hovering 0.68 0.72
        if(thrust < 0.6575)   thrust = 0.6575; 
        if(thrust > 0.73)   thrust = 0.725 + 0.03*abs(goal_pose.pose.position.y - odom.pose.pose.position.y) + 0.03*abs(goal_pose.pose.position.x - odom.pose.pose.position.x);
        
        goal_att.type_mask = 7;
        goal_att.thrust = thrust;
        goal_att.orientation = quat;

        att_pub.publish(goal_att);

        pre_thrust = thrust;

        #endif

        //rate control - square
        #if 1
        geometry_msgs::Quaternion quat;
        ROS_INFO("cnt: [%d]", set_goal_cnt);

        if(dist(odom, goal_pose) < 0.25)
        {
            if(set_goal_cnt == 0)
            {    
                target_yaw = 90*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 0;
                    goal_pose.pose.position.y = 4;
                    goal_pose.pose.position.z = 4;
            
                    set_goal_cnt =1;
                }
            }
            else if(set_goal_cnt == 1)
            {
                target_yaw = 0;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 4;
                    goal_pose.pose.position.y = 4;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 2;
                }
            }
            else if(set_goal_cnt == 2)
            {
                target_yaw = (-90)*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 4;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 3;
                }
            }
            else if(set_goal_cnt == 3)
            {
                target_yaw = (-180)*M_PI/180;
                if( abs(yaw - target_yaw) < 0.1)
                {
                    goal_pose.pose.position.x = 0;
                    goal_pose.pose.position.y = 0;
                    goal_pose.pose.position.z = 4;

                    set_goal_cnt = 0;
                }
            }
        }

        goal_vel.twist.linear.z = 0.2*(goal_pose.pose.position.z - odom.pose.pose.position.z);
        /*
        target_roll = (-0.1)*(goal_pose.pose.position.y - odom.pose.pose.position.y) + 0.2*odom.twist.twist.linear.y;
        target_pitch = 0.1*(goal_pose.pose.position.x - odom.pose.pose.position.x) - 0.2*odom.twist.twist.linear.x; 
        */
        target_roll = 0.1*(sin(yaw)*(goal_pose.pose.position.x - odom.pose.pose.position.x) - cos(yaw)*(goal_pose.pose.position.y - odom.pose.pose.position.y))+ 0.25*odom.twist.twist.linear.y;
        target_pitch = 0.1*(cos(yaw)*(goal_pose.pose.position.x - odom.pose.pose.position.x) + sin(yaw)*(goal_pose.pose.position.y - odom.pose.pose.position.y)) - 0.25*odom.twist.twist.linear.x;
        
        if(target_roll > 0.4)  target_roll = 0.4;
        else if(target_roll < -0.4) target_roll = -0.4;
        if(target_pitch > 0.4)  target_pitch = 0.4;
        else if(target_pitch < -0.4) target_pitch = -0.4;

        //gain initial 0.88
        goal_att.body_rate.y = 0.8*(target_pitch - pitch); //rate.y -> x axies 
        goal_att.body_rate.x = 0.8*(target_roll - roll); //rate.x -> y axies 
        goal_att.body_rate.z = 0.4*(target_yaw - yaw);
 
        ROS_INFO("%.2f | %.2f ", target_roll, roll);

        double thrust = pre_thrust + 0.075*(goal_pose.pose.position.z - odom.pose.pose.position.z) + 0.2*(goal_vel.twist.linear.z-odom.twist.twist.linear.z); //0.075, 0.2

        if(thrust < 0.6575)   thrust = 0.6575; 
        if(thrust > 0.73)   thrust = 0.725 + 0.02*abs(goal_pose.pose.position.y - odom.pose.pose.position.y) + 0.02*abs(goal_pose.pose.position.x - odom.pose.pose.position.x);

        pre_thrust = thrust;

        goal_att.type_mask = 128; //rate
        goal_att.thrust = thrust;
        att_pub.publish(goal_att);
        
        #endif

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}