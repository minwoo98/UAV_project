#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//----this code should be written in theh header file later----//
mavros_msgs::State current_state;
nav_msgs::Odometry odom;

int set_goal_cnt = 0;

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

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 0;
    goal.pose.position.y = 0;
    goal.pose.position.z = 5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(goal);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

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

        if(dist(odom, goal) < 0.2)
        {
            //waypoint에 도착하면 다음 goal_pose setting
            ROS_INFO("arrived at [%d]", set_goal_cnt);
            if(set_goal_cnt == 0)
            {
                goal.pose.position.x = 0;
                goal.pose.position.y = 5;
                goal.pose.position.z = 5;
                set_goal_cnt = 1;
            }
            else if(set_goal_cnt == 1)
            {
                goal.pose.position.x = 5;
                goal.pose.position.y = 5;
                goal.pose.position.z = 5;
                set_goal_cnt = 2;
            }
            else if(set_goal_cnt == 2)
            {
                goal.pose.position.x = 5;
                goal.pose.position.y = 0;
                goal.pose.position.z = 5;
                set_goal_cnt = 3;
            }
            else if(set_goal_cnt == 3)
            {
                goal.pose.position.x = 0;
                goal.pose.position.y = 0;
                goal.pose.position.z = 5;
                set_goal_cnt = 0;
            }
        }

        local_pos_pub.publish(goal);
            
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
