#include "offboard_landing/conversion.h"

int main(int argc, char **argv) 
{

    // initialize ros node
    ros::init(argc, argv, "gps");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> 
            ("mavros/state", 50, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 1000, globalPosition_cb);

    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 50, battery_cb);

    // publisher
    ros::Publisher goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) 
    {
        ROS_INFO_ONCE("Waiting for FCU connection ...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    while (ros::ok() && !global_position_received)
    {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");
    ros::Duration(1).sleep();
    ROS_INFO("Checking status...");
    ros::Duration(1).sleep();

    // check status
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);
        std::printf("Current GPS position: [%f, %f, %f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position.altitude);
        refpoint.latitude = global_position.latitude;   
        refpoint.longitude = global_position.longitude;             
        refpoint.altitude = global_position.altitude;   

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Check status done");
    ros::Duration(1).sleep();

    // set target position
    input_global_target();
    enu_goal = WGS84ToENU(goal_pos[0][0], goal_pos[0][1], goal_pos[0][2],
                          refpoint.latitude, refpoint.longitude, refpoint.altitude);
    goal_pose.pose.position.x = enu_goal.x;
    goal_pose.pose.position.y = enu_goal.y;
    goal_pose.pose.position.z = enu_goal.z;

    // send a few setpoints before starting
    ROS_INFO("Setting OFFBOARD stream...");
    for (int i=100; ros::ok() && i>0; --i) 
    {
        goal_pose.header.stamp = ros::Time::now();
        goal_pose_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Set OFFBOARD stream done. Ready");
    ros::Duration(1).sleep();
    
    int i=0;
    while (ros::ok()) 
    {
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);
        if (i < goal_num)
        {
            enu_goal = WGS84ToENU(goal_pos[i][0], goal_pos[i][1], goal_pos[i][2],
                        refpoint.latitude, refpoint.longitude, refpoint.altitude);
            goal_pose.pose.position.x = enu_goal.x;
            goal_pose.pose.position.y = enu_goal.y;
            goal_pose.pose.position.z = enu_goal.z;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose_pub.publish(goal_pose);
            
            distance = measureGPS(global_position.latitude, 
                                  global_position.longitude, 
                                  global_position.altitude, 
                                  goal_pos[i][0], goal_pos[i][1], goal_pos[i][2]);
            std::printf("Distance to goal: %.2f m \n", distance);

            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            enu_goal = WGS84ToENU(goal_pos[goal_num-1][0], 
                                  goal_pos[goal_num-1][1], 
                                  goal_pos[goal_num-1][2],
                refpoint.latitude, refpoint.longitude, refpoint.altitude);
            goal_pose.pose.position.x = enu_goal.x;
            goal_pose.pose.position.y = enu_goal.y;
            goal_pose.pose.position.z = enu_goal.z;
            goal_pose.header.stamp = ros::Time::now();
            goal_pose_pub.publish(goal_pose);

            ROS_INFO_ONCE("Reached final goal");

            ros::spinOnce();
            rate.sleep();
        }        
        
        // check GPS reached
        enu_curr = WGS84ToENU(global_position.latitude,
                              global_position.longitude,
                              global_position.altitude,
                refpoint.latitude, refpoint.longitude, refpoint.altitude);
        bool check = check_position(enu_goal.x, enu_goal.y, enu_goal.z,
                                    enu_curr.x, enu_curr.y, enu_curr.z);
        std::cout << check << std::endl;
		if(check)
		{
            std::printf("Current GPS position: [%f, %f, %f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position.altitude);
            ros::Duration(5).sleep();
			i = i + 1;
			ros::spinOnce();
		    rate.sleep();
		}
		else 
		{
			continue;
			ros::spinOnce();
		    rate.sleep();
		}

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}