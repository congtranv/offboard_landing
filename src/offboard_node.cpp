#include "offboard_landing/offboard_landing.h"
#include "offboard_landing/logging.h"


int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "setpoint");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber gps_pos_sub = nh.subscribe<mavros_msgs::GPSRAW> 
            ("mavros/gpsstatus/gps1/raw", 10, gpsPosition_cb);
    ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/rel_alt", 10, relativeAlt_cb);
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data_raw", 10, imuData_cb);
    ros::Subscriber mag_data_sub = nh.subscribe<sensor_msgs::MagneticField>
            ("mavros/imu/mag", 10, magData_cb);
    ros::Subscriber static_press_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("mavros/imu/static_pressure", 10, staticPress_cb);
    ros::Subscriber diff_press_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("mavros/imu/diff_pressure", 10, diffPress_cb);

    // service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

    // publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
	{
        std::cout << "[ INFO] Waiting for FCU connection...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // wait for GPS information
    while (ros::ok() && !global_position_received && !gps_position_received) 
    {
        std::cout << "[ INFO] Waiting for GPS signal...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] GPS position received \n";
    std::cout << "[ INFO] Checking status...\n";
    ros::Duration(1).sleep();

	// check current pose
	for(int i = 100; ros::ok() && i > 0; --i)
	{
        std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
                     current_pose.pose.position.x, 
                     current_pose.pose.position.y, 
                     current_pose.pose.position.z);
		
        std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position.altitude);

        std::cout << "Current relative altitude: " << rel_alt.data << " m \n";

        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery capacity: %.1f \n", batt_percent);
        std::cout << "\n";
		ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Check status done \n";
    
    creates();
    creates_sensor();
    
    gps_lat = double(gps_position.lat)/10000000;
    gps_lon = double(gps_position.lon)/10000000;
    gps_alt = double(gps_position.alt)/1000;

    // init reference point
    refpoint.latitude = global_position.latitude;
    refpoint.longitude = global_position.longitude;
    refpoint.altitude = global_position.altitude;
    std::printf("Reference GPS position: [%f, %f, %.3f]\n", 
                     refpoint.latitude, 
                     refpoint.longitude, 
                     refpoint.altitude);
    updates("initial",  current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z,
                        global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        gps_lat, gps_lon, gps_alt, 
                        rel_alt.data);
    updates("reference",  current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z,
                        refpoint.latitude,
                        refpoint.longitude,
                        refpoint.altitude,
                        gps_lat, gps_lon, gps_alt, 
                        rel_alt.data);
    updates_sensor("initial", imu_data.angular_velocity.x, 
                              imu_data.angular_velocity.y, 
                              imu_data.angular_velocity.z,
                              imu_data.linear_acceleration.x, 
                              imu_data.linear_acceleration.y, 
                              imu_data.linear_acceleration.z,
                              mag_data.magnetic_field.x, 
                              mag_data.magnetic_field.y, 
                              mag_data.magnetic_field.z,
                              static_press.fluid_pressure, 
                              diff_press.fluid_pressure);

    // set target pose
    input_target();
    if (input_type == true) // local setpoint
    {
        target_pose.pose.position.x = target_pos[0][0];
        target_pose.pose.position.y = target_pos[0][1];
        target_pose.pose.position.z = target_pos[0][2];
    }
    else // global setpoint
    {
        enu_goal = WGS84ToENU(goal_pos[0][0], goal_pos[0][1], goal_pos[0][2],
                    refpoint.latitude, refpoint.longitude, refpoint.altitude);
        target_pose.pose.position.x = enu_goal.x;
        target_pose.pose.position.y = enu_goal.y;
        target_pose.pose.position.z = enu_goal.z;
    }

    // send a few setpoints before starting
    std::cout << "[ INFO] Setting OFFBOARD stream...\n";
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Set OFFBOARD stream done \n";

    gps_lat = double(gps_position.lat)/10000000;
    gps_lon = double(gps_position.lon)/10000000;
    gps_alt = double(gps_position.alt)/1000;
    updates("pre-flight", current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z,
                          global_position.latitude,
                          global_position.longitude,
                          global_position.altitude,
                          gps_lat, gps_lon, gps_alt, 
                          rel_alt.data);
    updates_sensor("pre-flight", imu_data.angular_velocity.x, 
                                 imu_data.angular_velocity.y,
                                 imu_data.angular_velocity.z,
                                 imu_data.linear_acceleration.x, 
                                 imu_data.linear_acceleration.y, 
                                 imu_data.linear_acceleration.z,
                                 mag_data.magnetic_field.x, 
                                 mag_data.magnetic_field.y, 
                                 mag_data.magnetic_field.z,
                                 static_press.fluid_pressure, 
                                 diff_press.fluid_pressure);
    ros::Duration(1).sleep();

    while (ros::ok() && !current_state.armed)
    {
        std::cout << "[ INFO] Waiting arm and takeoff... \n";
        
        ros::spinOnce();
        rate.sleep();
    }

    int i = 0;
    if (input_type)
    {
        while (ros::ok())
        {
            std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
                         current_pose.pose.position.x, 
                         current_pose.pose.position.y, 
                         current_pose.pose.position.z);
            std::printf("Next local position: [%.3f, %.3f, %.3f]\n", 
                                target_pos[i][0], 
                                target_pos[i][1],
                                target_pos[i][2]);

            if (i < (target_num -1))
            {
                final_check = false;
                target_pose.pose.position.x = target_pos[i][0];
                target_pose.pose.position.y = target_pos[i][1];
                target_pose.pose.position.z = target_pos[i][2];
            
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);
        		ros::spinOnce();
                rate.sleep();
            }
            else
            {
                final_check = true;
                target_pose.pose.position.x = target_pos[target_num - 1][0];
                target_pose.pose.position.y = target_pos[target_num - 1][1];
                target_pose.pose.position.z = target_pos[target_num - 1][2];
            
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);
        		ros::spinOnce();
                rate.sleep();
            }

            bool check = check_position(target_pose.pose.position.x,
                                        target_pose.pose.position.y,
                                        target_pose.pose.position.z,
                                        current_pose.pose.position.x,
                                        current_pose.pose.position.y,
                                        current_pose.pose.position.z);
            std::cout << check << std::endl;
            if(check && !final_check)
            {
                t_check = ros::Time::now();
                std::printf("[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                                current_pose.pose.position.x, 
                                current_pose.pose.position.y, 
                                current_pose.pose.position.z);   
                // std::printf("[ INFO] Next local position: [%.3f, %.3f, %.3f]\n", 
                //                 target_pos[i+1][0], 
                //                 target_pos[i+1][1],
                //                 target_pos[i+1][2]);
            
                gps_lat = double(gps_position.lat)/10000000;
                gps_lon = double(gps_position.lon)/10000000;
                gps_alt = double(gps_position.alt)/1000;
                updates_check(i+1, current_pose.pose.position.x,
                                    current_pose.pose.position.y,
                                    current_pose.pose.position.z,
                                    global_position.latitude,
                                    global_position.longitude,
                                    global_position.altitude,
                                    gps_lat, gps_lon, gps_alt, rel_alt.data);
                updates_check_ss(i+1, imu_data.angular_velocity.x, 
                                      imu_data.angular_velocity.y,
                                      imu_data.angular_velocity.z,
                                      imu_data.linear_acceleration.x, 
                                      imu_data.linear_acceleration.y, 
                                      imu_data.linear_acceleration.z,
                                      mag_data.magnetic_field.x, 
                                      mag_data.magnetic_field.y, 
                                      mag_data.magnetic_field.z,
                                      static_press.fluid_pressure, 
                                      diff_press.fluid_pressure);
                // ros::Duration(5).sleep();
                while ((ros::Time::now() - t_check) < ros::Duration(10))
                {
                    std::cout << "[ INFO] Hover at checkpoint \n";
                    std::printf("[ INFO] Next local position: [%.3f, %.3f, %.3f]\n", 
                                target_pos[i+1][0], 
                                target_pos[i+1][1],
                                target_pos[i+1][2]);
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }

                i = i + 1;
                ros::spinOnce();
    		    rate.sleep();
    		}
            else if (check && final_check)
            {
                t_check = ros::Time::now();
                std::printf("[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", 
                                current_pose.pose.position.x, 
                                current_pose.pose.position.y, 
                                current_pose.pose.position.z);
    		        
                // std::printf("[ INFO] Ready to LANDING \n");
            
                gps_lat = double(gps_position.lat)/10000000;
                gps_lon = double(gps_position.lon)/10000000;
                gps_alt = double(gps_position.alt)/1000;
                updates_check(i+1, current_pose.pose.position.x,
                                   current_pose.pose.position.y,
                                   current_pose.pose.position.z,
                                   global_position.latitude,
                                   global_position.longitude,
                                   global_position.altitude,
                                   gps_lat, gps_lon, gps_alt, rel_alt.data);
                updates_check_ss(i+1, imu_data.angular_velocity.x, 
                                      imu_data.angular_velocity.y,
                                      imu_data.angular_velocity.z,
                                      imu_data.linear_acceleration.x, 
                                      imu_data.linear_acceleration.y, 
                                      imu_data.linear_acceleration.z,
                                      mag_data.magnetic_field.x, 
                                      mag_data.magnetic_field.y, 
                                      mag_data.magnetic_field.z,
                                      static_press.fluid_pressure, 
                                      diff_press.fluid_pressure);
                // ros::Duration(5).sleep();
                while ((ros::Time::now() - t_check) < ros::Duration(10))
                {
                    std::printf("[ INFO] Ready to LANDING \n");
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }

    			set_mode.request.custom_mode = "AUTO.LAND";
        	    if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
        		    std::cout << "[ INFO] AUTO.LAND enabled \n";
                    break;
                }

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
    }
    else
    {
        while (ros::ok())
        {
            std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                        global_position.latitude, 
                        global_position.longitude, 
                        global_position.altitude);
            std::printf("Next GPS position: [%f, %f, %.3f]\n", 
                                goal_pos[i][0], 
                                goal_pos[i][1],
                                goal_pos[i][2]);
            distance = measureGPS(global_position.latitude, 
                                global_position.longitude, 
                                global_position.altitude, 
                                goal_pos[i][0], goal_pos[i][1], goal_pos[i][2]);
            std::printf("Distance to next goal: %.2f m \n", distance);
            
            if (i < (goal_num - 1))
            {
                final_check = false;
                enu_goal = WGS84ToENU(goal_pos[i][0], goal_pos[i][1], goal_pos[i][2],
                           refpoint.latitude, refpoint.longitude, refpoint.altitude);
                target_pose.pose.position.x = enu_goal.x;
                target_pose.pose.position.y = enu_goal.y;
                target_pose.pose.position.z = enu_goal.z;
                
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);

                ros::spinOnce();
                rate.sleep();
            }
            else
            {
                final_check = true;
                enu_goal = WGS84ToENU(goal_pos[goal_num-1][0], 
                                      goal_pos[goal_num-1][1], 
                                      goal_pos[goal_num-1][2],
                           refpoint.latitude, refpoint.longitude, refpoint.altitude);
                target_pose.pose.position.x = enu_goal.x;
                target_pose.pose.position.y = enu_goal.y;
                target_pose.pose.position.z = enu_goal.z;
                
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);
                
                ros::spinOnce();
                rate.sleep();
            }
            enu_curr = WGS84ToENU(global_position.latitude,
                                  global_position.longitude,
                                  global_position.altitude,
                                  refpoint.latitude, 
                                  refpoint.longitude, 
                                  refpoint.altitude);
            bool check = check_position(enu_goal.x, enu_goal.y, enu_goal.z,
                                        enu_curr.x, enu_curr.y, enu_curr.z);
            std::cout << check << std::endl;
            if (check && !final_check)
            {
                t_check = ros::Time::now();
                std::printf("[ INFO] Reached position: [%f, %f, %.3f]\n", 
                                global_position.latitude, 
                                global_position.longitude, 
                                global_position.altitude);
                // std::printf("[ INFO] Next GPS position: [%f, %f, %.3f]\n", 
                //                 goal_pos[i+1][0], 
                //                 goal_pos[i+1][1],
                //                 goal_pos[i+1][2]);
                gps_lat = double(gps_position.lat)/10000000;
                gps_lon = double(gps_position.lon)/10000000;
                gps_alt = double(gps_position.alt)/1000;
                updates_check(i+1, current_pose.pose.position.x,
                                   current_pose.pose.position.y,
                                   current_pose.pose.position.z,
                                   global_position.latitude,
                                   global_position.longitude,
                                   global_position.altitude,
                                   gps_lat, gps_lon, gps_alt, rel_alt.data);
                updates_check_ss(i+1, imu_data.angular_velocity.x, 
                                      imu_data.angular_velocity.y,
                                      imu_data.angular_velocity.z,
                                      imu_data.linear_acceleration.x, 
                                      imu_data.linear_acceleration.y, 
                                      imu_data.linear_acceleration.z,
                                      mag_data.magnetic_field.x, 
                                      mag_data.magnetic_field.y, 
                                      mag_data.magnetic_field.z,
                                      static_press.fluid_pressure, 
                                      diff_press.fluid_pressure);
                // ros::Duration(5).sleep();
                while ((ros::Time::now() - t_check) < ros::Duration(10))
                {
                    std::cout << "[ INFO] Hover at checkpoint \n";
                    std::printf("[ INFO] Next GPS position: [%f, %f, %.3f]\n", 
                                goal_pos[i+1][0], 
                                goal_pos[i+1][1],
                                goal_pos[i+1][2]);
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }

                i = i + 1;
                ros::spinOnce();
                rate.sleep();
            }
            else if (check && final_check)
            {
                t_check = ros::Time::now();
                std::printf("[ INFO] Reached FINAL position: [%f, %f, %.3f]\n", 
                                global_position.latitude, 
                                global_position.longitude, 
                                global_position.altitude);
                // std::printf("[ INFO] Ready to LANDING \n");
                gps_lat = double(gps_position.lat)/10000000;
                gps_lon = double(gps_position.lon)/10000000;
                gps_alt = double(gps_position.alt)/1000;
                updates_check(i+1, current_pose.pose.position.x,
                                   current_pose.pose.position.y,
                                   current_pose.pose.position.z,
                                   global_position.latitude,
                                   global_position.longitude,
                                   global_position.altitude,
                                   gps_lat, gps_lon, gps_alt, rel_alt.data);
                updates_check_ss(i+1, imu_data.angular_velocity.x, 
                                      imu_data.angular_velocity.y,
                                      imu_data.angular_velocity.z,
                                      imu_data.linear_acceleration.x, 
                                      imu_data.linear_acceleration.y, 
                                      imu_data.linear_acceleration.z,
                                      mag_data.magnetic_field.x, 
                                      mag_data.magnetic_field.y, 
                                      mag_data.magnetic_field.z,
                                      static_press.fluid_pressure, 
                                      diff_press.fluid_pressure);
                // ros::Duration(5).sleep();
                while ((ros::Time::now() - t_check) < ros::Duration(10))
                {
                    std::printf("[ INFO] Ready to LANDING \n");
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }

                set_mode.request.custom_mode = "AUTO.LAND";
                if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    std::cout << "[ INFO] AUTO.LAND enabled \n";
                    break;
                }

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
    }

    return 0;
}