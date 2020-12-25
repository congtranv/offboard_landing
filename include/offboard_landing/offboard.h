/***** ros *****/
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
/***** local position *****/
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
/***** global position *****/
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
/***** tools *****/
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
/***** c++ *****/
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdio>

/****** DEFINE CONSTANTS ******/
const double PI  =3.141592653589793238463;
const double eR  =6378.137; //km

/****** DEFINE FUNCTIONS ******/
bool check_position(double, double, double,
				    double, double, double);
bool check_orientation(double, double, double, double,
				       double, double, double, double);
bool check_global(double, double, double,
				  double, double, double);

void input_local_target(void);
void input_global_target(void);
void input_target(void);

double degree(double);
double radian(double);

double measureGPS(double, double, double, double, double, double);
void files(std::string, double, double, double,
						double, double, double);

/****** DECLARE VARIANTS ******/
mavros_msgs::State current_state;
mavros_msgs::SetMode set_mode;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

bool global_position_received = false;

sensor_msgs::NavSatFix global_position;
geographic_msgs::GeoPoseStamped goal_position;

sensor_msgs::BatteryState current_batt;

tf::Quaternion q;

int target_num;
float target_pos[10][7];
double roll, pitch, yaw;
double r, p, y;

int goal_num;
double goal_pos[10][3];
double latitude, longitude, altitude, distance;
bool input_type = true;

float batt_percent;

/****** FUNCTIONS ******/

/***** callback functions *****/
// state callback 
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// local pose callback
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

// global position callback
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    global_position = *msg;
    global_position_received = true;
}

// battery status callback
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) 
{
    current_batt = *msg;
}

/****************************************************************************************************/
/***** check_position: check when drone reached the target positions. return the true or false ******/
/****************************************************************************************************/
bool check_position(double target_x, double target_y, double target_z,
					double currentx, double currenty, double currentz)
{
	bool reached;
	if(((target_x - 0.1) < currentx)
	&& (currentx < (target_x + 0.1)) 
	&& ((target_y - 0.1) < currenty)
	&& (currenty < (target_y + 0.1))
	&& ((target_z - 0.1) < currentz)
	&& (currentz < (target_z + 0.1)))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

/**********************************************************************************************************/
/***** check_orientation: check when drone reached the target orientations. return the true or false ******/
/**********************************************************************************************************/
bool check_orientation(double target_x, double target_y, double target_z, double target_w,
					   double currentx, double currenty, double currentz, double currentw)
{
	bool reached;
	// tf Quaternion to RPY
	tf::Quaternion qc(currentx,
					  currenty,
					  currentz,
					  currentw);
	tf::Matrix3x3 mc(qc);
	double rc, pc, yc;
	mc.getRPY(rc, pc, yc);

	tf::Quaternion qt(target_x,
					  target_y,
					  target_z,
					  target_w);
	tf::Matrix3x3 mt(qt);
	double rt, pt, yt;
	mt.getRPY(rt, pt, yt);
	// check
	if((((degree(rt)-1)<(degree(rc)))&&(degree(rc)<(degree(rt)+1)))
	 &&(((degree(pt)-1)<(degree(pc)))&&(degree(pc)<(degree(pt)+1)))
	 &&(((degree(yt)-1)<(degree(yc)))&&(degree(yc)<(degree(yt)+1)))) 
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

/****************************************************************************************************/
/***** check_global: check when drone reached the GPS goal positions. return the true or false ******/
/****************************************************************************************************/
bool check_global(double lat, double lon, double alt,
				double lat0, double lon0, double alt0)
{
	bool reached;
	if(
		((lat - 0.000001) < lat0)
	 && (lat0 < (lat + 0.000001)) 
	 && ((lon - 0.000001) < lon0)
	 && (lon0 < (lon + 0.000001))
	 && ((alt - 0.1) < alt0)
	 && (alt0 < (alt + 0.1))
	)
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

// bool check_global(double goal_x, double goal_y, double goal_z,
// 				  double curr_x, double curr_y, double curr_z)
// {
// 	bool reached;
// 	if(((goal_x - 0.1) < curr_x)
// 	&& (curr_x < (goal_x + 0.1)) 
// 	&& ((goal_y - 0.1) < curr_y)
// 	&& (curr_y < (goal_y + 0.1))
// 	&& ((goal_z - 0.1) < curr_z)
// 	&& (curr_z < (goal_z + 0.1)))
// 	{
// 		reached = 1;
// 	}
// 	else
// 	{
// 		reached = 0;
// 	}
// 	return reached;
// }
/**************************************************************************/
/***** input_local_target: input the number of waypoints and each point   *
 * coodinates (x, y, z). and input the yaw rotation at each waypoint ******/
/**************************************************************************/
void input_local_target()
{
	std::cout << "Input Local position(s)" << std::endl;
	std::cout << "Number of target(s): "; std::cin >> target_num;
	for (int i = 0; i < target_num; i++)
	{
		std::cout << "Target (" << i+1 << ") position (in meter):" <<std::endl; 
		std::cout << "pos_x_" << i+1 << ":"; std::cin >> target_pos[i][0];
		std::cout << "pos_y_" << i+1 << ":"; std::cin >> target_pos[i][1];
		std::cout << "pos_z_" << i+1 << ":"; std::cin >> target_pos[i][2];

		// std::cout << "Target (" << i+1 << ") orientation (in degree):" <<std::endl; 
		target_pos[i][3] = 0;
		target_pos[i][4] = 0;
		target_pos[i][5] = 0;
		// std::cout << "yaw_" << i+1 << ":"; std::cin >> target_pos[i][5];
	}
}

/****************************************************************************************/
/***** input_global_target: input the GPS [latitude, longitude, altitude] point(s) ******/
/****************************************************************************************/
// void input_global_target()
// {
// 	std::cout << "Input GPS position" << std::endl;
// 	std::cout << "Latitude  (degree):"; std::cin >> latitude;
// 	std::cout << "Longitude (degree):"; std::cin >> longitude;
// 	std::cout << "Altitude  (meter) :"; std::cin >> altitude;
// }

void input_global_target()
{
	std::cout << "Input GPS position(s)" << std::endl;
	std::cout << "Number of goal(s): "; std::cin >> goal_num;
	for (int i = 0; i < goal_num; i++)
	{
		std::cout << "Goal ("<< i+1 <<") position:" << std::endl;
		std::cout << "Latitude  " << i+1 << " (in degree): "; std::cin >> goal_pos[i][0];
		std::cout << "Longitude " << i+1 << " (in degree): "; std::cin >> goal_pos[i][1];
		std::cout << "Altitude  " << i+1 << "  (in meter): "; std::cin >> goal_pos[i][2];
	}
}

void input_target()
{
	char c;
	std::cout << "Waypoint type: (1) Local || (2) Global ? (1/2) \n"; std::cin >> c;
	if (c == '1')
	{
		input_local_target();
		input_type = true;
	}
	else if (c == '2')
	{
		input_global_target();
		input_type = false;
	}
	else 
	{
		input_target();
	}
}

/**********************************************************/
/***** degree: convert angle from radian to degree ******/
/**********************************************************/
double degree(double rad)
{
	double radian_to_degree = (rad*180)/PI;
	return radian_to_degree;
}

/**********************************************************/
/***** radian: convert angle from degree to radian ******/
/**********************************************************/
double radian(double deg)
{
	double degree_to_radian = (deg*PI)/180;
	return degree_to_radian;
}

/*********************************************************************************************/
/***** measureGPS: measure the distance between 2 GPS points that use haversine formula ******/
/*********************************************************************************************/
double measureGPS(double lat1, double lon1, double alt1, 
				  double lat2, double lon2, double alt2)
{
	double flat, plus, Distance;
	lat1 = radian(lat1); lon1 = radian(lon1);
	lat2 = radian(lat2); lon2 = radian(lon2);
	flat = 2*eR*asin(sqrt(sin((lat2-lat1)/2)*sin((lat2-lat1)/2)+cos(lat1)*cos(lat2)*sin((lon2-lon1)/2)*sin((lon2-lon1)/2)))*1000; //m
	if (alt1 == alt2)
	{
		Distance = flat;
	}
	else
	{
		if 	(alt1 > alt2)
		{
			plus = flat/((alt1/alt2)-1);
			Distance = sqrt((flat+plus)*(flat+plus) + alt1*alt1) - sqrt(plus*plus + alt2*alt2);
		}
		else
		{
			plus = flat/((alt2/alt1)-1);
			Distance = sqrt((flat+plus)*(flat+plus) + alt2*alt2) - sqrt(plus*plus + alt1*alt1);
		}
	}
	return Distance;
}

/****************************************************************************************/
/***** files: write local position and global position at the setpoint into a file ******/
/****************************************************************************************/
void files(std::string name, double x, double y, double z,
						double lat, double lon, double alt)
{
	std::fstream file;
	name = name + ".yaml";
	file.open(name, std::ios::app);
	if (file.is_open())
	{
		file << "Local : " << x << ", " << y << ", " << z << std::endl;
		file << "Global: " << lat << ", " << lon << ", " << alt << std::endl;
		file.close();
	}
	else std::cout << "Unable to open file \n";
}