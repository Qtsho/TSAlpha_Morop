#ifndef MARKERS_NODE_H
#define MARKERS_NODE_H

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sstream>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ts_alpha/SensorSettings.h>


struct Point {
	float x;
	float y;
	float z;
	float v;
	ros::Time stamp;
};


class PclHandler
{
protected:
	ros::NodeHandle nh;
	ros::Publisher pcl_pub;
	ros::Publisher cloud_pub;
	ros::ServiceServer settings_srv;
	std::string dev;
	std::string frame;
	int usb_stream;
	std::stringstream dataStream;
	std::vector<Point> newPoints;
	std::vector<Point> points;
	sensor_msgs::PointCloud2 pcl_msg;
	float lifetime;
	float volume_devider;
	float max_volume;
	float volume_offset;

public:
	PclHandler();
	void serialConnect();
	void publishingPointCloud();
	void serialSendCommand();
	void serialGetData();
	void stringToPoints();
	void updatePoints();
	void publishMarkers();
	bool changeSettingsTrigger(ts_alpha::SensorSettings::Request &req,
						ts_alpha::SensorSettings::Response &res);
	void changeSettings(std::string settings_cmd);
	void closeStream();
	void shutdown();
	int count = 0;
	bool stillConnected;
	std::string new_settings_cmd;
	bool bChangeSettings;
};

#endif
