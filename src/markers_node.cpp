#include "ts_alpha/markers_node.h"


PclHandler::PclHandler()
{
	ros::NodeHandle private_nh("~");

	// Get/Set parameters
	private_nh.param<std::string>("dev", dev, "/dev/ttyUSB0"); 					//Serial port that is used
	private_nh.param<std::string>("frame_id", frame, "ts_alpha");				//Frame for tf
	private_nh.param<float>("lifetime", lifetime, 0.1); 						 //Period of time for PCL
	private_nh.param<float>("volume_devider", volume_devider, 100.0);
	private_nh.param<float>("max_volume", max_volume, 20.0);
	private_nh.param<float>("volume_offset", volume_offset, 5.0);

	// Advertise topic for MarkerArray-Messages
	cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 100);
	pcl_pub = nh.advertise<visualization_msgs::MarkerArray>("ts_alpha_markers", 100);
	//Advertise topic cloud for point cloud message for navigation
	
	// Advertise service to change sensor settings
	settings_srv = nh.advertiseService("change_sensor_settings", &PclHandler::changeSettingsTrigger, this);

	pcl_msg.header.frame_id = frame;

	bChangeSettings = false;
}

void PclHandler::serialConnect()
{
	// Open serial port
	usb_stream = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if (usb_stream == -1)
	{
		ROS_ERROR("Could not connect to USB Device: %s", dev.c_str());
		//ROS_INFO("[INFO] Try reconnecting the USB Device and starting the node again");
	}
	else
	{
		ROS_INFO("Connected to USB Device: %s", dev.c_str());
		ROS_INFO("Frame ID is: %s", frame.c_str());
		ROS_INFO("Lifetime is: %f", lifetime);
		ROS_INFO("Maximum volume is: %f", max_volume);
		ROS_INFO("Volume offset is: %f", volume_offset);
		ROS_INFO("Volume devider is: %f", volume_devider);

		// Set options of serial data transfer
		struct termios options;
		tcgetattr(usb_stream, &options);
		options.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
		options.c_iflag = IGNPAR;
		options.c_oflag = 0;
		options.c_lflag = 0;
		tcflush(usb_stream, TCIFLUSH);
		tcsetattr(usb_stream, TCSANOW, &options);

		stillConnected = true;
	}

	//ROS_INFO("[INFO] Connecetion successful");
}


void PclHandler::serialSendCommand()
{
	// Send command to sensor for scanning 1 frame
	int tx_length = -1;
	if (usb_stream != -1)
	{
    	tx_length = write(usb_stream, "scanmode:3\r\n", 11);
    }

    //usleep(10 * 1000);
	if (tx_length = 11)
	{
		//ROS_INFO("[INFO] Sending scan-command successful");
	}
	else if (tx_length < 0)
	{
		ROS_ERROR("Scan-command could not be sent");
	}
}

void PclHandler::serialGetData() // Read data from serial port
{
	if (usb_stream != -1)
	{
		unsigned char BUF_RX[10000];
		int rx_length;
		ros::Time startTime	= ros::Time::now();

		// Read until end of frame ('E')
		do
		{
			memset(BUF_RX, 0, 10000);
			rx_length = 0;
			rx_length = read(usb_stream, (void*)BUF_RX, 10000);
			dataStream << BUF_RX;
			// Break if no data received
			if (ros::Time::now() - ros::Duration(1) > startTime)
			{
				ROS_ERROR("No data received");
				stillConnected = false;
				break;
			}
		}
		while (BUF_RX[rx_length-1] != 'E');
		//ROS_INFO("[INFO] Receiving serial data successful:");
		//ROS_INFO("%s", dataStream.str().c_str());
	}
}

void PclHandler::stringToPoints() // Parse string to points
{
	struct Point currentPoint;
	struct Point emptyPoint;
	std::string inString;
	bool negative = false;

	std::string strData = dataStream.str();
	newPoints.clear();

	//ROS_INFO("%d",strData.length());

	for (int i = 0; i < strData.length(); i++)
	{
		if (strData[i] == '-')
		{
			strData[i] = '0';
			negative = true;
		}
		if (isdigit(strData[i]))
		{
			inString += (char)strData[i];
		}
		if (strData[i] == 'Y'){
	  		if (negative)
			{
				currentPoint.x = -atof(inString.c_str())/1000;
	  		}
	  		else
			{
				currentPoint.x = atof(inString.c_str())/1000;
	  		}
	  		inString = "";
	  		negative = false;
		}
		if (strData[i] == 'Z')
		{
			if (negative){
				currentPoint.y = -atof(inString.c_str())/1000;
			}
			else
			{
				currentPoint.y = atof(inString.c_str())/1000;
			}
			inString = "";
			negative = false;
		}
		if (strData[i] == 'V')
		{
			if (negative)
			{
				currentPoint.z = -atof(inString.c_str())/1000;
			}
			else
			{
				currentPoint.z = atof(inString.c_str())/1000;
			}
			inString = "";
			negative = false;

		}
		if (strData[i] == 'X')
		{
	  		inString = "";
		}
		if ((strData[i] == 'P' && inString != "3000000") || strData[i] == 'E')
		{
			currentPoint.v = atof(inString.c_str());
			inString = "";
			currentPoint.stamp = ros::Time::now();
			if (currentPoint.v > 0)
			{
				newPoints.push_back(currentPoint);
			}
			currentPoint = emptyPoint;
		}
  	}
	dataStream.str( std::string() );
	dataStream.clear();

	//ROS_INFO("[INFO] Parsing string successful");
}

void PclHandler::updatePoints() // Update vector of points
{
	// insert new points
	for (int i = 0; i < newPoints.size(); i++)
	{
		if (newPoints[i].z >= 0.15)
		{
			points.push_back(newPoints[i]);
		}
	}
	// delete outdated points
	if (points.size() > 0)
	{
		ros::Time currentTime = ros::Time::now();
		while (currentTime - points[0].stamp > ros::Duration(lifetime))
		{
			for (int i = 0; i < points.size() - 1; i++)
			{
				points[i] = points[i+1];
			}
			points.pop_back();
			if (points.size() == 0)
			{
				break;
			}
		}
	}
	//ROS_INFO("[INFO] Updating PCL successful");
}

void PclHandler::publishMarkers() // Publish vector of points as Marker Array
{
	visualization_msgs::MarkerArray marker_array;

	for (int i = 0; i < points.size(); i++)
	{
		visualization_msgs::Marker marker;
		

		marker.header.frame_id = frame;
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
   	 	marker.id = i;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;

		// TS Alpha uses different coordinate frame than the cartesian coordinate frame that is used in ROS
		marker.pose.position.x = points[i].z;
		marker.pose.position.y = -points[i].x;
		marker.pose.position.z = points[i].y;

		
		if ((points[i].v + volume_offset) < max_volume)
		{
			marker.scale.x = (points[i].v + volume_offset)/volume_devider;
    	marker.scale.y = (points[i].v + volume_offset)/volume_devider;
    	marker.scale.z = (points[i].v + volume_offset)/volume_devider;
		}
		else
		{
			marker.scale.x = (max_volume + volume_offset)/volume_devider;
    	marker.scale.y = (max_volume + volume_offset)/volume_devider;
    	marker.scale.z = (max_volume + volume_offset)/volume_devider;
		}

		marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

		marker.lifetime = ros::Duration(lifetime);

		marker_array.markers.push_back(marker);

		//ROS_INFO("X: %f, Y: %f, Z: %f, V: %f", points[i].z, -points[i].x, points[i].y, points[i].v);
	}

	pcl_pub.publish(marker_array);
}

void PclHandler::publishingPointCloud() // Publish vector of points as Marker Array
{
	sensor_msgs::PointCloud cloud;

	cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";
    cloud.points.resize(points.size());
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(points.size());

    for(unsigned int i = 0; i < points.size(); ++i){
      	cloud.points[i].x = 1 + count;
      	cloud.points[i].y = 2 + count;
      	cloud.points[i].z = 3 + count;
      	cloud.channels[0].values[i] = 100 + count;
    }
    cloud_pub.publish(cloud);
    ++count;
    
}


// Callback-function if service is called to change sensor-settings
bool PclHandler::changeSettingsTrigger(ts_alpha::SensorSettings::Request &req,
						ts_alpha::SensorSettings::Response &res)
{
	new_settings_cmd = req.command;
	bChangeSettings = true;
	res.success = true;
	return true;
}

void PclHandler::changeSettings(std::string settings_cmd)
{
	if (bChangeSettings)
	{
		if (usb_stream != -1)
		{
    	int tx_length = write(usb_stream, (settings_cmd + "\r").c_str(), settings_cmd.length() + 1);

			if (tx_length >= 0)
			{
				ROS_INFO("Sending settings-command '%s' successful", settings_cmd.c_str());
			}
			else
			{
				ROS_ERROR("Settings-command could not be sent");
			}
    }
		//ROS_INFO("Sensor settings changed");
		bChangeSettings = false;
	}
}

void PclHandler::closeStream()
{
	int closed = close(usb_stream);
}

void PclHandler::shutdown()
{
	if (usb_stream != -1)
	{
		int out = write(usb_stream, "\r\n", 1);
		usleep(80 * 1000);
		unsigned char BUF_RX[200];
  	int rx_length = read(usb_stream, (void*)BUF_RX, 200);
		int closed = close(usb_stream);
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ts_alpha_node");

	PclHandler pcl_handler;

	ros::Rate loop_rate(20);

	pcl_handler.serialConnect();

	// Send initial settings
	pcl_handler.bChangeSettings = true;
	pcl_handler.changeSettings("CnWave00008");
	pcl_handler.bChangeSettings = true;
	pcl_handler.changeSettings("Cfiltr00005");
	pcl_handler.bChangeSettings = true;
	pcl_handler.changeSettings("CdThre00002");
	pcl_handler.changeSettings("");

	while (ros::ok()) {

		if (!pcl_handler.stillConnected) {
			pcl_handler.closeStream();
			pcl_handler.serialConnect();
		}

		pcl_handler.serialGetData();
		pcl_handler.stringToPoints();
		pcl_handler.updatePoints();
		pcl_handler.publishMarkers();
		pcl_handler.publishingPointCloud();
		pcl_handler.changeSettings(pcl_handler.new_settings_cmd);

		ros::spinOnce();
		loop_rate.sleep();
	}

	pcl_handler.shutdown();

	return 0;
}
