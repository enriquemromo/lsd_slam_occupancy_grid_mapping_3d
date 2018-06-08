#include <stdio.h>
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "std_msgs/String.h"
#include "settings.h"
#include <vector>
#include <fstream>
#include "lsd_slam_occupancy_grid_mapping_3d/PointCloudMsg.h"
#include "lsd_slam_occupancy_grid_mapping_3d/PointCloudArrayMsg.h"
#include <qapplication.h>
#include <QtGui>
#include "OccupancyGridMapping3D.h"
#include "PointCloud.h"

/**
* This tutorial demonstrates simple receipt of messages over the ROS system.
*/

PointCloud* pointCloud;
OccupancyGridMapping3D* occupancyGridMapping3D = 0;

using namespace std;
using namespace ros;

void readFile()
{
	std::string delimiter = ",";
	string line;
	int limit = 0;
ifstream myfile ("/home/toto/laboratorioSLAMFullFormated.ply");
//	ifstream myfile ("/home/toto/nubeDePrueba.ply");
	int index = 0;
	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
    		{
			size_t pos = 0;
			std::string token;
			string data[3];
			int index = 0;
				
			//	cout << line<<endl;
			

			while ((pos = line.find(delimiter)) != std::string::npos) 
			{
				token = line.substr(0, pos);
				data[index]=token;
				line.erase(0, pos + delimiter.length());
				index++;
			}
			
			data[index] = line;
	
	
	
			double x = strtof((data[0]).c_str(),0);
			double y = strtof((data[1]).c_str(),0);
			double z = strtof((data[2]).c_str(),0);
			index++;
			limit ++;
			/*
			if(limit==50000)
			{
				break;
			}*/
			//ROS_INFO_STREAM("x: " << x << "y " << y << "z " << z );
			//Rotate
			if(rotated){
			x = x * -1;
            y = y * -1;
			}
            pointCloud->setSortedPoint(x, y, z);

 		}
		myfile.close();

	}
}



void poseLSDSlamCallback(const lsd_slam_occupancy_grid_mapping_3d::PointCloudArrayMsg::ConstPtr& msg)
{
	for (int i=0; i<msg->points.size(); ++i)
    {
      const lsd_slam_occupancy_grid_mapping_3d::PointCloudMsg &data = msg->points[i];
      ROS_INFO_STREAM("x: " << data.x << "y " << data.y <<
                      "z " << data.z );
                      //Rotate
                     
                     double x = data.x * -1;
                     double y = data.y * -1;
                     pointCloud->setSortedPoint(x, y, data.z);
    }
    
	if(occupancyGridMapping3D != 0 && !msg->points.empty())
	{
		occupancyGridMapping3D->process();
	}
}

void createOccupancyGridMapping3D( int argc, char** argv )
{	
		printf("Started ROS thread\n");
		ros::init(argc, argv, "occupy_3d");
		ROS_INFO("lsd_slam_occupancy_grid_mapping_3d started");
		ros::NodeHandle n;

	 	Subscriber liveFrames_sub = n.subscribe(n.resolveName("/lsd_slam/pointcloud"),1, poseLSDSlamCallback);
		ros::spin();
		printf("Exiting ROS thread\n");

	exit(1);
		
}


void loop()
{
	if(occupancyGridMapping3D != 0)
	{
		while(true)
		{
			occupancyGridMapping3D->process();
			
			boost::this_thread::sleep( boost::posix_time::milliseconds(1800000) );
			
		}

	}
}


int main(int argc, char **argv)
{
	pointCloud = new PointCloud();
		
	if(test)
	{
		readFile();
	} 
	else 
	{
		ROS_INFO("Entro");
		boost::thread rosThread  = boost::thread(createOccupancyGridMapping3D, argc, argv);
	}
	QApplication application(argc, argv);
	occupancyGridMapping3D = new OccupancyGridMapping3D();
	occupancyGridMapping3D->setWindowTitle("Occupancy Grid Mapping 3D");
	occupancyGridMapping3D->setPoint(pointCloud);
	occupancyGridMapping3D->show();
	boost::thread t1 = boost::thread(loop);
	application.exec();
	return 0;
}


