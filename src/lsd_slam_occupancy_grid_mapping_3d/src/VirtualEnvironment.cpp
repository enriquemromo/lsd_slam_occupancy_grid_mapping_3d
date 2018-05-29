#include "VirtualEnvironment.h"
#include <iostream> 
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream> 
#include <ros/console.h>
#include "settings.h"
#include <algorithm>

using std::cout;
using std::cerr;
using std::ios;
using std::endl;
using std::fstream;


bool my_compareY(const struct Point &left, const struct Point &right)
{
	return (left.y < right.y);
}

void VirtualEnvironment::generate(std::vector<Point> glQuads)
{
	double distance;
	double diference;
	std::vector<Point>::iterator it;
	
	fstream world("environment.wordl",ios::out);
	
	if(!world)
	{
		cerr << "No se pudo crear archivo " << endl; 
		exit(1);
		
	}
	sort (glQuads.begin(), glQuads.end(), my_compareY); 
	
	Point firtsPoint = glQuads.front();
	diference = -(firtsPoint.y);
	distance = diference + (voxelSize / 2);
	
	std::stringstream ss;
	int index = 0; 
	
	world << "<sdf version='1.4'> " << endl;
	world << "  <world name='default'> " << endl;
	world << "    <light name='sun' type='directional'> " << endl;
	world << "      <cast_shadows>1</cast_shadows> " << endl;
	world << "      <pose>0 0 10 0 -0 0</pose>" << endl;
	world << "      <diffuse>0.8 0.8 0.8 1</diffuse>" << endl;
	world << "      <specular>0.2 0.2 0.2 1</specular>" << endl;
	world << "      <attenuation>" << endl;
	world << "        <range>1000</range>" << endl;
	world << "        <constant>0.9</constant>" << endl;
	world << "        <linear>0.01</linear>" << endl;
	world << "        <quadratic>0.001</quadratic>" << endl;
	world << "      </attenuation>" << endl;
	world << "      <direction>-0.5 0.1 -0.9</direction>" << endl;
	world << "    </light>" << endl;
	world << "    <model name='ground_plane'>" << endl;
	world << "      <static>1</static>" << endl;
	world << "      <link name='link'>" << endl;
	world << "        <collision name='collision'>" << endl;
	world << "          <geometry>" << endl;
	world << "            <plane>" << endl;
	world << "              <normal>0 0 1</normal>" << endl;
	world << "              <size>100 100</size>" << endl;
	world << "            </plane>" << endl;
	world << "          </geometry>" << endl;
	world << "          <surface>" << endl;
	world << "            <friction>" << endl;
	world << "              <ode>" << endl;
	world << "                <mu>100</mu>" << endl;
	world << "                <mu2>50</mu2>" << endl;
	world << "              </ode>" << endl;
	world << "            </friction>" << endl;
	world << "            <bounce/>" << endl;
	world << "            <contact>" << endl;
	world << "              <ode/>" << endl;
	world << "            </contact>" << endl;
	world << "          </surface>" << endl;
	world << "          <max_contacts>10</max_contacts>" << endl;
	world << "        </collision>" << endl;
	world << "        <visual name='visual'>" << endl;
	world << "          <cast_shadows>0</cast_shadows>" << endl;
	world << "          <geometry>" << endl;
	world << "            <plane>" << endl;
	world << "              <normal>0 0 1</normal>" << endl;
	world << "              <size>100 100</size>" << endl;
	world << "            </plane>" << endl;
	world << "          </geometry>" << endl;
	world << "          <material>" << endl;
	world << "          	<script>Gazebo/WoodPallet</script>" << endl;
	world << "          </material>" << endl;
	world << "        </visual>" << endl;
	world << "        <velocity_decay>" << endl;
	world << "          <linear>0</linear>" << endl;
	world << "          <angular>0</angular>" << endl;
	world << "        </velocity_decay>" << endl;
	world << "        <self_collide>0</self_collide>" << endl;
	world << "        <kinematic>0</kinematic>" << endl;
	world << "        <gravity>0</gravity>" << endl;
	world << "      </link>" << endl;
	world << "    </model>" << endl;
	world << "    <physics type='ode'>" << endl;
	world << "      <max_step_size>0.001</max_step_size>" << endl;
	world << "      <real_time_factor>1</real_time_factor>" << endl;
	world << "      <real_time_update_rate>1000</real_time_update_rate>" << endl;
	world << "      <gravity>0 0 -9.8</gravity>" << endl;
	world << "    </physics>" << endl;
	world << "    <scene>" << endl;
	world << "      <ambient>0.4 0.4 0.4 1</ambient>" << endl;
	world << "      <background>0.7 0.7 0.7 1</background>" << endl;
	world << "      <shadows>1</shadows>" << endl;
	world << "    </scene>" << endl;
	world << "    <spherical_coordinates>" << endl;
	world << "      <surface_model>EARTH_WGS84</surface_model>" << endl;
	world << "      <latitude_deg>0</latitude_deg>" << endl;
	world << "      <longitude_deg>0</longitude_deg>" << endl;
	world << "      <elevation>0</elevation>" << endl;
	world << "      <heading_deg>0</heading_deg>" << endl;
	world << "    </spherical_coordinates>" << endl;
	
	for(it = glQuads.begin(); it != glQuads.end(); ++it )
	{
		
		ss << index;
		world << "    <model name='"<< "unit_box_" + ss.str() <<"'>" << endl;
		world << "      <pose>"<<(*it).x<<  " " << (*it).z <<" "
		<< ((*it).y + distance)<<" 0 -0 0</pose>" << endl;
		world << "      <link name='link'>" << endl;
		world << "        <inertial>" << endl;
		world << "          <mass>50</mass>" << endl;
		//world << "          <inertia>" << endl;
		//world << "            <ixx>1</ixx>" << endl;
		//world << "            <ixy>0</ixy>" << endl;
		//world << "            <ixz>0</ixz>" << endl;
		//world << "            <iyy>1</iyy>" << endl;
		//world << "            <iyz>0</iyz>" << endl;
		//world << "            <izz>1</izz>" << endl;
		//world << "          </inertia>" << endl;
		world << "        </inertial>" << endl;
		world << "        <collision name='collision'>" << endl;
		world << "          <geometry>" << endl;
		world << "            <box>" << endl;
		world << "              <size>"<< voxelSize <<  " " << voxelSize <<" "
		<< voxelSize <<"</size>" << endl;
		world << "            </box>" << endl;
		world << "          </geometry>" << endl;
		world << "          <max_contacts>10</max_contacts>" << endl;
		//world << "          <surface>" << endl;
		//world << "            <bounce/>" << endl;
		//world << "            <friction>" << endl;
		//world << "              <ode/>" << endl;
		//world << "            </friction>" << endl;
		//world << "            <contact>" << endl;
		//world << "              <ode/>" << endl;
		//world << "            </contact>" << endl;
		//world << "          </surface>" << endl;
		world << "        </collision>" << endl;
		world << "        <visual name='visual'>" << endl;
		world << "          <geometry>" << endl;
		world << "            <box>" << endl;
		world << "              <size>"<<voxelSize <<  " " << voxelSize <<" "
		<< voxelSize <<"</size>" << endl;
		world << "            </box>" << endl;
		world << "          </geometry>" << endl;
		world << "          <material>" << endl;
		world << "          	<script>Gazebo/WoodPallet</script>" << endl;
		world << "          </material>" << endl;
		world << "        </visual>" << endl;
		//world << "        <velocity_decay>" << endl;
		//world << "          <linear>0</linear>" << endl;
		//world << "          <angular>0</angular>" << endl;
		//world << "        </velocity_decay>" << endl;
		world << "        <self_collide>0</self_collide>" << endl;
		world << "        <kinematic>0</kinematic>" << endl;
		world << "        <gravity>0</gravity>" << endl;	
		world << "      </link>" << endl;
		world << "      <static>1</static>" << endl;
		world << "    </model>" << endl;
		index++;
		ss.str(std::string());
	}
	world << "" << endl;
	world << "    <state world_name='default'>" << endl;
	world << "      <sim_time>115 280000000</sim_time>" << endl;
	world << "      <real_time>33 498449766</real_time>" << endl;
	world << "      <wall_time>1503598739 954665075</wall_time>" << endl;
	world << "      <model name='ground_plane'>" << endl;
	world << "        <pose>0 0 0 0 -0 0</pose>" << endl;
	world << "        <link name='link'>" << endl;
	world << "          <pose>0 0 0 0 -0 0</pose>" << endl;
	world << "          <velocity>0 0 0 0 -0 0</velocity>" << endl;
	world << "          <acceleration>0 0 0 0 -0 0</acceleration>" << endl;
	world << "          <wrench>0 0 0 0 -0 0</wrench>" << endl;
	world << "        </link>" << endl;
	world << "      </model>" << endl;
	
	index = 0;

	for(it = glQuads.begin(); it != glQuads.end(); ++it )
	{
		ss << index;
		world << "    <model name='"<< "unit_box_" + ss.str() <<"'>" << endl;
		world << "      <pose>"<<(*it).x <<  " " << (*it).z << " "
		<< ((*it).y + distance)<<" 0 -0 0</pose>" << endl;
		world << "        <link name='link'>" << endl;
		world << "      <pose>"<<(*it).x<<  " " << (*it).z <<" "
		<< ((*it).y + distance)<<" 0 -0 0</pose>" << endl;
		world << "          <velocity>0 0 0 0 -0 0</velocity>" << endl;
		world << "          <acceleration>0 0 0 0 -0 0</acceleration>" << endl;
		world << "          <wrench>0 0 0 0 -0 0</wrench>" << endl;
		world << "        </link>" << endl;
		world << "      </model>" << endl;
		index++;
		ss.str(std::string());
	}
	

	world << "" << endl;
	world << "    </state>" << endl;
	world << "    <gui fullscreen='0'>" << endl;
	world << "      <camera name='user_camera'>" << endl;
	world << "        <pose>17.935 -13.8855 4.50806 0 0.275643 2.35619</pose>" << endl;
	world << "        <view_controller>orbit</view_controller>" << endl;
	world << "      </camera>" << endl;
	world << "    </gui>" << endl;
	world << "  </world>" << endl;
	world << "</sdf>" << endl;

	
	ROS_INFO("Entorno Generado");
	
}
