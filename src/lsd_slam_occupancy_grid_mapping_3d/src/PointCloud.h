#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <deque>
#include <vector>
#include <string>

struct Point{

	double x;
	double y;
	double z;


};

struct Quad{
	std::string name;
	double x;
	double y;
	double z;
	double sizeX;
	double sizeY;
	double sizeZ;
};

class PointCloud {

		public:
		
		void setPoint(double x, double y, double z);
		void setSortedPoint(double x, double y, double z);
		void setPoints(std::vector<Point> points);
		std::vector<Point> getPoints();
		std::deque<Point> getSortedPoints();
		private:
		
		std::vector<Point> points;
		std::deque<Point> sortedPoints;
	
};

#endif

