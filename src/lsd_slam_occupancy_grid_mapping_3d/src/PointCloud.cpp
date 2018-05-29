
#include <vector>
#include "PointCloud.h"

using namespace std;

void PointCloud::setPoint(double x, double y, double z)
{
	Point point;
	
	point.x = x;
	point.y = y;
	point.z = z;

	this->points.push_back(point);
}
void PointCloud::setSortedPoint(double x, double y, double z)
{
	Point point;
	point.x = x;
	point.y = y;
	point.z = z;
	this->sortedPoints.push_back(point);
}

void PointCloud::setPoints(vector<Point> points)
{
	this->points.insert(this->points.end(), points.begin(), points.end());

	
}



vector<Point> PointCloud::getPoints()
{
	return this->points;
}

deque<Point> PointCloud::getSortedPoints()
{
	return this->sortedPoints;
}

