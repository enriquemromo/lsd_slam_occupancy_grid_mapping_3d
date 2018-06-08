#include "OccupancyGridMapping3D.h"
#include "VirtualEnvironment.h"
#include <algorithm>
#include <QGLViewer/manipulatedFrame.h>
#include <ros/console.h>
#include <ctime>
#include <math.h>

using namespace qglviewer;
using namespace std;

bool my_compare(const struct Point &left, const struct Point &right)
{
	if (left.x == right.x)
	{
		if (left.y == right.y)
		{
			return (left.z < right.z);
		}
		else
		{
			return (left.y < right.y);
		}
	}
	return (left.x < right.x);
}

bool my_compareX(const struct Point &left, const struct Point &right)
{
	return (left.x < right.x);
}
bool my_compareY(const struct Point &left, const struct Point &right)
{
	return (left.y < right.y);
}

bool my_compareZ(const struct Point &left, const struct Point &right)
{
	return (left.z < right.z);
}

static void drawQuad(float x, float y, float z)
{

	float size = voxelSize / 2;

	//glClear ( GL_COLOR_BUFFER_BIT ) ;
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	//glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

	//glTranslatef(0.0f,0.0f,-4.0f);//move forward 4 units

	//glColor3f(0.0f,0.0f,1.0f); //blue color

	//glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
	// Top face (y = 1.0f)
	// Define vertices in counter-clockwise (CCW) order with normal pointing out
	glColor3f(0.0f, 1.0f, 0.0f); // Green
	glVertex3f(x + size, y + size, z - size);
	glVertex3f(x - size, y + size, z - size);
	glVertex3f(x - size, y + size, z + size);
	glVertex3f(x + size, y + size, z + size);
	// Bottom face (y = -1.0f)
	glColor3f(1.0f, 0.5f, 0.0f); // Orange

	glVertex3f(x + size, y - size, z + size);
	glVertex3f(x - size, y - size, z + size);
	glVertex3f(x - size, y - size, z - size);
	glVertex3f(x + size, y - size, z - size);

	// Front face  (z = 1.0f)
	glColor3f(1.0f, 0.0f, 0.0f); // Red

	glVertex3f(x + size, y + size, z + size);
	glVertex3f(x - size, y + size, z + size);
	glVertex3f(x - size, y - size, z + size);
	glVertex3f(x + size, y - size, z + size);

	// Back face (z = -1.0f)
	glColor3f(1.0f, 1.0f, 0.0f); // Yellow

	glVertex3f(x + size, y - size, z - size);
	glVertex3f(x - size, y - size, z - size);
	glVertex3f(x - size, y + size, z - size);
	glVertex3f(x + size, y + size, z - size);

	// Left face (x = -1.0f)
	glColor3f(0.0f, 0.0f, 1.0f); // Blue

	glVertex3f(x - size, y + size, z + size);
	glVertex3f(x - size, y + size, z - size);
	glVertex3f(x - size, y - size, z - size);
	glVertex3f(x - size, y - size, z + size);

	// Right face (x = 1.0f)
	glColor3f(1.0f, 0.0f, 1.0f); // Magenta

	glVertex3f(x + size, y + size, z - size);
	glVertex3f(x + size, y + size, z + size);
	glVertex3f(x + size, y - size, z + size);
	glVertex3f(x + size, y - size, z - size);
	//glEnd();  // End of drawing color-cube
}

static void drawTest(float x, float y, float z)
{
	glVertex3f(x, y, z);
}
OccupancyGridMapping3D::OccupancyGridMapping3D()
{
	setPathKey(Qt::Key_0, 0);
	setPathKey(Qt::Key_1, 1);
	setPathKey(Qt::Key_2, 2);
	setPathKey(Qt::Key_3, 3);
	setPathKey(Qt::Key_4, 4);
	setPathKey(Qt::Key_5, 5);
	setPathKey(Qt::Key_6, 6);
	setPathKey(Qt::Key_7, 7);
	setPathKey(Qt::Key_8, 8);
	setPathKey(Qt::Key_9, 9);
	iterationCounter = 0;
}

void OccupancyGridMapping3D::drawMap()
{

	//glClear ( GL_COLOR_BUFFER_BIT ) ;
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	//glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
	glBegin(GL_QUADS);

	vector<Point>::iterator it;

	sort(glQuads.begin(), glQuads.end(), my_compare);
	int index = 1;
	for (it = glQuads.begin(); it != glQuads.end(); it++)
	{
		drawQuad((*it).x, (*it).y, (*it).z);
		if (showPosition)
		{
			printf("Voxel %d x = %f, y = %f,, z = %f \n", index, (*it).x, (*it).y, (*it).z);
		}
		index++;
	}

	glEnd();
}

// Draws a spiral
void OccupancyGridMapping3D::drawSpiral()
{
	const float nbSteps = 1000.0;

	glBegin(GL_QUAD_STRIP);
	for (int i = 0; i < nbSteps; ++i)
	{
		const float ratio = i / nbSteps;
		const float angle = 21.0 * ratio;
		const float c = cos(angle);
		const float s = sin(angle);
		const float r1 = 1.0 - 0.8f * ratio;
		const float r2 = 0.8f - 0.8f * ratio;
		const float alt = ratio - 0.5f;
		const float nor = 0.5f;
		const float up = sqrt(1.0 - nor * nor);
		glColor3f(1.0 - ratio, 0.2f, ratio);
		glNormal3f(nor * c, up, nor * s);
		glVertex3f(r1 * c, alt, r1 * s);
		glVertex3f(r2 * c, alt + 0.05f, r2 * s);
	}
	glEnd();
}

void OccupancyGridMapping3D::init()
{
	restoreStateFromFile();
	setSceneRadius(5);
	showEntireScene();
	setAxisIsDrawn();
	glDisable(GL_LIGHTING);
	maxDistanceVoxel = sqrt(pow(voxelSize, 2) * 3);
}

void OccupancyGridMapping3D::draw()
{
	// Place light at camera position
	const Vec cameraPos = camera()->position();
	const GLfloat pos[4] = {(float)cameraPos[0], (float)cameraPos[1],
							(float)cameraPos[2], 1.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, pos);

	// Orientate light along view direction
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, camera()->viewDirection());

	meddleMutex.lock();

	if (drawVoxel)
	{
		drawMap();
	}

	if (drawPixel)
	{
		deque<Point> points = this->pointCloud->getSortedPoints();
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_POINTS);

		std::deque<Point>::iterator it;

		for (it = points.begin(); it != points.end(); it++)
		{
			drawTest((*it).x, (*it).y, (*it).z);
		}
		glEnd();
	}

	/**
	if(true)
	{
		glLineWidth(0.02); 
		
		glColor3f(1.0, 1.0, 1.0);
		
		glBegin(GL_LINES);
		
		std::vector<Line>::iterator it;
		
			for(it = lines.begin(); it != lines.end(); it++ )
				{
								
					glVertex3f((*it).x1, (*it).y1, (*it).z1);
					glVertex3f((*it).x2, (*it).y2, (*it).z2);
					
				}

		glEnd();
	}
	**/

	if (showGrid)
	{
		drawGrid();
	}

	meddleMutex.unlock();
}

void OccupancyGridMapping3D::setPoint(PointCloud *pointCloud)
{
	this->pointCloud = pointCloud;
}

void OccupancyGridMapping3D::process()
{
	//meddleMutex.lock();

	ROS_INFO("edgeVoxel %f", voxelSize);
	ROS_INFO("numPoints %d", numPoints);
	ROS_INFO("method %d", method);

	switch (method)
	{
	case 1:
		method1();
		break;
	case 2:
		method2();
		break;
	case 3:
		method3();
		break;
	case 4:
		method4();
		break;
	case 5:
		method5();
		break;
	case 6:
		method6();
		break;
	case 7:
		method7();
		break;
	case 8:
		method8();
		break;
	case 9:
		method9();
		break;
	}
	if(showIterationCount){
		ROS_INFO("iteration %d", iterationCounter);
	}
	
}

void OccupancyGridMapping3D::method1()
{
	ROS_INFO("Metodo 1");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x - voxelSize / 2;
			limitMaxX = base.x + voxelSize / 2;

			limitMinY = base.y - voxelSize / 2;
			limitMaxY = base.y + voxelSize / 2;

			limitMinZ = base.z - voxelSize / 2;
			limitMaxZ = base.z + voxelSize / 2;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{
				Point quad;
				quad.x = recalculateValue(base.x);
				quad.y = recalculateValue(base.y);
				quad.z = recalculateValue(base.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

void OccupancyGridMapping3D::method2()
{

	ROS_INFO("Metodo 2");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x - voxelSize / 2;
			limitMaxX = base.x + voxelSize / 2;

			limitMinY = base.y - voxelSize / 2;
			limitMaxY = base.y + voxelSize / 2;

			limitMinZ = base.z - voxelSize / 2;
			limitMaxZ = base.z + voxelSize / 2;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{

				Point tempPoint = diference(vectorVoxel);

				Point quad;
				quad.x = recalculateValue(tempPoint.x);
				quad.y = recalculateValue(tempPoint.y);
				quad.z = recalculateValue(tempPoint.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}
void OccupancyGridMapping3D::method3()
{
	ROS_INFO("Metodo 3");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x - voxelSize / 2;
			limitMaxX = base.x + voxelSize / 2;

			limitMinY = base.y - voxelSize / 2;
			limitMaxY = base.y + voxelSize / 2;

			limitMinZ = base.z - voxelSize / 2;
			limitMaxZ = base.z + voxelSize / 2;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{

				Point tempPoint = voxelCenter(vectorVoxel);

				Point quad;
				quad.x = recalculateValue(tempPoint.x);
				quad.y = recalculateValue(tempPoint.y);
				quad.z = recalculateValue(tempPoint.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

void OccupancyGridMapping3D::method4()
{

	ROS_INFO("Metodo 4");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x;
			limitMaxX = base.x + maxDistanceVoxel;

			limitMinY = base.y;
			limitMaxY = base.y + maxDistanceVoxel;

			limitMinZ = base.z;
			limitMaxZ = base.z + maxDistanceVoxel;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{

				Point quad;
				quad.x = recalculateValue(base.x);
				quad.y = recalculateValue(base.y);
				quad.z = recalculateValue(base.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

void OccupancyGridMapping3D::method5()
{

	ROS_INFO("Metodo 5");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x;
			limitMaxX = base.x + maxDistanceVoxel;

			limitMinY = base.y;
			limitMaxY = base.y + maxDistanceVoxel;

			limitMinZ = base.z;
			limitMaxZ = base.z + maxDistanceVoxel;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{
				Point tempPoint = diference(vectorVoxel);
				Point quad;
				quad.x = recalculateValue(tempPoint.x);
				quad.y = recalculateValue(tempPoint.y);
				quad.z = recalculateValue(tempPoint.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

void OccupancyGridMapping3D::method6()
{
	ROS_INFO("Metodo 6");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x;
			limitMaxX = base.x + maxDistanceVoxel;

			limitMinY = base.y;
			limitMaxY = base.y + maxDistanceVoxel;

			limitMinZ = base.z;
			limitMaxZ = base.z + maxDistanceVoxel;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{
				Point tempPoint = voxelCenter(vectorVoxel);
				Point quad;
				quad.x = recalculateValue(tempPoint.x);
				quad.y = recalculateValue(tempPoint.y);
				quad.z = recalculateValue(tempPoint.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

void OccupancyGridMapping3D::method7()
{

	ROS_INFO("Metodo 7");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x - maxDistanceVoxel / 2;
			limitMaxX = base.x + maxDistanceVoxel / 2;

			limitMinY = base.y - maxDistanceVoxel / 2;
			limitMaxY = base.y + maxDistanceVoxel / 2;

			limitMinZ = base.z - maxDistanceVoxel / 2;
			limitMaxZ = base.z + maxDistanceVoxel / 2;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{

				Point quad;
				quad.x = recalculateValue(base.x);
				quad.y = recalculateValue(base.y);
				quad.z = recalculateValue(base.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

void OccupancyGridMapping3D::method8()
{
	ROS_INFO("Metodo 8");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x - maxDistanceVoxel / 2;
			limitMaxX = base.x + maxDistanceVoxel / 2;

			limitMinY = base.y - maxDistanceVoxel / 2;
			limitMaxY = base.y + maxDistanceVoxel / 2;

			limitMinZ = base.z - maxDistanceVoxel / 2;
			limitMaxZ = base.z + maxDistanceVoxel / 2;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{
				Point tempPoint = diference(vectorVoxel);
				Point quad;
				quad.x = recalculateValue(tempPoint.x);
				quad.y = recalculateValue(tempPoint.y);
				quad.z = recalculateValue(tempPoint.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

void OccupancyGridMapping3D::method9()
{
	ROS_INFO("Metodo 9");
	glQuads.clear();
	deque<Point> pointsCloud = this->pointCloud->getSortedPoints();
	//cout << "Tamaño " << pointsCloud.size()<<endl;
	if (!pointsCloud.empty())
	{
		sort(pointsCloud.begin(), pointsCloud.end(), my_compare);
		Point base;
		Point target;
		Point tempPoint;
		double distance = 0.0;
		double limitMinX = 0.0;
		double limitMaxX = 0.0;
		double limitMinY = 0.0;
		double limitMaxY = 0.0;
		double limitMinZ = 0.0;
		double limitMaxZ = 0.0;
		int index = 0;

		while (!pointsCloud.empty())
		{
			vectorVoxel.clear();
			base = pointsCloud.front();
			pointsCloud.pop_front();
			vectorVoxel.push_back(base);
			index = 0;

			limitMinX = base.x - maxDistanceVoxel / 2;
			limitMaxX = base.x + maxDistanceVoxel / 2;

			limitMinY = base.y - maxDistanceVoxel / 2;
			limitMaxY = base.y + maxDistanceVoxel / 2;

			limitMinZ = base.z - maxDistanceVoxel / 2;
			limitMaxZ = base.z + maxDistanceVoxel / 2;

			do
			{

				if (!pointsCloud.empty())
				{
					tempPoint = pointsCloud.at(index);
					iterationCounter++;
				}
				else
				{
					break;
				}
				if ((limitMinX < tempPoint.x && tempPoint.x < limitMaxX) &&
					(limitMinY < tempPoint.y && tempPoint.y < limitMaxY) &&
					(limitMinZ < tempPoint.z && tempPoint.z < limitMaxZ))
				{
					Point pointVoxel;
					pointVoxel.x = tempPoint.x;
					pointVoxel.y = tempPoint.y;
					pointVoxel.z = tempPoint.z;

					vectorVoxel.push_back(pointVoxel);

					pointsCloud.erase(pointsCloud.begin() + index);

					if (vectorVoxel.size() >= numPoints)
					{
						break;
					}
				}
				index++;
			} while (index < pointsCloud.size() && tempPoint.x < limitMaxX);

			if (vectorVoxel.size() >= numPoints)
			{
				Point tempPoint = voxelCenter(vectorVoxel);
				Point quad;
				quad.x = recalculateValue(tempPoint.x);
				quad.y = recalculateValue(tempPoint.y);
				quad.z = recalculateValue(tempPoint.z);

				this->glQuads.push_back(quad);
			}
		}

		int voxels = glQuads.size();
		ROS_INFO("Voxels %d", voxels);
	}
}

double OccupancyGridMapping3D::recalculateValue(double value)
{

	double size = value / voxelSize;

	int roundedValue = round(size);

	double newValue = voxelSize * roundedValue;

	double minVal = newValue - (voxelSize / 2);
	double maxVal = newValue + (voxelSize / 2);

	double diferenceMinValue = abs(minVal - value);

	double diferenceMaxValue = abs(maxVal - value);

	if (diferenceMinValue < diferenceMaxValue)
	{
		return minVal;
	}
	else
	{
		return maxVal;
	}
}

Point OccupancyGridMapping3D::voxelCenter(std::vector<Point> vectorVoxel)
{

	vector<Point>::iterator it;
	int n = vectorVoxel.size();
	double totalX = 0;
	double totalY = 0;
	double totalZ = 0;

	Point point;

	for (it = vectorVoxel.begin(); it != vectorVoxel.end(); ++it)
	{
		totalX += (*it).x;
		totalY += (*it).y;
		totalZ += (*it).z;
	}

	point.x = totalX / n;
	point.y = totalY / n;
	point.z = totalZ / n;

	return point;
}

Point OccupancyGridMapping3D::diference(std::vector<Point> vectorVoxel)
{
	Point voxel;
	Point tempA;
	Point tempB;

	sort(vectorVoxel.begin(), vectorVoxel.end(), my_compareX);
	tempA = vectorVoxel.back();
	tempB = vectorVoxel.front();

	voxel.x = tempA.x + ((tempA.x - tempB.x) / 2);

	sort(vectorVoxel.begin(), vectorVoxel.end(), my_compareY);

	tempA = vectorVoxel.back();
	tempB = vectorVoxel.front();

	voxel.y = tempA.y + ((tempA.y - tempB.y) / 2);

	sort(vectorVoxel.begin(), vectorVoxel.end(), my_compareZ);

	tempA = vectorVoxel.back();
	tempB = vectorVoxel.front();

	voxel.z = tempA.z + ((tempA.z - tempB.z) / 2);

	ROS_INFO_STREAM("diference x: " << voxel.x << " y " << voxel.y << " z " << voxel.z);

	return voxel;
}

void OccupancyGridMapping3D::keyPressEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_P:
		drawPixel = drawPixel ? false : true;
		break;
	case Qt::Key_C:
		drawVoxel = drawVoxel ? false : true;
		break;
	case Qt::Key_G:
		ROS_INFO("Generando Entorno Virtaul");
		VirtualEnvironment virtualEnvironment;
		virtualEnvironment.generate(this->glQuads);
		break;
	case Qt::Key_Y:
		showGrid = showGrid ? false : true;
		break;
	}
}

QString OccupancyGridMapping3D::helpString() const
{
	return QString("");
}

void OccupancyGridMapping3D::drawGrid()
{

	glLineWidth(2.5);
	glColor3f(66, 134, 244);
	glBegin(GL_LINES);

	float x;
	float y;
	float z;

	float limitX = 1;
	float limitY = 1;
	float limitZ = voxelSize * 3;

	for (z = 0; z <= 1; z += voxelSize)
	{
		for (x = 0; x <= 1; x += voxelSize)
		{
			glVertex3f(x, limitY, z);
			glVertex3f(x, 0, z);
		}

		glVertex3f(limitX, limitY, z);
		glVertex3f(limitX, 0, z);

		for (y = 0; y <= 1; y += voxelSize)
		{
			glVertex3f(limitX, y, z);
			glVertex3f(0, y, z);
		}

		glVertex3f(limitX, limitY, z);
		glVertex3f(0, limitX, z);
	}

	for (x = 0; x <= limitX + voxelSize; x += voxelSize)
	{
		for (y = 0; y <= limitY + voxelSize; y += voxelSize)
		{
			glVertex3f(x, y, voxelSize * 3);
			glVertex3f(x, y, 0);
		}
	}

	glEnd();
}