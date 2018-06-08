#include <QGLViewer/qglviewer.h>
#include "settings.h"
#include "PointCloud.h"
#include "boost/thread.hpp"
#include "qevent.h"
#include <vector>
#include <queue>

struct Line{
	
	float x1;
	float x2;
	float y1;
	float y2;
	float z1;
	float z2;
	
};


class OccupancyGridMapping3D : public QGLViewer {

public:
	OccupancyGridMapping3D();
	void setPoint(PointCloud* pointCloud);
	void process();
	
protected:
	virtual void init();
	virtual void draw();
	virtual void keyPressEvent(QKeyEvent *e);
	virtual QString helpString() const;

private:
	std::vector<Point> glQuads;
	std::queue<Point>  pilaPoinCloud;
	boost::mutex meddleMutex;
	PointCloud* pointCloud;
	float maxDistanceVoxel;
	void drawMap();
	void drawGrid();
	virtual void drawSpiral();
	std::vector<Line> lines;
	std::vector<Point> vectorVoxel;
	std::vector<Point>::iterator it;
	Point voxelCenter(std::vector<Point> vectorVoxel);
	void method1();
	void method2();
	void method3();
	void method4();
	void method5();
	void method6();
	void method7();
	void method8();
	void method9();
	double recalculateValue(double value);
	Point diference(std::vector<Point> vectorVoxel);

	int iterationCounter;

	
};
