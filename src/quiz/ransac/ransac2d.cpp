/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "ransac3d.h"	// Ransac3D
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZI> pointProcessor;
	return pointProcessor.loadPcd("../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

#include <assert.h>

using namespace std;

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	while (maxIterations--) {
		unordered_set<int> inliers;
		int i1 = rand() % cloud->points.size();
		int i2 = rand() % cloud->points.size();
		if (i1 == i2) continue;
		inliers.insert(i1);
		inliers.insert(i2);
		auto pt1 = cloud->points[i1];
		auto pt2 = cloud->points[i2];
		float a = pt1.y - pt2.y;
		float b = pt2.x - pt1.x;
		float c = pt1.x * pt2.y - pt2.x * pt1.y;
		float s = sqrt(a*a + b*b);
	 	if (s == 0.0f) continue;
		for (int i = 0; i < cloud->points.size(); ++i) {
			if (inliers.count(i) > 0) continue;
			pcl::PointXYZ pt = cloud->points[i];
			float d = fabs(a * pt.x + b * pt.y + c) / s;
			if (d <= distanceTol) inliers.insert(i);
		}
		if (inliers.size() > inliersResult.size()) inliersResult = inliers;
	}
	
	return inliersResult;
}


/*
TO DO: restore 2d test so file name makes sense and remove 3d
or put it in some common area to be used for environment.cpp project;
for now copying it there.
*/

int main ()
{
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData3D();

	auto inAndOut = Ransac3D ( cloud, 100, 0.2f );

	renderPointCloud(viewer,inAndOut.first,"inliers",Color(1,0,0));
    renderPointCloud(viewer,inAndOut.second,"outliers",Color(1,1,1));
    while (!viewer->wasStopped ()) viewer->spinOnce ();
}
