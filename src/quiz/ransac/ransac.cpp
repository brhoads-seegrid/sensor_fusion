
#include "ransac3d.h"    // Ransac3D
#include "../../render/render.h"
#include "../../processPointClouds.h"
#include "../../processPointClouds.cpp"
#include <unordered_set>
#include <assert.h>
using namespace std;

pcl::PointCloud<pcl::PointXYZ  >::Ptr CreateData();
pcl::PointCloud<pcl::PointXYZI >::Ptr CreateData3D();
pcl::visualization::PCLVisualizer::Ptr initScene();
int driver1_visualize_random_objects();
int driver2_visualize_pcd();
int visualize_in_and_out ( const pair <
    pcl::PointCloud<pcl::PointXYZI>::Ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr > &);
int unit1_visualize_minimal_pointcloud();

int main () {
    //unit1_visualize_minimal_pointcloud();
    //driver1_visualize_random_objects();
    driver2_visualize_pcd();
    return 0;
}

int unit1_visualize_minimal_pointcloud() {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (
        new pcl::PointCloud<pcl::PointXYZI> );
    cloud->points.emplace_back ( 0, 0, 0, 0 );
    cloud->points.emplace_back ( 1, 0, 0, 0 );
    cloud->points.emplace_back ( 0, 1, 0, 0 );
    cloud->points.emplace_back ( 0, 0, 1, 0 );

    auto inAndOut = Ransac3D ( cloud, 1, 0.000001f );

    auto in = inAndOut.first, out = inAndOut.second;
    for ( auto pt : in->points ) {
        cout << "in: " << pt << endl;
    }
    for ( auto pt : out->points ) {
        cout << "out: " << pt << endl;
    }

    assert (  in->points.size() == 3 );
    assert ( out->points.size() == 1 );

    visualize_in_and_out ( inAndOut );
}

int driver1_visualize_random_objects() {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData3D();
    auto inAndOut = Ransac3D ( cloud, 100, 0.2f );
}

int driver2_visualize_pcd() {

    string path = "../../../src/sensors/data/pcd/data_1/0000000000.pcd";
    ProcessPointClouds<pcl::PointXYZI> ppc;
    auto pc = ppc.loadPcd ( path );
    auto inAndOut = Ransac3D ( pc, 100, 0.1f );
    visualize_in_and_out ( inAndOut );
}

int visualize_in_and_out ( const pair <
    pcl::PointCloud<pcl::PointXYZI>::Ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr > & inAndOut ) {

    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
    renderPointCloud(viewer,inAndOut.first,"inliers",Color(1,0,0));
    renderPointCloud(viewer,inAndOut.second,"outliers",Color(1,1,1));
    while (!viewer->wasStopped ()) viewer->spinOnce ();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (
        new pcl::PointCloud<pcl::PointXYZ>() );

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

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D() {

    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    return pointProcessor.loadPcd("../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

