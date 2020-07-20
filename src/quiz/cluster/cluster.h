#ifndef CLUSTER_DEFINED
#define CLUSTER_DEFINED


/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

int cluster_3d();
int cluster_2d();

std::vector <
    pcl::PointCloud<pcl::PointXYZI>::Ptr >
    Cluster3d ( const pcl::PointCloud<pcl::PointXYZI>::Ptr &,
        float tol );

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(vector<vector<float>> points);

void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0);

void clusterHelper(int i, const vector<vector<float>> &points, vector<int> &cluster, vector<bool> &processed, KdTree* tree, float distanceTol);

vector<vector<int>> euclideanCluster(const vector<vector<float>>& points, KdTree* tree, float distanceTol);

#endif

