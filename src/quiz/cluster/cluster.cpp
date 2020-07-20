/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "cluster.h"
#include "../../render/render.h"
#include "../../render/box.h"
#include "../../processPointClouds.h"
#include "../../processPointClouds.cpp"
#include <chrono>
#include <string>
#include "kdtree.h"

int cluster_3d ()
{
    string path = "../../../src/sensors/data/pcd/data_1/0000000000.pcd";
    ProcessPointClouds<pcl::PointXYZI> ppc;
    auto pcIn = ppc.loadPcd ( path );

    std::vector <
        pcl::PointCloud<pcl::PointXYZI>::Ptr >
        clusters =
        Cluster3d ( pcIn, 0.1f );

    return 0;
}

std::vector <
    pcl::PointCloud<pcl::PointXYZI>::Ptr >
    Cluster3d ( const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn,
        float tol ) {

    vector<vector<float>> ptsIn;
    for ( auto p : pcIn->points ) {
        ptsIn.push_back ( { p.x, p.y, p.z } );
    }
    
    KdTree kdTree;
    //for ( auto pt : pcIn->points ) {
    for ( int i = 0; i < pcIn->points.size(); ++i ) {
        auto pt = pcIn->points[i];
        kdTree.insert ( {pt.x, pt.y, pt.z} , i );
    }

    vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > pcClusters;
    vector<vector<int>> clusters = euclideanCluster ( ptsIn, &kdTree, tol );
    for ( auto cluster : clusters ) {
        pcClusters.emplace_back ( new pcl::PointCloud<pcl::PointXYZI>() );
        for ( auto i : cluster ) {
            pcClusters.back()->push_back ( 
                        pcIn->points[i] );
        }
    }

    return pcClusters;
}

int cluster_2d ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	vector<vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//vector<vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	//auto clusters = KdTreeClusters(cloud);

	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);
  
  	cout << "Test Search" << endl;
  	vector<int> nearby = tree->search({-6,7},3.0);
  	for(int index : nearby)
      cout << index << ",";
  	cout << endl;

  	// Time segmentation process
  	auto startTime = chrono::steady_clock::now();
  	//
  	vector<vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = chrono::steady_clock::now();
  	auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
  	cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << endl;

  	// Render clusters
  	int clusterId = 0;
	vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}


pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	//viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(vector<vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

void clusterHelper(int i, const vector<vector<float>> &points, vector<int> &cluster, vector<bool> &processed, KdTree* tree, float distanceTol)
{
	processed[i] = true;
	cluster.push_back(i);
	auto nearby = tree->search(points[i], distanceTol);
	for (auto j : nearby) {
		if (!processed[j]) {
			clusterHelper(j, points, cluster, processed, tree, distanceTol);
		}
	}
}

vector<vector<int>> euclideanCluster(const vector<vector<float>>& points, KdTree* tree, float distanceTol)
{
	vector<vector<int>> clusters;
	size_t n = points.size();
	vector<bool> processed(n, false);

	for (int i = 0; i < n; ++i) {
		if (!processed[i]) {
			vector<int> cluster;
			clusterHelper(i, points, cluster, processed, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}


