#ifndef RANSAC3D_DEFINED
#define RANSAC3D_DEFINED


#include <pcl/io/pcd_io.h>      // pcl, PointXYZ, PointCloud, Ptr
#include <utility>              // pair


// Compute indexes for custom Ransac:
std::pair < pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr >
Ransac3D ( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    int maxIterations, float distanceTol );



#endif
