
#include "ransac3d.h"
#include <pcl/io/pcd_io.h>      // pcl, PointXYZ, PointCloud, Ptr
#include <unordered_set>
#include <random>       // rand, srand
#include <time.h>
#include <assert.h>
#include <utility>      // pair
using namespace std;
    // TO DO: add missing boost & other headers.


// Helpers for Ransac below. Not sure why PCL doesn't provide them.
// Not worth const refs.
// TO DO: something with intensity.



pcl::PointXYZI operator- ( pcl::PointXYZI p1, pcl::PointXYZI p2 )
{
    //return pcl::PointXYZI ( p1.x - p2.x, p1.y - p2.y, p1.z - p2.z );
        // Apparently this ctor is pretty new (1.11) and Udacity is using an
        // older version of PCL. Sorry if that was mentioned somewhere,
        // but I didn't notice any version information.
    pcl::PointXYZI p3;
    p3.x = p1.x - p2.x;
    p3.y = p1.y - p2.y;
    p3.z = p1.z - p2.z;
        // Default value of intensity okay.
    return p3;
}

pcl::PointXYZI crossProduct ( pcl::PointXYZI v1, pcl::PointXYZI v2 )
{
    /*
    return pcl::PointXYZI (
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x );
        
        // Apparently this ctor is pretty new (1.11) and Udacity is using an
        // older version of PCL. Sorry if that was mentioned somewhere,
        // but I didn't notice any version information.
    */
    pcl::PointXYZI v3;
    v3.x = v1.y * v2.z - v1.z * v2.y;
    v3.y = v1.z * v2.x - v1.x * v2.z;
    v3.z = v1.x * v2.y - v1.y * v2.x );
        // Default value of intensity okay.
    return v3;
}


pair < pcl::PointCloud<pcl::PointXYZI>::Ptr,
       pcl::PointCloud<pcl::PointXYZI>::Ptr >
Ransac3D ( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    int maxIterations, float distanceTol )
{
    unordered_set<int> inliersResult;
    
    srand ( time ( NULL ) );

    while ( maxIterations-- ) {

        int i1 = rand() % cloud->points.size();
        int i2 = rand() % cloud->points.size();
        int i3 = rand() % cloud->points.size();
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;
        unordered_set<int> inliers {i1, i2, i3};
        auto p1 = cloud->points[i1];
        auto p2 = cloud->points[i2];
        auto p3 = cloud->points[i3];
        auto v1 = p2 - p1;
        auto v2 = p3 - p1;

        auto v3 = crossProduct (v1, v2);
        float a = v3.x, b = v3.y, c = v3.z, d = -(a*p1.x + b*p1.y + c*p1.z);
        
        float s = sqrt(a*a + b*b + c*c);
        if (s == 0.0f) continue;
        for (int i = 0; i < cloud->points.size(); ++i) {
            if (inliers.count(i) > 0) continue;
            pcl::PointXYZI pt = cloud->points[i];
            float distance = fabs(a * pt.x + b * pt.y + c * pt.z + d) / s;
            if (distance <= distanceTol) inliers.insert(i);
        }
        if (inliers.size() > inliersResult.size()) inliersResult = inliers;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers (
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers (
        new pcl::PointCloud<pcl::PointXYZI>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZI point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    return pair <
        pcl::PointCloud<pcl::PointXYZI>::Ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr
    > ( cloudInliers, cloudOutliers );
}


