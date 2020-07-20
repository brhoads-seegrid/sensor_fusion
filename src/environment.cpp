/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "quiz/ransac/ransac3d.h"           // Ransac3D
#include "quiz/cluster/cluster.h"           // Cluster3D
#include <pcl/io/pcd_io.h>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <vector>
#include <limits>
#include <random>       // rand
#include <time.h>
#include <utility>      // pair (through auto)
using namespace std;
    // TO DO: add missing boost & other headers.


// Initialize camera for viewer:
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);


// Class for storing things reused from frame to frame:

class Highway {

public:

    Highway ( pcl::visualization::PCLVisualizer::Ptr &viewer,
              const string &streamRoot )
        : m_Viewer ( viewer )
        , m_Ppc()
        , m_StreamPaths ( m_Ppc.streamPcd ( streamRoot ) )
    {
    }

    void SetClusterTol ( float f ) { m_ClusterTol = f; }
    void SetVoxelSize  ( float f ) { m_VoxelSize  = f; }
    void SetDxRoi      ( float f ) { m_DxRoi      = f; }

    Color color() { return Color ( rand() % 2, rand() % 2, rand() % 2 ); }
        // TO DO: mk pvt or static.
        // TO DO: explicit cast to float.

    void cityBlock ( size_t iFrame )    // m_Ppc cannot be const, for one.
    {
        auto pcRaw = m_Ppc.loadPcd ( m_StreamPaths[iFrame].string() );

        auto pcFiltered = m_Ppc.FilterCloud ( pcRaw, m_VoxelSize,
            Eigen::Vector4f ( -m_DxRoi, -m_DxRoi/2.0f,     -2.0f, 1.0f),
            Eigen::Vector4f (  m_DxRoi,  m_DxRoi/2.0f + 2.0f,  2.0f, 1.0f));
                // y roi assymmetric because we drive on the right.
                // z roi filter mainly for phantom points beneath road.

        //auto pcInAndOut = m_Ppc.SegmentPlane ( pcFiltered, 100, 0.2f );
        auto pcInAndOut = Ransac3D             ( pcFiltered, 100, 0.2f );

        //vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters
        //    = m_Ppc.Clustering ( pcInAndOut.second, m_ClusterTol, 5,
        //        numeric_limits<int>::max() );   // no cluster size upper bound.
        vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters
            = Cluster3d ( pcInAndOut.second, m_ClusterTol );

        int name = 1;   // Not worth coming up with meaningful unique strings.
        for ( auto c : clusters ) {
            Box box = m_Ppc.BoundingBox ( c );
            renderBox( m_Viewer, box, name++ );
            renderPointCloud ( m_Viewer, c, to_string ( name++ ), color() );
        }

        renderPointCloud ( m_Viewer, pcInAndOut.first, "road",
            Color ( 0.0f, 1.0f, 0.0f ) );      // Green like in example gif.
    }

private:

    pcl::visualization::PCLVisualizer::Ptr &m_Viewer;
    ProcessPointClouds<pcl::PointXYZI> m_Ppc;
    vector<boost::filesystem::path> m_StreamPaths;
            // TO DO: mk const?
    float m_ClusterTol  =    0.3f;
    float m_VoxelSize   =    0.1f;
    float m_DxRoi       =    8.0f;
};


int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (
        new pcl::visualization::PCLVisualizer ( "FPS" ) );
    initCamera ( FPS, viewer );
    Highway highway ( viewer, "../src/sensors/data/pcd/data_1" );
        // TO DO: boost::program_options or move inside.
        // ASSUMES BEING CALLED FROM INSIDE BUILD DIRECTORY!

    size_t delay = 300000;
    if (argc >= 2) highway.SetClusterTol    ( atof ( argv[1] ) );
    if (argc >= 3) highway.SetVoxelSize     ( atof ( argv[2] ) );
    if (argc >= 4) highway.SetDxRoi         ( atof ( argv[3] ) );
    if (argc >= 5) delay =                  ( atof ( argv[4] ) );
        // TO DO: boost::program_options.

    size_t iWhile = 0;          // ok if it overflows.
    size_t iFrame = 0;
    bool wasStopped = false;
    while ( !viewer->wasStopped() ) {
        if ( iWhile++ % delay == 0 ) {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            highway.cityBlock( iFrame );
            iFrame = ( iFrame + 1 ) % 22;   // Known file name limit.
        }
        viewer->spinOnce ();
    }
}


// TO DO: reformat this to match the above class and main.
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    int distance = 16;  // m
    
    switch(setAngle)
    {
        case XY       : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown  : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side     : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS      : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

