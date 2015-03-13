#ifndef MY_VIZ
#define MY_VIZ

#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

void visualize1(PointCloud<PointXYZ> cloud) {
    visualization::CloudViewer viewer ("Simple Cloud Viewer 1");
    PointCloud<PointXYZ>::Ptr cloudptr(&cloud);
    viewer.showCloud (cloudptr);
    while (!viewer.wasStopped ()) {
    }
}

void visualize2(PointCloud<PointXYZ> cloud) {
    visualization::CloudViewer viewer ("Simple Cloud Viewer 2");
    PointCloud<PointXYZ>::Ptr cloudptr(&cloud);
    viewer.showCloud (cloudptr);
    while (!viewer.wasStopped ()) {
    }
}

#endif