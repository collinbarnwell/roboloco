#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/pcl_base.h"
#include "pcl/PointIndices.h"
#include "pcl/conversions.h"
#include "pcl/common/impl/io.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
 #include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

double dst(pcl::PointXYZ a, pcl::PointXYZ b) {
    double m = pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2);
    return sqrt(m);
}

PointXYZ crossp(PointXYZ A, PointXYZ B) {
  float x = A.y*B.z - (B.y*A.z); 
  float y = B.x*A.z - (A.x*B.z); 
  float z = A.x*B.y - (A.y*B.x);
  return PointXYZ(x, y, z);
}

float dotp(PointXYZ a, PointXYZ b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Normal normalize(PointXYZ n) {
    float l = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
    return Normal(n.x/l, n.y/l, n.z/l);
}

Normal planeNorm(PointXYZ a, PointXYZ b, PointXYZ c) {
    PointXYZ v1(a.x - b.x, a.y - b.y, a.z - b.z);
    PointXYZ v2(a.x - c.x, a.y - c.y, a.z - c.z);
    return normalize(crossp(v1, v2));
}

float magnitude(PointXYZ n) {
    return sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
}

float dotpn(PointXYZ a, Normal b) {
    return a.x * b.normal_x + a.y * b.normal_y + a.z * b.normal_z;
}

float planeToPtDist(PointXYZ currpoint, PointXYZ a, Normal norm) {
    PointXYZ planeToPoint = PointXYZ(a.x - currpoint.x, a.y - currpoint.y, a.z - currpoint.z);
    return abs(dotpn(planeToPoint, norm));
}

void visualize(PointCloud<PointXYZ> cloud) {
    visualization::CloudViewer viewer ("Simple Cloud Viewer");
    PointCloud<PointXYZ>::Ptr cloudptr(&cloud);
    viewer.showCloud (cloudptr);
    while (!viewer.wasStopped ())
    {
    }
}