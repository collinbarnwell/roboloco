#include "localize.hpp"
#include "my_viz.hpp"

using namespace std;
using namespace pcl;


pcl::PointCloud<pcl::PointXYZ> my_cloud_global;
// PointCloud<PointXYZ>::Ptr my_cloud_ptr(&my_cloud_global);


void pointcloudCallback(const sensor_msgs::PointCloud2 msg) { 
    cout << "inside callback" << endl;

    pcl::fromROSMsg(msg, my_cloud_global);

    PointCloud<PointXYZ> PlanePoints;
    unordered_map<PointXYZ, Normal> my_normals;
    PlanePoints = fspf(my_cloud_global, my_normals);

    // visualize2(PlanePoints);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/right_cloud_transform", 1, pointcloudCallback);

    cout << "inside of node" << endl;
    
    ros::spin();
    return 0;
}
