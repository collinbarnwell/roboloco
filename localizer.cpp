#include "raycast.hpp"
#include "fspf.hpp"
#include "my_viz.hpp"

using namespace std;
using namespace pcl;


pcl::PointCloud<pcl::PointXYZ> my_cloud;
PointCloud<PointXYZ>::Ptr my_cloud_ptr(&my_cloud);


void pointcloudCallback(const sensor_msgs::PointCloud2 msg) { 
    cout << "inside callback" << endl;

    pcl::fromROSMsg(msg, my_cloud);

    PointCloud<PointXYZ> PlanePoints;
    PlanePoints = fspf(my_cloud);

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
