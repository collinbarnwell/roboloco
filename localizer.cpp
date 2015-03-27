#include "localize.hpp"
#include "my_viz.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

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

    vector<Particle> belief; // create random belief
    CGRLocalize(belief, PlanePoints, my_normals);

    // visualize2(PlanePoints);
}

void cmdCallback(const geometry_msgs::Twist& vel_cmd) {
    // ROS_INFO("I heard: [%f]", vel_cmd.linear.y);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/right_cloud_transform", 1, pointcloudCallback);

    ros::NodeHandle nh2;
    ros::Subscriber sub2 = nh2.subscribe("/cmd_vel", 1, cmdCallback)

    cout << "inside of node" << endl;
    
    ros::spin();
    return 0;
}
