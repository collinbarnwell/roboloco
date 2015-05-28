#include "localize.hpp"
#include "my_viz.hpp"
#include "labmap.hpp"
#include "fspf.hpp"

#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace pcl;

ros::Subscriber sub, sub2;

class Everything 
{
    private:
        MovementKeeper myMoves;
        vector<Particle> belief;
    public:
        void pointcloudCallback(const sensor_msgs::PointCloud2 msg);
        void cmdCallback(const geometry_msgs::Twist& vel_cmd);
};

void Everything::pointcloudCallback(const sensor_msgs::PointCloud2 msg) 
{   
    cout << "Pointcloud callback running..." << endl;

    ros::Time::init();
    myMoves.initTime();

    // PointCloud<PointXYZ> receiver_cloud;

    // PointCloud<PointXYZ>::Ptr my_cloud_ptr(new PointCloud<PointXYZ>, null_deleter());

    PointCloud<PointXYZ> my_cloud;
    // my_cloud = *my_cloud_ptr;

    pcl::fromROSMsg(msg, my_cloud);

    PointCloud<PointXYZ>::Ptr my_cloud_ptr (new PointCloud<PointXYZ> (my_cloud)); 

    PointCloud<PointXYZ> PlanePoints;
    PointCloud<PointXY> PlaneNorms;

    cout << "before fspf" << endl;

    fspf(my_cloud, my_cloud_ptr, PlanePoints, PlaneNorms);

    cout << "finised fspf" << endl;

    int beliefsize = belief.size();
    if (beliefsize == 0) {
        // create random belief
        // Room corners: (0, 0) to (10.2, 7.6)
        for (int i = 0; i < PARTICLE_NUM; i++) {
            belief.push_back(randomParticle());
        }
    }

    cout << "Initialized belief" << endl;

    motionEvolve(belief, myMoves);
    myMoves.reset();

    cout << "Evolved motion" << endl;

    vector<Line> map = makeMap();

    cout << "Entering CGR" << endl;

    CGRLocalize(belief, PlanePoints, PlaneNorms, map);

    // visualize2(PlanePoints);
    cout << "Pointcloud callback finished running." << endl;
}

void Everything::cmdCallback(const geometry_msgs::Twist& vel_cmd) 
{
    myMoves.makeMoves(vel_cmd.linear.x, vel_cmd.angular.z);
    // cout << "Just moved." << endl;
}


int main(int argc, char** argv)
{
    Everything container;

    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;


    sub = nh.subscribe("/filtered_voxel", 1,
                                        &Everything::pointcloudCallback,
                                        &container);

    sub2 = nh.subscribe("/cmd_vel", 1000, &Everything::cmdCallback,
                                            &container);

    cout << "Subscribers have been initialized." << endl;
    
    ros::spin();
    return 0;
}
