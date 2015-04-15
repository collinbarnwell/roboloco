#include "localize.hpp"
#include "my_viz.hpp"
#include "labmap.hpp"
#include "fspf.hpp"

#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

#define PARTICLE_NUM 200

using namespace std;
using namespace pcl;


class Everything 
{
    private:
        pcl::PointCloud<pcl::PointXYZ> my_cloud;
        // PointCloud<PointXYZ>::Ptr my_cloud_ptr(&my_cloud_global);

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

    pcl::fromROSMsg(msg, my_cloud);

    PointCloud<PointXYZ> PlanePoints;
    PointCloud<PointXY> PlaneNorms;

    fspf(my_cloud, PlanePoints, PlaneNorms);

    if (belief.size() == 0) {
        // create random belief
        // Room corners: (0, 0) to (10.2, 7.6)
        for (int i = 0; i < PARTICLE_NUM; i++) {
            belief.push_back(randomParticle());
        }
    }

    motionEvolve(belief, myMoves);
    myMoves.reset();

    vector<Line> map = makeMap();

    CGRLocalize(belief, PlanePoints, PlaneNorms, map);

    // visualize2(PlanePoints);
    cout << "Pointcloud callback finished running." << endl;
}

void Everything::cmdCallback(const geometry_msgs::Twist& vel_cmd) 
{
    myMoves.makeMoves(vel_cmd.linear.x, vel_cmd.angular.z);
    cout << "Just moved." << endl;
}


int main(int argc, char** argv) 
{
    Everything container;

    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/right_cloud_transform", 1,
                                        &Everything::pointcloudCallback,
                                        &container);

    ros::NodeHandle nh2;
    ros::Subscriber sub2 = nh2.subscribe("/cmd_vel", 1, &Everything::cmdCallback,
                                            &container);

    cout << "inside of node" << endl;
    
    ros::spin();
    return 0;
}
