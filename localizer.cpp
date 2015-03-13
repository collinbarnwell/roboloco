// #ifndef HEADERFILE_H
// #define LOCALIZER_H

// #include "raycast.hpp"
#include "basic_utilities.hpp"
#include "my_viz.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include "pcl/pcl_base.h"
#include "pcl/PointIndices.h"
#include "pcl/conversions.h"
#include "pcl/common/impl/io.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#define MAX_HOODS 30000 // kmax
#define MAX_PTS 20000 // nmax
#define LOCAL_SAMPS 80 // (l) num local samples
// #define PLANE_SIZE .5 // S
#define PLANE_OFFSET .03 // e
#define MIN_INLIER .9 // alpha-in
#define NEIGH_SIZE .25 // replaces Global_neigh
#define NEIGH_DENSITY 20
// #define GLOBAL_NEIGH 60 // n Neighborhood for global samples (in pixels) *** NOT USED ** 

using namespace std;
using namespace pcl;


void fspf();

pcl::PointCloud<pcl::PointXYZ> my_cloud;
PointCloud<PointXYZ>::Ptr my_cloud_ptr(&my_cloud);

PointCloud<PointXYZ> range;
PointCloud<PointXYZ>::Ptr rangeptr(&range);

pcl::PointCloud<PointXYZ> PlanePoints;
pcl::PointCloud<PointXYZ> Outliers;  


void pointcloudCallback(const sensor_msgs::PointCloud2 msg) { 
    cout << "inside callback" << endl;
    PlanePoints.clear();
    Outliers.clear();

    // PointCloud<PointXYZ> cld;// = my_cloud;
    pcl::fromROSMsg(msg, my_cloud);

    // visualize1(my_cloud);

    // filter cloud to new smaller pointcloud

    cout << "Old cloud size: " << my_cloud.size();

    VoxelGrid<PointXYZ> sor;
    sor.setInputCloud (my_cloud_ptr);
    sor.setLeafSize (0.02, 0.02, 0.02);
    sor.filter (my_cloud);

    cout << "   New cloud size: " << my_cloud.size();

    fspf();
    cout << "     Filtered Cloud size: " << PlanePoints.size() << endl;
    cout << "Outliers: " << Outliers.size() << endl;

    visualize2(PlanePoints);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/right_cloud_transform", 1, pointcloudCallback);

    cout << "inside of node" << endl;
    
    ros::spin();
    return 0;
}

void fspf() {

    int k = 0;
    int n = 0;

    while (k < MAX_HOODS && n < MAX_PTS) 
    {
        k++;
        int cloudsize = my_cloud.size();

        // Organize cloudinto kdtree for nearest neighbor search
        KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud(my_cloud_ptr);

        // get random point a
        int aindex = rand()%cloudsize;
        PointXYZ a = my_cloud[aindex];

        //RANSAC
        vector<float> pointRadiusSquaredDistance;
        vector<int> pointIdxRadiusSearch;
        int numinliers = 0;
        kdtree.radiusSearch(a, NEIGH_SIZE, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        int numsearch = pointIdxRadiusSearch.size();
        // cout << numsearch << endl;
        boost::shared_ptr<vector<int> > rangeIndices (new vector<int> (pointIdxRadiusSearch));

        // filter range to new pointcloud
        ExtractIndices<PointXYZ> rangefilter;
        rangefilter.setInputCloud(my_cloud_ptr);
        rangefilter.setIndices(rangeIndices);
        rangefilter.setNegative(false);
        rangefilter.filter(range);

        // create best plane pointcloud
        PointCloud<PointXYZ> BPP;
        vector<int> bestInliers;

        if (range.size() < NEIGH_DENSITY) { continue; }
        
        for (int i = 0; i < LOCAL_SAMPS; i++) {
            // pick 2 more points
            PointXYZ b = range[rand()%range.size()];
            PointXYZ c = range[rand()%range.size()];

            // make a plane
            Normal norm = planeNorm(a, b, c);

            // find points inliers
            vector<int> inlierIndices;
            for (int j = 0; j < range.size(); j++) {
                int currIndex = pointIdxRadiusSearch[j];
                PointXYZ currpoint = my_cloud[currIndex]; // make sure these are proper indices
                float pdst = planeToPtDist(currpoint, a, norm);
                if (pdst <= PLANE_OFFSET) {
                    inlierIndices.push_back(currIndex);
                }
            }

            // is it better or worse than best
            if (inlierIndices.size() > numinliers) {
                // filter inliers to BPP if best
                ExtractIndices<PointXYZ> BPPfilter;
                BPPfilter.setInputCloud(rangeptr);
                boost::shared_ptr<vector<int> > inliers(new vector<int> (inlierIndices));
                BPPfilter.setIndices(inliers);
                BPPfilter.setNegative(false);
                BPPfilter.filter(BPP);

                // save indices to inliers if best
                bestInliers = inlierIndices;
                numinliers = inlierIndices.size();
            }
        }

        // add best inliers to Outliers or PlanePoints
        if (numinliers >= MIN_INLIER*numsearch) {
            n += numinliers;
            PlanePoints += BPP;
        } else {
            Outliers += BPP;
        }
        
        // Extract possible points from shrinkingcloud
        ExtractIndices<PointXYZ> extract ; 
        extract.setInputCloud (my_cloud_ptr);
        boost::shared_ptr<vector<int> > bestInlierIndices(new vector<int> (bestInliers));
        extract.setIndices (bestInlierIndices);
        //extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud 
        extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest 
        extract.filter (my_cloud);
    }
}
