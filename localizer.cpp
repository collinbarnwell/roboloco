#include "utilities.h"

#define MAX_HOODS 20000 // kmax
#define MAX_PTS 2000 // nmax
#define LOCAL_SAMPS 80 // (l) num local samples
#define PLANE_SIZE .5 // S
#define PLANE_OFFSET .02 // e
#define MIN_INLIER .8 // alpha-in
#define NEIGH_SIZE .5 // replaces Global_neigh
#define GLOBAL_NEIGH 60 // n Neighborhood for global samples (in pixels) *** NOT USED ** 

using namespace std;
using namespace pcl;


void fspf(pcl::PointCloud<pcl::PointXYZ> cloud);
pcl::PointCloud<pcl::PointXYZ> my_cloud;


void pointcloudCallback(const sensor_msgs::PointCloud2 msg) { 
    cout << "inside callback" << endl;
    
    // pcl::fromROSMsg(msg, my_cloud);
    
    // // visualize(my_cloud);
    // cout << "HAI" << endl;

    // PointCloud<PointXYZ> filteredPoints = fspf(my_cloud);
    // cout << "K BAI" << endl;
    // visualize(filteredPoints);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    cout << "hai" << endl;
    ros::Subscriber sub = nh.subscribe("/right_cloud_transform", 1, pointcloudCallback);

    cout << "inside of node" << endl;
    
    ros::spin();
    return 0;
}

void fspf(pcl::PointCloud<pcl::PointXYZ> cloud) {
    pcl::PointCloud<PointXYZ> PlanePoints;
    pcl::PointCloud<PointXYZ> Outliers;
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>(cloud);

    int k = 0;
    int n = 0;

    // new cloud that will shrink
    PointCloud<PointXYZ> shrinkingCloud(cloud);


    while (k < MAX_HOODS && n < MAX_PTS) {
        k++;
        int cloudsize = shrinkingCloud.size();

        // Organize cloudinto kdtree for nearest neighbor search
        PointCloud<PointXYZ>::Ptr cloudptr(&shrinkingCloud);
        KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud(cloudptr);

        // get random point a
        int aindex = rand()%cloudsize;
        PointXYZ a = cloud[aindex];

        //RANSAC
        vector<float> pointRadiusSquaredDistance;
        vector<int> pointIdxRadiusSearch;
        int numinliers = 0;
        int numsearch = kdtree.radiusSearch(a, NEIGH_SIZE, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        boost::shared_ptr<vector<int> > rangeIndices (new vector<int> (pointIdxRadiusSearch));

        // filter range to new pointcloud
        ExtractIndices<PointXYZ> rangefilter;
        rangefilter.setInputCloud(cloudptr);
        rangefilter.setIndices(rangeIndices);
        rangefilter.setNegative(false);
        PointCloud<PointXYZ> range;
        rangefilter.filter(range);

        // create best plane pointcloud
        PointCloud<PointXYZ> BPP;
        PointCloud<PointXYZ>::Ptr rangeptr(&range);
        vector<int> bestInliers;
        
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
                PointXYZ currpoint = shrinkingCloud[currIndex]; // make sure these are proper indices
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
        extract.setInputCloud (cloudptr);
        boost::shared_ptr<vector<int> > bestInlierIndices(new vector<int> (bestInliers));
        extract.setIndices (bestInlierIndices);
        //extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud 
        extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest 
        extract.filter (shrinkingCloud); 
    }

    // return PlanePoints;
}
