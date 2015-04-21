#ifndef FSPF_FXN
#define FSPF_FXN

#include "pcl/pcl_base.h"
#include "pcl/PointIndices.h"
#include "pcl/conversions.h"
// #include "pcl/common/impl/io.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>

#include "basic_utilities.hpp"



using namespace std;
using namespace pcl;


void fspf(PointCloud<PointXYZ> my_cloud, PointCloud<PointXYZ>::Ptr my_cloud_ptr, PointCloud<PointXYZ> &PlanePoints, PointCloud<PointXY> &PlanePointsNormals) {

    // void fspf(PointCloud<PointXYZ> &my_cloud, PointCloud<PointXYZ> &PlanePoints, PointCloud<PointXY> &PlanePointsNormals) {

    // PointCloud<PointXYZ>::Ptr my_cloud_ptr(&my_cloud); // possibly trying to be freed

    // PointCloud<PointXYZ> my_cloud = *my_cloud_ptr;
    // PointCloud<PointXYZ>::Ptr my_cloud_ptr = my_cloud.makeShared();


    PointCloud<PointXYZ> range;
    // PointCloud<PointXYZ>::Ptr rangeptr(&range);    // problem
    pcl::PointCloud<PointXYZ> Outliers;

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
        boost::shared_ptr<vector<int> > rangeIndices (new vector<int> (pointIdxRadiusSearch));

        // filter range to new pointcloud
        ExtractIndices<PointXYZ> rangefilter;
        rangefilter.setInputCloud(my_cloud_ptr);
        rangefilter.setIndices(rangeIndices);
        rangefilter.setNegative(false);
        rangefilter.filter(range);

        // create best plane pointcloud
        vector<int> bestInliers;
        Normal bestNorm;
        PointCloud<PointXYZ> BPP;

        // ExtractIndices<PointXYZ> BPPfilter;        
        // BPPfilter.setInputCloud(rangeptr);

        int rangesize = range.size();
        if (rangesize < NEIGH_DENSITY) { continue; }

        for (int i = 0; i < LOCAL_SAMPS; i++) {
            // pick 2 more points
            PointXYZ b = range[rand()%rangesize];
            PointXYZ c = range[rand()%rangesize];

            // make a plane
            Normal norm = planeNorm(a, b, c);

            // find points inliers
            vector<int> inlierIndices;
            for (int j = 0; j < rangesize; j++) {
                int currIndex = pointIdxRadiusSearch[j];
                PointXYZ currpoint = my_cloud[currIndex]; // make sure these are proper indices
                float pdst = planeToPtDist(currpoint, a, norm);
                if (pdst <= PLANE_OFFSET) {
                    inlierIndices.push_back(currIndex); // problems
                }
            }

            // is it better or worse than best
            if (inlierIndices.size() > numinliers) {
                // filter inliers to BPP if best

                // boost::shared_ptr<vector<int> > inliers(new vector<int> (inlierIndices));
                // BPPfilter.setIndices(inliers);
                // BPPfilter.setNegative(false);
                // BPPfilter.filter(BPP);

                BPP.clear();

                int maxq = inlierIndices.size();
                for (int q = 0; q < maxq; q++) 
                {
                    int ind = inlierIndices[q];
                    BPP.push_back(my_cloud.points[ind]);
                }

                // save indices to inliers if best
                bestInliers = inlierIndices;

                numinliers = inlierIndices.size();
                bestNorm = norm;
            }
        }

        // add best inliers to Outliers or PlanePoints
        if (numinliers >= MIN_INLIER*numsearch) {
            n += numinliers;
            PlanePoints += BPP;

            // declare temporary normals vector
            PointCloud<PointXY> normals;

            int bppsize = BPP.size();
            for (int i = 0; i < bppsize; i++) {
                // calculate 2D-projected normal of BPP
                PointXY newpoint = projectNorm(bestNorm);
                normals.push_back(newpoint);
            }

            PlanePointsNormals += normals;

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

    cout << "Filtered Cloud size: " << PlanePoints.size() << endl;
    cout << "Outliers size: " << Outliers.size() << endl;

    return;
}


#endif
