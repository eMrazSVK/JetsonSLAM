#include <iostream>
#include <string.h>
#include "../include/RS_PCL_converter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/features/pfh.h"
#include <pcl/impl/point_types.hpp>
#include "pcl/features/pfh.h"
#include "pcl/kdtree/kdtree_flann.h"


namespace jSlamReg {

    // params: input cloud, file name
    void loadCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                   std::string);
    
    // params: input cloud, output cloud, upper filter limit,
    //         lower filter limit, axis name
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                           int,
                           int,
                           std::string);
   
    // params: input cloud, leaf size 1, leaf size 2, leaf size 3       
    void downSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                    float,
                    float,
                    float);
                    
    // params: input cloud, result points, min_scale
    //         n_octaves, n_scale_per_octave, min_contrast
    void findSIFTKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                           float,
                           int,
                           int,
                           float);
                           
    // params: input cloud, normals output, search radius
    void estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                         pcl::PointCloud<pcl::Normal>::Ptr,
                         float);
     
    // params: input cloud, keypoints, normals,feature radius, output descriptors             
    void computePFHFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                            pcl::PointCloud<pcl::Normal>::Ptr,
                            float,
                            pcl::PointCloud<pcl::PFHSignature125>::Ptr);
                            
    // params: source descriptors, target descriptors, output correspondences, 
    //         output correspondence scores 
    void findCorrespondences(pcl::PointCloud<pcl::PFHSignature125>::Ptr,
                             pcl::PointCloud<pcl::PFHSignature125>::Ptr,
                             std::vector<int>&, 
                             std::vector<float>&);
     
    //params:  
                             
    void visualize_correspondences(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                                   std::vector<int>,
                                   std::vector<float>);
                                   
    void visualize_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                           const pcl::PointCloud<pcl::Normal>::Ptr);
           
              
    void visualize_keypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                          const pcl::PointCloud<pcl::PointWithScale>::Ptr);
                               
}
