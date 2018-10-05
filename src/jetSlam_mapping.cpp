#include "../include/RS_PCL_converter.h"
#include "../include/jetSlam_registration_toolkit.h"


int main () {
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("8.pcd", *cloud);

    // Passthrough filter for discarding distant points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);
    
    // Downsample using Voxel Grid
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.001f, 0.001f, 0.001f);
    sor.filter(*cloud_filtered);   
    
    pcl::io::savePCDFileASCII("filtered.pcd", *cloud_filtered);


    // ====== REGISTERING CLOUDS =======
    // Basic pipeline for registering clouds
    // 0. Load clouds - already done
    // 1. Compute keypoints - using SIFT
    // 2. Compute feature descriptors - Normals
    // 3. Correspondence Estimation - Unknown yet
    // 4. Correspondence Rejection  - RANSAC
    const float min_scale = 0.05f;
    const int n_octaves = 6;
    const int n_scales_per_octave = 10;
    const float min_contrast = 0.2f;
    
    // Estimate the sift interest points using Intensity values from RGB values
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_filtered);
    sift.compute(result);

    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);
    
    pcl::io::savePCDFileASCII("sift_points.pcd", *cloud_temp);
    
    
    // Feature descriptors - 2. krok
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_temp);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree2);
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the featurescorrectly and use the same memory management context for all your shared object libraries.

In
    ne.compute (*cloud_normals);
    
    // Visualization of keypoints along with the original cloud
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud_filtered, 255, 255, 0);
    viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
    viewer.addPointCloud(cloud_filtered, "cloud");
    viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    while(!viewer.wasStopped ())
    {
    viewer.spinOnce ();
    }
    */
        
    //=================================- TOOLKIT TESTING -===================================
    // raw input clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // processed clouds 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_proc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2_proc(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // loading clouds from file - temporary
    jSlamReg::loadCloud(cloud_1,"../shared_resources/2.pcd");
    jSlamReg::loadCloud(cloud_2,"../shared_resources/2.pcd");
    
    // discarding distant points
    jSlamReg::passThroughFilter(cloud_1, cloud_1_proc, 2.0, 0.0, "z");
    jSlamReg::passThroughFilter(cloud_2, cloud_2_proc, 2.0, 0.0, "z");
    
    // reduction of points
    jSlamReg::downSample(cloud_1_proc, 0.005f, 0.005f, 0.005f);
    jSlamReg::downSample(cloud_2_proc, 0.005f, 0.005f, 0.005f);
    
    // containers holding detected keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_2(new pcl::PointCloud<pcl::PointXYZRGB>);;

    // compute keypoints based on SIFT
    jSlamReg::findSIFTKeyPoints(cloud_1_proc, keypoints_1, 0.01f, 3, 3, 10.0);
    jSlamReg::findSIFTKeyPoints(cloud_2_proc, keypoints_2, 0.01f, 3, 3, 10.0);                            

    // containers holding estimating normals
    pcl::PointCloud<pcl::Normal>::Ptr est_normals_1(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr est_normals_2(new pcl::PointCloud<pcl::Normal>);
    
    // estimate normals on the processed cloud                               
    jSlamReg::estimateNormals(cloud_1_proc, est_normals_1, 0.03);
    jSlamReg::estimateNormals(cloud_2_proc, est_normals_2, 0.03);
    
    // containers holding the PFH feature descriptors
    pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH_descriptors_1(new pcl::PointCloud<pcl::PFHSignature125>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH_descriptors_2(new pcl::PointCloud<pcl::PFHSignature125>);
    
    std::cout<<keypoints_1->size()<<std::endl;
    // compute PFH Feature descriptors
    jSlamReg::computePFHFeatures(cloud_1_proc, keypoints_1, est_normals_1, 0.03, PFH_descriptors_1);
    jSlamReg::computePFHFeatures(cloud_2_proc, keypoints_2, est_normals_2, 0.03, PFH_descriptors_2);
    
    
    // containers holding correspondences data
    std::vector<int> correspondences;
    std::vector<float> correspondence_scores;
    
    // compute correspondences between pair of clouds
    jSlamReg::findCorrespondences(PFH_descriptors_1, PFH_descriptors_2, correspondences, correspondence_scores);
    
    std::cout<<correspondences.size()<<std::endl;
    // Visualise correspondeces
    
    jSlamReg::visualize_correspondences(cloud_1_proc,
                                        keypoints_1,
                                        cloud_2_proc,
                                        keypoints_2,
                                        correspondences,
                                        correspondence_scores);
                                        
    jSlamReg::visualize_normals(cloud_1, cloud_1_proc, est_normals_1);
    
    
    // Just visualize results
    /*     
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (keypoints_1, 255, 255, 0);
    
    viewer.addPointCloud(cloud_1_proc, "cloudd");
    viewer.addPointCloud(keypoints_1, cloud_color_handler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    
    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    */
    return EXIT_SUCCESS;

}
