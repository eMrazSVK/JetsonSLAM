#include "../include/jetSlam_registration_toolkit.h"


void jSlamReg::loadCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                         std::string filename) 
{
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud);
}


void jSlamReg::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
                                 int filter_upper_limit,
                                 int filter_lower_limit,
                                 std::string axis)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(filter_lower_limit, filter_upper_limit);
    
    pass.setInputCloud(cloud_in);
    pass.filter(*cloud_out);
} 

void jSlamReg::downSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                          float leaf_1,
                          float leaf_2,
                          float leaf_3)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize(leaf_1, leaf_2, leaf_3);
    
    sor.setInputCloud(cloud_in);
    sor.filter(*cloud_in);
}

              
void jSlamReg::findSIFTKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr result, 
                                 float min_scale,
                                 int n_octaves,
                                 int n_scales_per_octave,
                                 float min_contrast)
{
    pcl::PointCloud<pcl::PointWithScale> result_tmp;
    
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
   
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    
    sift.setInputCloud(cloud_in);
    sift.compute(result_tmp);
    
    copyPointCloud(result_tmp, *result);
 
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_hhh (new pcl::PointCloud<pcl::PointWithScale>);
    copyPointCloud(result_tmp, *keypoints_hhh);
    jSlamReg::visualize_keypoints(cloud_in, keypoints_hhh);

}


void jSlamReg::estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                               pcl::PointCloud<pcl::Normal>::Ptr normals_out,
                               float radius)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normals;
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search   surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    normals.setSearchMethod(tree2);

    // Use all neighbors in a sphere of radius 3cm
    normals.setRadiusSearch(radius);
   
    // Compute the features
    normals.setInputCloud(cloud_in);
    normals.compute(*normals_out);
}

                  
void jSlamReg::computePFHFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
                                  pcl::PointCloud<pcl::Normal>::Ptr normals,
                                  float feature_radius,
                                  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_out)
{
    // Create a PFHEstimation object
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;
    
    // Set it to use a FLANN-based KdTree to perform its
    // neighborhood searches
    pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    // Specify the radius of the PFH feature
    pfh_est.setRadiusSearch(feature_radius);
    
    // Use all of the points for analyzing the local structure of the cloud
    pfh_est.setSearchSurface(cloud_in);
    pfh_est.setInputNormals(normals);
    
    pfh_est.setInputCloud(keypoints);
    
    pfh_est.compute(*descriptors_out);
}


void jSlamReg::findCorrespondences(pcl::PointCloud<pcl::PFHSignature125>::Ptr source_descriptors,
                                   pcl::PointCloud<pcl::PFHSignature125>::Ptr target_descriptors,
                                   std::vector<int> &correspondences_out, 
                                   std::vector<float> &correspondence_scores_out)
{
    // Resize the output vector
    correspondences_out.resize (source_descriptors->size ());
    correspondence_scores_out.resize (source_descriptors->size ());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source_descriptors->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }

}


void jSlamReg::visualize_correspondences (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
                                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1,
                                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
                                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2,
                                          std::vector<int> correspondences,
                                          std::vector<float> correspondence_scores)
{
    // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
    // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

    // Create some new point clouds to hold our transformed data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Shift the first clouds' points to the left
    // const Eigen::Vector3f translate (0.0, 0.0, 0.3);
    const Eigen::Vector3f translate (0.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
    pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

    // Shift the second clouds' points to the right
    pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
    pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

    // Add the clouds to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (points_left, "points_left");
    viz.addPointCloud (points_right, "points_right");

    // Compute the median correspondence score
    std::vector<float> temp (correspondence_scores);
    std::sort (temp.begin (), temp.end ());
    float median_score = temp[temp.size ()/2];

    // Draw lines between the best corresponding points
    for (size_t i = 0; i < keypoints_left->size (); ++i)
    {
    if (correspondence_scores[i] > median_score)
    {
      continue; // Don't draw weak correspondences
    }

    // Get the pair of points
    const pcl::PointXYZRGB & p_left = keypoints_left->points[i];
    const pcl::PointXYZRGB & p_right = keypoints_right->points[correspondences[i]];

    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;

    // Draw the line
    viz.addLine (p_left, p_right, r, g, b, ss.str ());
    }

    // Give control over to the visualizer
    viz.spin ();
}


void jSlamReg::visualize_normals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_points,
                        const pcl::PointCloud<pcl::Normal>::Ptr normals)                      
{
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  viz.addPointCloud (normal_points, "normal_points");

  viz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (normal_points, normals, 1, 0.01, "normals");

  // Give control over to the visualizer
  viz.spin ();
}
        
        
void jSlamReg::visualize_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                                   const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints)
{
  // Add the points to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");

  // Draw each keypoint as a sphere
  for (size_t i = 0; i < keypoints->size (); ++i)
  {
    // Get the point data
    const pcl::PointWithScale & p = keypoints->points[i];

    // Pick the radius of the sphere *
    float r = 2 * p.scale;
    // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
    //   radius of 2*p.scale is a good illustration of the extent of the keypoint

    // Generate a unique string for each sphere
    std::stringstream ss ("keypoint");
    ss << i;

    // Add a sphere at the keypoint
    viz.addSphere (p, 2*p.scale, 1.0, 0.0, 0.0, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();
}

