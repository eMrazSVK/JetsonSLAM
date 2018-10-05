#include <librealsense2/rs.hpp>
#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ptr;


std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame, rs2::texture_coordinate);

pcl_ptr points_to_pcl(const rs2::points&, const rs2::video_frame& );
