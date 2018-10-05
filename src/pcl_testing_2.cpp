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

using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
using namespace cv;


std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}


pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color) {

    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    // PCL Cloud config
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    
    auto ptr = points.get_vertices();
    auto tex_coords = points.get_texture_coordinates();
    
    for (int i = 0; i < points.size(); ++i)
	{
        cloud->points[i].x = ptr[i].x;
		cloud->points[i].y = ptr[i].y;
		cloud->points[i].z = ptr[i].z;

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);

		cloud->points[i].r = std::get<0>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<2>(current_color);
	}

	return cloud;
}

int main() {

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    
    int i,j,count;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Start streaming with default recommended configuration
    pipe.start(cfg);
    
    // Stream profiles for extrinsics and intrinsics
    auto depth_stream = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    
    rs2_intrinsics intrinsics = depth_stream.get_intrinsics();
    rs2_extrinsics extrinsics = depth_stream.get_extrinsics_to(color_stream);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(i = 0; i < 100; i++)
    {
        // Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }
    std::string save_filename;
    count = 0;
    
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);

    //while(1) {
    
        frames = pipe.wait_for_frames();
        aligned_frames = align.process(frames);
        auto depth_frame = aligned_frames.get_depth_frame();
        auto color_frame = aligned_frames.get_color_frame();
        Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        points = pc.calculate(depth_frame);
        pc.map_to(color_frame);
        auto pcl_points = points_to_pcl(points, color_frame);
        
        pcl::visualization::CloudViewer viewer("To Som Ja");
        viewer.showCloud(pcl_points);
        cvNamedWindow( "Image", CV_WINDOW_AUTOSIZE );
        imshow("Image",color);
        cvWaitKey(0);

        save_filename = "testik" + std::to_string(++count) + ".pcd";
        pcl::io::savePCDFileASCII(save_filename, *pcl_points);
        /*
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        */

        while (!viewer.wasStopped()) {}
    //}
}


