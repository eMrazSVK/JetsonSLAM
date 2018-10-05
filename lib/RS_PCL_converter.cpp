#include "../include/RS_PCL_converter.h"


std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    
    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}


pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    cv::Mat colorr(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", colorr);
        
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    int i = 1;
    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();
    
    // Tuple to store color of current vertices
    std::tuple<uint8_t, uint8_t, uint8_t> current_color;
    

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (auto& point : cloud->points)
    {
        // Setting XYZ coords
        point.x = vertices->x;
        point.y = vertices->y;
        point.z = vertices->z;
        
        // Get color of vertices of current iteration
        current_color = get_texcolor(color, tex_coords[i]);
        
        // Reversed order - 2-1-0 because of BGR model used in camera
        point.r = std::get<2>(current_color);
        point.g = std::get<1>(current_color);
        point.b = std::get<0>(current_color);
        
        vertices++;
        i++;
    }
    
   return cloud;
}

