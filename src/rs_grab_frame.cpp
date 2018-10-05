#include "../include/RS_PCL_converter.h"


int main() {

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
   
    // Start streaming with default recommended configuration
    rs2::pipeline_profile profile = pipe.start(cfg);
    
    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    // The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(RS2_STREAM_COLOR);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for(int i = 0; i < 100; i++)
    {
        // Wait for all configured streams to produce a frame
        auto frames = pipe.wait_for_frames();
    }
        
    auto frames = pipe.wait_for_frames();
    
    //Get processed aligned frame
    auto processed = align.process(frames);
    
    // Trying to get both other and aligned depth frames
    //auto colored_frame = processed.first(RS2_STREAM_COLOR);
    //auto depth = processed.get_depth_frame();
    
    auto depth = frames.get_depth_frame();
    auto colored_frame = frames.get_color_frame();

    // Order here is crucial! 
    // map_to() color frame has to be done befor point calculation
    // otherwise texture won't be mapped
    pc.map_to(colored_frame);
    auto points = pc.calculate(depth);

    // Actual calling of conversion and saving XYZRGB cloud to file
    pcl_ptr cloud = points_to_pcl(points, colored_frame);
    pcl::io::savePCDFileASCII("../shared_resources/2.pcd", *cloud);

    //cv::waitKey(0);

  
    return EXIT_SUCCESS;
}


