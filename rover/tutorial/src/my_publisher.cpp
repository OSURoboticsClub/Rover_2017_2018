#include <string.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    std::string capture_device_path;
    int fps;
    
    int large_image_width;
    int large_image_height;
    int small_image_width;
    int small_image_height;
    
    ros::init(argc, argv, "camera");
    ros::NodeHandle node_handle("~");
    
    node_handle.param("device_path", capture_device_path, std::string("/dev/video0"));
    node_handle.param("fps", fps, 30);
    
    node_handle.param("large_image_width", large_image_width, 1280);
    node_handle.param("large_image_height", large_image_height, 720);
    node_handle.param("small_image_width", small_image_width, 640);
    node_handle.param("small_image_height", small_image_height, 360);
        
    cv::VideoCapture cap(capture_device_path);

    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, large_image_width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, large_image_height);
    cap.set(CV_CAP_PROP_FPS, fps);

    if(cap.isOpened() == false){
        return -1;
    }
    
    image_transport::ImageTransport full_res_image_transport(node_handle);
    image_transport::ImageTransport lower_res_image_transport(node_handle);
    
    image_transport::Publisher full_size_publisher = full_res_image_transport.advertise("image_720p", 1);
    image_transport::Publisher lower_size_publisher = lower_res_image_transport.advertise("image_360p", 1);

    cv::Mat image;
    cv::Mat image_smaller;

    ros::Rate loop_rate(fps + 5);
    
    
    while (node_handle.ok()) {
        
        cap.read(image);
        
        if(!image.empty()){
            cv::resize(image, image_smaller, cv::Size(small_image_width, small_image_height));
            
            sensor_msgs::ImagePtr full_res_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            sensor_msgs::ImagePtr lower_res_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_smaller).toImageMsg();
            
            full_size_publisher.publish(full_res_message);
            lower_size_publisher.publish(lower_res_message);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

