#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

class RoverCamera{
public:
    RoverCamera(int argc, char** argv){
        ros::init(argc, argv, "camera");

        node_handle = new ros::NodeHandle("~");

        node_handle->param("is_rtsp_camera", is_rtsp_camera, false);
        node_handle->param("device_path", capture_device_path, std::string("/dev/video0"));
        node_handle->param("fps", fps, 30);

        node_handle->param("large_image_width", large_image_width, 1280);
        node_handle->param("large_image_height", large_image_height, 720);
        node_handle->param("medium_image_width", medium_image_width, 640);
        node_handle->param("medium_image_height", medium_image_height, 360);
        node_handle->param("small_image_width", small_image_width, 256);
        node_handle->param("small_image_height", small_image_height, 144);

        broadcast_large_image = true;
        broadcast_medium_image = false;
        broadcast_small_image = false;

        if(is_rtsp_camera){
            cap = new cv::VideoCapture(capture_device_path);

        }else{
            cap = new cv::VideoCapture(capture_device_path);

            cap->set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
            cap->set(CV_CAP_PROP_FRAME_WIDTH, large_image_width);
            cap->set(CV_CAP_PROP_FRAME_HEIGHT, large_image_height);
            cap->set(CV_CAP_PROP_FPS, fps);
        }

        large_image_node_name = "image_" + std::to_string(large_image_width) + "x" + std::to_string(large_image_height);
        medium_image_node_name = "image_" + std::to_string(medium_image_width) + "x" + std::to_string(medium_image_height);
        small_image_node_name = "image_" + std::to_string(small_image_width) + "x" + std::to_string(small_image_height);

        large_image_transport = new image_transport::ImageTransport(*node_handle);
        medium_image_transport = new image_transport::ImageTransport(*node_handle);
        small_image_transport = new image_transport::ImageTransport(*node_handle);

        large_image_publisher = large_image_transport->advertise(large_image_node_name, 1);
        medium_image_publisher = medium_image_transport->advertise(medium_image_node_name, 1);
        small_image_publisher = small_image_transport->advertise(small_image_node_name, 1);

        if(is_rtsp_camera){
            loop_rate = new ros::Rate(fps + 10);
        }else{
            loop_rate = new ros::Rate(fps + 2);
        }
    }

    void run(){
        if(!cap->isOpened()){
            std::cout << "Could not open device: " << capture_device_path << std::endl;
            return;
        }

        while (ros::ok()) {

            cap->read(image_large);

            if(!image_large.empty()){
                cv::resize(image_large, image_medium, cv::Size(medium_image_width, medium_image_height));
                cv::resize(image_medium, image_small, cv::Size(small_image_width, small_image_height));

                sensor_msgs::ImagePtr large_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_large).toImageMsg();
                sensor_msgs::ImagePtr medium_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_medium).toImageMsg();
                sensor_msgs::ImagePtr small_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_small).toImageMsg();

                large_image_publisher.publish(large_image_message);
                medium_image_publisher.publish(medium_image_message);
                small_image_publisher.publish(small_image_message);
            }

            ros::spinOnce();
            loop_rate->sleep();
        }
    }

    ~RoverCamera(){
        if(cap->isOpened()){
            cap->release();
        }
    }

private:
    ros::NodeHandle *node_handle;

    cv::VideoCapture *cap;

    bool is_rtsp_camera;

    std::string capture_device_path;
    int fps;

    ros::Rate *loop_rate;

    int large_image_width;
    int large_image_height;
    int medium_image_width;
    int medium_image_height;
    int small_image_width;
    int small_image_height;

    bool broadcast_large_image;
    bool broadcast_medium_image;
    bool broadcast_small_image;

    std::string large_image_node_name;
    std::string medium_image_node_name;
    std::string small_image_node_name;

    image_transport::ImageTransport *large_image_transport;
    image_transport::ImageTransport *medium_image_transport;
    image_transport::ImageTransport *small_image_transport;

    image_transport::Publisher large_image_publisher;
    image_transport::Publisher medium_image_publisher;
    image_transport::Publisher small_image_publisher;

    cv::Mat image_large;
    cv::Mat image_medium;
    cv::Mat image_small;

};

int main(int argc, char** argv)
{
    RoverCamera camera(argc, argv);
    camera.run();
}
