#include <sl/Camera.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <sensor_msgs/Image.h>

#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed");
    ros::NodeHandle pnh{"~"};
    ros::NodeHandle nh;

    auto topic = pnh.param<std::string>("topic", "/image");
    auto pub = nh.advertise<sensor_msgs::Image>(topic, 1);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 60;
    init_params.depth_mode = sl::DEPTH_MODE::NONE;
    if (zed.open(init_params) != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error{"Failed to open ZED camera"};
    }

    sl::RuntimeParameters runtime_parameters;

    sl::Mat image;
    cv::Mat bgr;

    while (ros::ok()) {
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, sl::VIEW::LEFT);

            cv::Mat bgra{static_cast<int>(image.getHeight()), static_cast<int>(image.getWidth()), CV_8UC4, image.getPtr<sl::uchar1>()};
            cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time::now();
            msg.height = bgr.rows;
            msg.width = bgr.cols;
            msg.encoding = "bgr8";
            msg.is_bigendian = false;
            msg.step = bgr.step;
            msg.data = {bgr.data, bgr.data + bgr.total() * bgr.elemSize()};
            pub.publish(msg);

        } else {
            throw std::runtime_error{"Failed to grab frame"};
        }

        ros::spinOnce();
    }

    zed.close();
    return 0;
}