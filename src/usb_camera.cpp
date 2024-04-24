#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <sensor_msgs/Image.h>

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "usb_camera");
    ros::NodeHandle pnh{"~"};
    ros::NodeHandle nh;

    int cameraId = pnh.param<int>("camera_id", 4);

    cv::VideoCapture cap{
            "v4l2src device=/dev/video4 "
            "! image/jpeg,width=1280,height=720,framerate=30/1 "
            "! jpegdec "
            "! video/x-raw,format=I420 " // For some reason fails if trying to use BGR
            "! appsink",
            cv::CAP_GSTREAMER};

    auto topic = pnh.param<std::string>("topic", "/image");
    auto pub = nh.advertise<sensor_msgs::Image>(topic, 1);

    ros::Rate rate{30};
    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) throw std::runtime_error{"Failed to capture frame"};

        cv::Mat bgr;
        cvtColor(frame, bgr, cv::COLOR_YUV2BGR_I420);

        cv::Matx33d cameraMatrix{945.69568215, 0., 659.99381561, 0., 951.85384486, 365.3696245, 0., 0., 1.};
        cv::Matx<double, 5, 1> distCoeffs{-0.52908235, 0.28727436, 0.00566978, -0.00345952, -0.02408352};

        cv::Mat bgrUndistorted;
        undistort(bgr, bgrUndistorted, cameraMatrix, distCoeffs);

        sensor_msgs::Image msg;
        msg.header.stamp = ros::Time::now();
        msg.height = bgrUndistorted.rows;
        msg.width = bgrUndistorted.cols;
        msg.encoding = "bgr8";
        msg.is_bigendian = false;
        msg.step = bgrUndistorted.step;
        msg.data = {bgrUndistorted.data, bgrUndistorted.data + bgrUndistorted.total() * bgrUndistorted.elemSize()};

        pub.publish(msg);

        ros::spinOnce();

        rate.sleep();
    }

    return EXIT_SUCCESS;
}