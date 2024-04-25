#include <sl/Camera.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed");
    ros::NodeHandle pnh{"~"};
    ros::NodeHandle nh;

    auto topic = pnh.param<std::string>("topic", "/image");
    auto pub = nh.advertise<sensor_msgs::Image>(topic, 1);

    tf2_ros::TransformBroadcaster broadcaster;

    sl::Camera zed;
    sl::InitParameters initParams;
    initParams.camera_resolution = sl::RESOLUTION::HD720;
    initParams.camera_fps = 60;
    initParams.depth_mode = sl::DEPTH_MODE::NONE;
    if (zed.open(initParams) != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error{"Failed to open ZED camera"};
    }

    sl::CameraParameters cameraParams = zed.getCameraInformation().camera_configuration.calibration_parameters.right_cam;

    sl::RuntimeParameters runtime_parameters;

    sl::Mat image;
    cv::Mat bgr;

    auto arucoDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    auto arucoParams = cv::aruco::DetectorParameters::create();

    while (ros::ok()) {
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, sl::VIEW::RIGHT);

            cv::Mat bgra{static_cast<int>(image.getHeight()), static_cast<int>(image.getWidth()), CV_8UC4, image.getPtr<sl::uchar1>()};
            cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

            std::vector<std::vector<cv::Point2f>> arucoCorners;
            std::vector<int> arucoIds;
            cv::aruco::detectMarkers(bgr, arucoDictionary, arucoCorners, arucoIds, arucoParams);
            cv::aruco::drawDetectedMarkers(bgr, arucoCorners, arucoIds);

            for (std::size_t i = 0; i < arucoIds.size(); ++i) {
                float sideLength = 0.09f;
                std::array<cv::Point3f, 4> objectPoints{
                        cv::Point3f{-sideLength / 2, -sideLength / 2, 0},
                        cv::Point3f{sideLength / 2, -sideLength / 2, 0},
                        cv::Point3f{sideLength / 2, sideLength / 2, 0},
                        cv::Point3f{-sideLength / 2, sideLength / 2, 0},
                };
                cv::Matx33f cameraMatrix{
                        cameraParams.fx, 0, cameraParams.cx,
                        0, cameraParams.fy, cameraParams.cy,
                        0, 0, 1};
                cv::Matx<double, 5, 1> distCoeffs{cameraParams.disto[0], cameraParams.disto[1], cameraParams.disto[2], cameraParams.disto[3], cameraParams.disto[4]};

                cv::Vec3f rvec;
                cv::Vec3f tvec;
                cv::solvePnP(objectPoints, arucoCorners[i], cameraMatrix, distCoeffs, rvec, tvec);

                float angle = cv::norm(rvec);
                cv::Vec3f axis = rvec / angle;

                axis *= -1;
                tvec[0] *= -1;

                geometry_msgs::TransformStamped tf;
                tf.header.stamp = ros::Time::now();
                tf.header.frame_id = "map";
                tf.child_frame_id = "bot_" + std::to_string(arucoIds[i]);
                tf.transform.translation.x = tvec[0];
                tf.transform.translation.y = tvec[1];
                tf.transform.rotation.x = axis[0] * std::sin(angle / 2);
                tf.transform.rotation.y = axis[1] * std::sin(angle / 2);
                tf.transform.rotation.z = axis[2] * std::sin(angle / 2);
                tf.transform.rotation.w = std::cos(angle / 2);
                broadcaster.sendTransform(tf);
            }

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