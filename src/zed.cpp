#include <sl/Camera.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <numeric>

constexpr float CAMERA_HEIGHT = 1.4;

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed");
    ros::NodeHandle pnh{"~"};
    ros::NodeHandle nh;

    auto topic = pnh.param<std::string>("topic", "/image");
    auto pub = nh.advertise<sensor_msgs::Image>(topic, 1);
    auto debugPub = nh.advertise<sensor_msgs::Image>(topic + "_aruco_detections", 1);

    tf2_ros::TransformBroadcaster broadcaster;

    sl::Camera zed;
    sl::InitParameters initParams;
    initParams.camera_resolution = sl::RESOLUTION::HD720;
    initParams.camera_fps = 60;
    initParams.depth_mode = sl::DEPTH_MODE::NONE;
    if (zed.open(initParams) != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error{"Failed to open ZED camera"};
    }

    sl::CameraParameters cameraParams = zed.getCameraInformation().camera_configuration.calibration_parameters_raw.right_cam;

    sl::RuntimeParameters runtime_parameters;

    sl::Mat image;

    cv::Mat bgrDistored, bgr;

    auto arucoDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    auto arucoParams = cv::aruco::DetectorParameters::create();

    cv::Matx33f cameraMatrix{
            cameraParams.fx, 0, cameraParams.cx,
            0, cameraParams.fy, cameraParams.cy,
            0, 0, 1};

    cv::Vec<double, 4> fisheyeDistCoeffs{
            cameraParams.disto[0],
            cameraParams.disto[1],
            cameraParams.disto[4],
            cameraParams.disto[5],
    };

    cv::Matx33f optimalCameraMatrix;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, fisheyeDistCoeffs, cv::Size{1280, 720}, cv::Matx33f::eye(), optimalCameraMatrix, 1);

    ROS_INFO_STREAM("Optimal camera matrix: " << optimalCameraMatrix);

    auto imageToWorld = [&](cv::Point2f const& point, float height) -> cv::Point2f {
        float fx = optimalCameraMatrix(0, 0);
        float fy = optimalCameraMatrix(1, 1);
        float cx = optimalCameraMatrix(0, 2);
        float cy = optimalCameraMatrix(1, 2);
        float bearingX = (point.x - cx) / fx;
        float bearingY = -(point.y - cy) / fy;
        float positionX = std::tan(bearingX) * height;
        float positionY = std::tan(bearingY) * height;
        return {positionX, positionY};
    };

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 200;
    params.maxArea = 1200;
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    params.maxCircularity = 1;
    params.filterByConvexity = true;
    params.minConvexity = 0.8;
    params.maxConvexity = 1;
    params.filterByColor = true;
    params.blobColor = 255;
    auto detector = cv::SimpleBlobDetector::create(params);

    while (ros::ok()) {
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, sl::VIEW::RIGHT_UNRECTIFIED);

            cv::Mat bgra{static_cast<int>(image.getHeight()), static_cast<int>(image.getWidth()), CV_8UC4, image.getPtr<sl::uchar1>()};
            cv::cvtColor(bgra, bgrDistored, cv::COLOR_BGRA2BGR);

            cv::fisheye::undistortImage(bgrDistored, bgr, cameraMatrix, fisheyeDistCoeffs, optimalCameraMatrix);

            std::vector<std::vector<cv::Point2f>> arucoCorners;
            std::vector<int> arucoIds;
            cv::aruco::detectMarkers(bgr, arucoDictionary, arucoCorners, arucoIds, arucoParams);

            for (std::size_t i = 0; i < arucoIds.size(); ++i) {
                cv::Point2f center = std::accumulate(arucoCorners[i].begin(), arucoCorners[i].end(), cv::Point2f{}) / 4;

                cv::Point2f positionInMap = imageToWorld(center, CAMERA_HEIGHT - 0.16);

                cv::Point2f sideDirection = arucoCorners[i][1] - arucoCorners[i][0];
                float angleInMap = -std::atan2(sideDirection.y, sideDirection.x) + M_PI;

                geometry_msgs::TransformStamped tf;
                tf.header.stamp = ros::Time::now();
                tf.header.frame_id = "map";
                tf.child_frame_id = "bot_" + std::to_string(arucoIds[i]);
                tf.transform.translation.x = positionInMap.x;
                tf.transform.translation.y = positionInMap.y;
                tf.transform.rotation.z = std::sin(angleInMap / 2);
                tf.transform.rotation.w = std::cos(angleInMap / 2);
                broadcaster.sendTransform(tf);
            }

            cv::Mat blur;
            cv::GaussianBlur(bgr, blur, {11, 11}, 0);

            cv::Mat hsv;
            cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

            cv::Mat thresh;
            cv::inRange(hsv, cv::Scalar{0, 130, 40}, cv::Scalar{30, 255, 255}, thresh);

            std::vector<cv::KeyPoint> keypoints;
            detector->detect(thresh, keypoints);

            if (keypoints.size() > 1) ROS_WARN_STREAM_THROTTLE(1, "Too many balls!");

            if (keypoints.size() == 1) {
                cv::KeyPoint const& keypoint = keypoints.front();

                cv::Point2f positionInMap = imageToWorld(keypoint.pt, CAMERA_HEIGHT);

                geometry_msgs::TransformStamped tf;
                tf.header.stamp = ros::Time::now();
                tf.header.frame_id = "map";
                tf.child_frame_id = "ball";
                tf.transform.translation.x = positionInMap.x;
                tf.transform.translation.y = positionInMap.y;
                tf.transform.rotation.w = 1;
                broadcaster.sendTransform(tf);
            }

            if (pub.getNumSubscribers()) {
                sensor_msgs::Image msg;
                msg.header.stamp = ros::Time::now();
                msg.height = bgr.rows;
                msg.width = bgr.cols;
                msg.encoding = "bgr8";
                msg.is_bigendian = false;
                msg.step = bgr.step;
                msg.data = {bgr.data, bgr.data + bgr.total() * bgr.elemSize()};
                pub.publish(msg);
            }
            
            if (debugPub.getNumSubscribers()) {
                cv::aruco::drawDetectedMarkers(bgr, arucoCorners, arucoIds);
                cv::drawKeypoints(bgr, keypoints, bgr, {0, 0, 255}, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                sensor_msgs::Image debugMsg;
                debugMsg.header.stamp = ros::Time::now();
                debugMsg.height = bgr.rows;
                debugMsg.width = bgr.cols;
                debugMsg.encoding = "bgr8";
                debugMsg.is_bigendian = false;
                debugMsg.step = bgr.step;
                debugMsg.data = {bgr.data, bgr.data + bgr.total() * bgr.elemSize()};
                debugPub.publish(debugMsg);
            }

        } else {
            throw std::runtime_error{"Failed to grab frame"};
        }

        ros::spinOnce();
    }

    zed.close();
    return 0;
}
