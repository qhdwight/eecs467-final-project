#include <ros/init.h>
#include <ros/node_handle.h>

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "mppi");
    ros::NodeHandle pnh{"~"};
    ros::NodeHandle nh;

    ros::Rate rate{30};
    while (ros::ok()) {

        rate.sleep();
    }

    return EXIT_SUCCESS;
}