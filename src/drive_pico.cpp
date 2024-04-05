#include <ros/node_handle.h>

#include <rc/

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "drive_pico");
    ros::NodeHandle nh;

    ros::spin();

    return EXIT_SUCCESS;
}