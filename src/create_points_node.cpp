#include <points_global_planner/create_points.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "create_points");
    ros::NodeHandle nh("~");
    CreatePoints listener(nh);
    ros::spin();
    return 0;
}
