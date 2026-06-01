#include <plan_manage/ego_replan_fsm.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace ego_planner;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego_planner_node");
    ros::NodeHandle nh("~");

    EGOReplanFSM rebo_replan;

    rebo_replan.init(nh);  // 从这里开始初始化

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
