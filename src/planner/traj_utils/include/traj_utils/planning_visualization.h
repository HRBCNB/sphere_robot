#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <bspline_opt/uniform_bspline.h>
#include <path_searching/dyn_a_star.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <traj_utils/polynomial_traj.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>

using std::vector;
namespace ego_planner
{
class PlanningVisualization
{
   private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;
    ros::Publisher init_list_pub;
    ros::Publisher optimal_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher intermediate_state_pub;

   public:
    PlanningVisualization(/* args */)
    {
    }
    ~PlanningVisualization()
    {
    }
    PlanningVisualization(ros::NodeHandle& nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale,
                           Eigen::Vector4d color, int id);
    void generatePathDisplayArray(visualization_msgs::MarkerArray& array, const vector<Eigen::Vector3d>& list,
                                  double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray& array, const vector<Eigen::Vector3d>& list,
                                   double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayAStarList(std::vector<std::vector<PathNode>> a_star_paths, int id);
    void displayArrowList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale, Eigen::Vector4d color,
                          int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer,
    // double sleep_time, const int start_iteration); void displayNewArrow(ros::Publisher& guide_vector_pub,
    // ego_planner::BsplineOptimizer::Ptr optimizer);
};
}  // namespace ego_planner
#endif