#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <traj_utils/planning_visualization.h>

#include <plan_manage/plan_container.hpp>

namespace ego_planner
{

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class EGOPlannerManager
{
    // SECTION stable
   public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                        const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pos,
                        const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                 const Eigen::Vector3d& start_acc, const std::vector<Eigen::Vector3d>& waypoints,
                                 const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc);

    void initPlanModules(ros::NodeHandle& nh, PlanningVisualization::Ptr vis = NULL);
    bool isAStarOnly() const { return astar_only_; }
    bool isTrajectoryPointSafe(const Eigen::Vector3d& pos, Eigen::Vector3d* hit_pos = nullptr) const;

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

   private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continous_failures_count_{0};
    bool astar_only_{false};
    bool astar_test_wall_{false};
    std::string astar_test_scene_{"single"};
    double astar_height_{0.0};
    double astar_wall_x_{-13.5};
    double astar_wall_thickness_{0.4};
    double astar_wall_y_half_width_{9.0};
    double astar_wall_height_{0.35};
    double roll_over_height_{0.15};
    double max_jump_h_{0.6};
    bool direct_astar_jump_active_{false};
    double direct_astar_jump_hold_time_{0.0};
    bool optimize_direct_astar_{false};
    double direct_astar_sample_dist_{0.8};
    int direct_astar_smooth_iter_{2};
    double direct_astar_smooth_weight_{0.45};
    double direct_astar_jump_clearance_{0.25};
    double direct_astar_time_scale_{1.35};
    int astar_pool_size_x_{100};
    int astar_pool_size_y_{100};
    int astar_pool_size_z_{100};

    void updateTrajInfo(const UniformBspline& position_traj, const ros::Time time_now);
    bool isTrajectoryCollisionFree(UniformBspline& position_traj, double sample_step, Eigen::Vector3d* hit_pos = nullptr) const;

    void reparamBspline(UniformBspline& bspline, vector<Eigen::Vector3d>& start_end_derivative, double ratio,
                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc);

    bool refineTrajAlgo(UniformBspline& traj, vector<Eigen::Vector3d>& start_end_derivative, double ratio, double& ts,
                        Eigen::MatrixXd& optimal_control_points);

    // !SECTION stable

    // SECTION developing

   public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
};
}  // namespace ego_planner

#endif
