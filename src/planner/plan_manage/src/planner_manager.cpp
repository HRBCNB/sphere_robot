// #include <fstream>
#include <plan_manage/planner_manager.h>

#include <algorithm>
#include <cmath>
#include <thread>

namespace ego_planner
{

// SECTION interfaces for setup and query

EGOPlannerManager::EGOPlannerManager()
{
}

EGOPlannerManager::~EGOPlannerManager()
{
    // std::cout << "des manager" << std::endl;
}

bool EGOPlannerManager::isTrajectoryPointSafe(const Eigen::Vector3d& pos, Eigen::Vector3d* hit_pos) const
{
    if (!grid_map_) return true;

    Eigen::Vector3d check_pos = pos;
    if (std::abs(pos.z() - astar_height_) <= 0.08)
    {
        check_pos.z() = astar_height_;
    }

    if (!grid_map_->getInflateOccupancy(check_pos)) return true;

    const double raw_top = grid_map_->getRawObstacleHeight(check_pos);
    const double inflated_top = grid_map_->getObstacleHeight(check_pos);
    const double obstacle_top = raw_top > 1e-3 ? raw_top : inflated_top;
    const bool roll_over_allowed = std::abs(check_pos.z() - astar_height_) <= 0.15 && obstacle_top > 0.0 &&
                                   obstacle_top <= astar_height_ + roll_over_height_ + 1e-3;
    const bool airborne_jump_clear = std::abs(check_pos.z() - astar_height_) > 0.15 &&
                                     obstacle_top > astar_height_ + roll_over_height_ + 1e-3 &&
                                     obstacle_top <= astar_height_ + max_jump_h_ + 0.05 &&
                                     !grid_map_->getOccupancy(check_pos);
    if (roll_over_allowed || airborne_jump_clear) return true;

    if (hit_pos) *hit_pos = pos;
    return false;
}

bool EGOPlannerManager::isTrajectoryCollisionFree(UniformBspline& position_traj, double sample_step, Eigen::Vector3d* hit_pos) const
{
    if (!grid_map_) return true;

    double t_start = 0.0;
    double t_end = 0.0;
    position_traj.getTimeSpan(t_start, t_end);
    const double dt = std::max(0.01, sample_step);

    for (double t = t_start; t <= t_end + 1e-6; t += dt)
    {
        const Eigen::Vector3d pos = position_traj.evaluateDeBoorT(std::min(t, t_end));
        if (!isTrajectoryPointSafe(pos, hit_pos)) return false;
    }

    return true;
}

void EGOPlannerManager::initPlanModules(ros::NodeHandle& nh, PlanningVisualization::Ptr vis)
{
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/astar_only", astar_only_, false);
    nh.param("manager/astar_test_wall", astar_test_wall_, false);
    nh.param("manager/astar_test_scene", astar_test_scene_, std::string("single"));
    nh.param("manager/astar_height", astar_height_, 0.0);
    nh.param("manager/astar_wall_x", astar_wall_x_, -13.5);
    nh.param("manager/astar_wall_thickness", astar_wall_thickness_, 0.4);
    nh.param("manager/astar_wall_y_half_width", astar_wall_y_half_width_, 9.0);
    nh.param("manager/astar_wall_height", astar_wall_height_, 0.35);
    nh.param("manager/optimize_direct_astar", optimize_direct_astar_, false);
    nh.param("manager/direct_astar_sample_dist", direct_astar_sample_dist_, 0.8);
    nh.param("manager/direct_astar_smooth_iter", direct_astar_smooth_iter_, 2);
    nh.param("manager/direct_astar_smooth_weight", direct_astar_smooth_weight_, 0.45);
    nh.param("manager/direct_astar_jump_clearance", direct_astar_jump_clearance_, 0.25);
    nh.param("manager/direct_astar_time_scale", direct_astar_time_scale_, 1.35);
    nh.param("manager/direct_astar_vel_dt_weight", direct_astar_vel_dt_weight_, 1.0);
    nh.param("manager/direct_astar_acc_dt_weight", direct_astar_acc_dt_weight_, 0.45);
    nh.param("manager/direct_astar_roll_sample_dist", direct_astar_roll_sample_dist_, 0.35);
    nh.param("manager/direct_astar_jump_sample_dist", direct_astar_jump_sample_dist_, 0.18);
    nh.param("manager/direct_astar_jump_anchor_repeat", direct_astar_jump_anchor_repeat_, 1);
    nh.param("manager/direct_astar_roll_anchor_repeat", direct_astar_roll_anchor_repeat_, 2);
    nh.param("manager/direct_astar_mode_time_allocation", direct_astar_mode_time_allocation_, true);
    nh.param("manager/direct_astar_roll_time_scale", direct_astar_roll_time_scale_, 1.4);
    nh.param("manager/direct_astar_jump_time_scale", direct_astar_jump_time_scale_, 1.0);
    nh.param("manager/direct_astar_jump_speed", direct_astar_jump_speed_, 2.5);
    nh.param("manager/direct_astar_min_dt", direct_astar_min_dt_, 0.04);
    nh.param("manager/direct_astar_time_realloc_max_iter", direct_astar_time_realloc_max_iter_, 1);
    nh.param("manager/direct_astar_allow_jump_impulse", direct_astar_allow_jump_impulse_, true);
    nh.param("manager/direct_astar_roll_interpolate_z", direct_astar_roll_interpolate_z_, false);
    nh.param("manager/astar_pool_size_x", astar_pool_size_x_, 100);
    nh.param("manager/astar_pool_size_y", astar_pool_size_y_, 100);
    nh.param("manager/astar_pool_size_z", astar_pool_size_z_, 100);
    nh.param("planner/roll_over_height", roll_over_height_, 0.15);
    nh.param("planner/max_jump_h", max_jump_h_, 0.6);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(
        grid_map_, Eigen::Vector3i(astar_pool_size_x_, astar_pool_size_y_, astar_pool_size_z_));
    bspline_optimizer_rebound_->a_star_->setJumpParams(nh);

    visualization_ = vis;
}

// !SECTION

// SECTION rebond replanning

bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                                      Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel,
                                      bool flag_polyInit, bool flag_randomPolyTraj)
{
    // 记录调用次数，便于调试重规划是否频繁触发。
    // static int count = 0;
    // std::cout << endl << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose()

    //      << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose() << endl;

    // 起点已经非常接近目标点时，直接认为没有必要继续规划。
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
        // cout << "Close to goal" << endl;
        continous_failures_count_++;
        return false;
    }

    if (direct_astar_jump_active_ && local_data_.duration_ > 1e-3)
    {
        const double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();
        if (t_cur >= 0.0 && t_cur < direct_astar_jump_hold_time_)
        {
            // ROS_INFO_THROTTLE(0.5, "[EGOPlannerManager] keep current direct A* jump traj, t=%.2f/%.2f",

            //                   t_cur, direct_astar_jump_hold_time_);
            continous_failures_count_ = 0;
            return true;
        }
        direct_astar_jump_active_ = false;
    }

    if (astar_only_)
    {
        Eigen::Vector3d astar_start = start_pt;
        Eigen::Vector3d astar_goal = local_target_pt;
        astar_start.z() = astar_height_;
        astar_goal.z() = astar_height_;

        if (astar_test_wall_)
        {
            const double res = 0.1;
            int occupied_points = 0;
            auto add_box = [&](double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) {
                for (double x = x_min; x <= x_max + 1e-6; x += res)
                {
                    for (double y = y_min; y <= y_max + 1e-6; y += res)
                    {
                        for (double z = z_min; z <= z_max + 1e-6; z += res)
                        {
                            const Eigen::Vector3d pos(x, y, z);
                            grid_map_->setOccupied(pos);
                            grid_map_->setOccupancy(pos, 1.0);
                            ++occupied_points;
                        }
                    }
                }
            };

            if (astar_test_scene_ == "complex")
            {
                add_box(-15.8, -15.4, -0.85, 0.25, 0.0, 0.30);
                add_box(-14.2, -13.8, -0.20, 0.95, 0.0, 0.35);
                add_box(-12.6, -12.2, -0.95, 0.15, 0.0, 0.30);
                add_box(-13.4, -13.0, 1.40, 2.20, 0.0, 0.45);
                add_box(-11.7, -11.3, -1.80, -1.05, 0.0, 0.45);
                // ROS_WARN("[EGOPlannerManager] astar_only inserted complex test obstacles, voxels=%d", occupied_points);
            }
            else if (astar_test_scene_ == "bend")
            {
                // Mixed demo scene along the start-goal line:
                // 1) wide very-low bump: should stay ROLL with roll-over cost.
                add_box(-16.75, -16.35, -0.95, 0.95, 0.0, 0.10);

                // 2) low jump wall across the local corridor: too high to roll over, low enough to JUMP.
                add_box(-15.35, -15.05, -0.85, 0.85, 0.0, 0.38);
                // Side guards make detouring this jump wall more expensive than a short jump.
                add_box(-15.55, -14.85, 1.10, 1.75, 0.0, 0.80);
                add_box(-15.55, -14.85, -1.75, -1.10, 0.0, 0.80);

                // 3) tall block on the center line: should be ROLL detour, not jump.
                add_box(-13.65, -13.00, -0.60, 0.60, 0.0, 1.20);

                // 4) second narrow low wall after the detour: should allow another JUMP after returning to the line.
                add_box(-11.70, -11.40, -0.65, 0.65, 0.0, 0.35);

                // Side clutter that makes the environment less empty while leaving visible detour corridors.
                add_box(-13.95, -13.45, 1.45, 2.20, 0.0, 0.55);
                add_box(-12.70, -12.10, -2.00, -1.20, 0.0, 0.55);
                add_box(-10.80, -10.25, 1.15, 1.85, 0.0, 0.40);
                // ROS_WARN("[EGOPlannerManager] astar_only inserted mixed roll/jump bend test obstacles, voxels=%d", occupied_points);
            }
            else if (astar_test_scene_ == "midterm_global")
            {
                add_box(-16.80, -16.35, -0.70, 0.70, 0.0, 0.10);
                add_box(-14.90, -14.50, -0.70, 0.70, 0.0, 0.34);
                add_box(-15.10, -14.30, 1.00, 1.70, 0.0, 0.75);
                add_box(-15.10, -14.30, -1.70, -1.00, 0.0, 0.75);
                add_box(-12.60, -11.70, -0.75, 0.75, 0.0, 1.20);
                add_box(-12.80, -11.50, 1.20, 2.00, 0.0, 0.75);
                add_box(-9.75, -9.35, -0.65, 0.65, 0.0, 0.32);
                add_box(-7.60, -6.95, 0.65, 1.55, 0.0, 0.85);
                add_box(-7.60, -6.95, -1.55, -0.65, 0.0, 0.85);
                add_box(-5.45, -5.05, -0.65, 0.65, 0.0, 0.30);
                add_box(-3.60, -3.15, 1.15, 1.85, 0.0, 0.45);
                add_box(-2.55, -2.10, -1.85, -1.15, 0.0, 0.45);
                // ROS_WARN("[EGOPlannerManager] astar_only inserted midterm_global demo obstacles, voxels=%d", occupied_points);
            }
            else if (astar_test_scene_ == "detour")
            {
                add_box(-16.8, -16.4, -0.45, 0.45, 0.0, 0.30);
                add_box(-14.7, -13.3, -0.65, 0.65, 0.0, 1.25);
                add_box(-11.8, -11.4, -0.45, 0.45, 0.0, 0.30);
                // ROS_WARN("[EGOPlannerManager] astar_only inserted detour test obstacles, voxels=%d", occupied_points);
            }
            else if (astar_test_scene_ == "roll_tracking")
            {
                // ROLL-only tracking demo: one central obstacle, so the reference is a clean single detour.
                add_box(-14.70, -13.30, -0.70, 0.70, 0.0, 1.20);
                add_box(-12.20, -11.70, 1.35, 1.95, 0.0, 0.45);
                add_box(-16.30, -15.90, 1.05, 1.65, 0.0, 0.45);
                add_box(-15.60, -15.15, -1.95, -1.30, 0.0, 0.50);
                add_box(-11.10, -10.70, -1.65, -1.05, 0.0, 0.45);
                add_box(-10.45, -10.05, 1.00, 1.55, 0.0, 0.40);
                add_box(-13.05, -12.65, 0.95, 1.35, 0.0, 0.35);
                add_box(-12.70, -12.25, -0.35, 0.35, 0.0, 0.10);
                add_box(-8.80, -8.25, 0.10, 0.95, 0.0, 0.85);
                add_box(-9.60, -9.15, -1.05, -0.20, 0.0, 0.80);
                add_box(-7.20, -6.75, -1.85, -1.10, 0.0, 0.50);
                add_box(-6.55, -6.05, 0.05, 0.90, 0.0, 0.85);
                add_box(-5.55, -5.05, 1.10, 1.80, 0.0, 0.55);
                add_box(-4.10, -3.65, -0.90, -0.25, 0.0, 0.75);
                add_box(-3.35, -2.90, 0.35, 1.05, 0.0, 0.70);
                add_box(-2.55, -2.10, 1.35, 1.95, 0.0, 0.45);
                add_box(-1.85, -1.45, -0.45, 0.30, 0.0, 0.65);
                add_box(-1.25, -0.85, -1.25, -0.70, 0.0, 0.40);
                // ROS_WARN("[EGOPlannerManager] astar_only inserted roll_tracking test obstacles, voxels=%d", occupied_points);
            }
            else
            {
                add_box(astar_wall_x_ - astar_wall_thickness_ * 0.5, astar_wall_x_ + astar_wall_thickness_ * 0.5,
                        -astar_wall_y_half_width_, astar_wall_y_half_width_, 0.0, astar_wall_height_);
                // ROS_WARN("[EGOPlannerManager] astar_only inserted test wall into occupancy: x=%.2f, y=[%.2f, %.2f], h=%.2f, voxels=%d",

                //          astar_wall_x_, -astar_wall_y_half_width_, astar_wall_y_half_width_, astar_wall_height_, occupied_points);
            }
        }

        vector<vector<PathNode>> a_star_pathes;
        if (bspline_optimizer_rebound_->a_star_->AstarSearch(0.1, astar_start, astar_goal))
        {
            a_star_pathes.push_back(bspline_optimizer_rebound_->a_star_->getPath());
        }
        else
        {
            ROS_ERROR("[EGOPlannerManager] astar_only direct A* search failed. projected start=(%.2f %.2f %.2f), goal=(%.2f %.2f %.2f)",
                      astar_start.x(), astar_start.y(), astar_start.z(), astar_goal.x(), astar_goal.y(), astar_goal.z());
        }

        int total_nodes = 0;
        int jump_nodes = 0;
        bool printed_first_jump = false;
        Eigen::Vector3d first_jump = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_jump = Eigen::Vector3d::Zero();
        for (const auto& path : a_star_pathes)
        {
            total_nodes += static_cast<int>(path.size());
            for (const auto& node : path)
            {
                if (node.mode == JUMP)
                {
                    ++jump_nodes;
                    if (!printed_first_jump)
                    {
                        first_jump = node.pos;
                        printed_first_jump = true;
                    }
                    last_jump = node.pos;
                }
            }
        }

        if (jump_nodes > 0)
        {
            // ROS_WARN("[EGOPlannerManager] A* jump span: first=(%.2f %.2f %.2f), last=(%.2f %.2f %.2f)",

            //          first_jump.x(), first_jump.y(), first_jump.z(), last_jump.x(), last_jump.y(), last_jump.z());
        }

        // for (size_t path_id = 0; path_id < a_star_pathes.size(); ++path_id)
        // {
        //     const auto& path = a_star_pathes[path_id];
        //     ROS_WARN("[EGOPlannerManager] A* path %zu points:", path_id);
        //     for (size_t node_id = 0; node_id < path.size(); ++node_id)
        //     {
        //         const auto& node = path[node_id];
        //         ROS_WARN("  [%03zu] %s  x=%.2f y=%.2f z=%.2f", node_id, node.mode == JUMP ? "JUMP" : "ROLL",
        //                  node.pos.x(), node.pos.y(), node.pos.z());
        //     }
        // }

        visualization_->displayAStarList(a_star_pathes, 0);
        ROS_WARN("[EGOPlannerManager] astar_only direct A*: paths=%zu, nodes=%d, jump_nodes=%d, z=%.2f. Skip bspline optimization and trajectory publishing.",
                 a_star_pathes.size(), total_nodes, jump_nodes, astar_height_);
        continous_failures_count_ = 0;
        return !a_star_pathes.empty();
    }

    // 统计本次重规划各阶段耗时。
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT - 生成初始参考路径和边界导数 ***/
    // ts 是初始采样步长，距离越近时会放大一点，避免初始路径过短。
    double ts = (start_pt - local_target_pt).norm() > 0.1
                    ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2
                    : pp_.ctrl_pt_dist / pp_.max_vel_ *
                          5;  // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    vector<TRAJ_MODE> point_modes;
    vector<vector<PathNode>> init_a_star_pathes;
    double direct_astar_jump_end_ratio = 0.0;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do  // 采样pointset
    {
        // 每次重试都重新生成初始点集和边界导数。
        point_set.clear();
        start_end_derivatives.clear();
        flag_regenerate = false;

        // 第一种初始化方式：直接生成一条多项式轨迹作为初始参考。
        if (flag_first_call || flag_polyInit ||
            flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/)  // Initial path generated from a
                                                                                       // min-snap traj by order.
        {
            flag_first_call = false;
            flag_force_polynomial = false;

            PolynomialTraj gl_traj;

            // 根据距离和最大速度/加速度，估算一段足够平滑的总时间。
            double dist = (start_pt - local_target_pt).norm();
            double time =
                pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist
                    ? sqrt(dist / pp_.max_acc_)
                    : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

            // 不随机时，直接连成单段多项式；随机时，在中间插一个扰动点。
            if (!flag_randomPolyTraj)
            {
                gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt,
                                                               local_target_vel, Eigen::Vector3d::Zero(), time);
            }
            else
            {
                // 用与起终点方向正交的两个方向生成随机中间点，增加初始路径多样性。
                Eigen::Vector3d horizen_dir =
                    ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
                Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
                Eigen::Vector3d random_inserted_pt =
                    (start_pt + local_target_pt) / 2 +
                    (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 *
                        (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                    (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 *
                        (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
                Eigen::MatrixXd pos(3, 3);
                pos.col(0) = start_pt;
                pos.col(1) = random_inserted_pt;
                pos.col(2) = local_target_pt;
                Eigen::VectorXd t(2);
                t(0) = t(1) = time / 2;
                gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc,
                                                      Eigen::Vector3d::Zero(), t);
            }

            // 沿初始轨迹采样，得到后续用于 B 样条参数化的点集。
            double t;
            bool flag_too_far;
            ts *= 1.5;  // ts will be divided by 1.5 in the next
            do
            {
                ts /= 1.5;
                point_set.clear();
                flag_too_far = false;
                Eigen::Vector3d last_pt = gl_traj.evaluate(0);
                for (t = 0; t < time; t += ts)
                {
                    Eigen::Vector3d pt = gl_traj.evaluate(t);
                    // 若相邻采样点过稀，则缩小步长重新采样。
                    if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
                    {
                        flag_too_far = true;
                        break;
                    }
                    last_pt = pt;
                    point_set.push_back(pt);
                }
            } while (flag_too_far || point_set.size() < 7);  // To make sure the initial path has enough points.
            t -= ts;
            start_end_derivatives.push_back(gl_traj.evaluateVel(0));
            start_end_derivatives.push_back(local_target_vel);
            start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
            start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
        }
        else  // 第二种初始化方式：沿上一条可行轨迹继续外推，复用已有规划结果。
        {
            double t;
            double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

            // 先把上一条轨迹转成伪弧长，再按弧长均匀抽样。
            vector<double> pseudo_arc_length;
            vector<Eigen::Vector3d> segment_point;
            pseudo_arc_length.push_back(0.0);
            for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
            {
                segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
                if (t > t_cur)
                {
                    pseudo_arc_length.push_back(
                        (segment_point.back() - segment_point[segment_point.size() - 2]).norm() +
                        pseudo_arc_length.back());
                }
            }
            t -= ts;

            double poly_time =
                (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
            if (poly_time > ts)
            {
                // 末端不足时，补一段多项式把当前位置接到目标点。
                PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
                    local_data_.position_traj_.evaluateDeBoorT(t), local_data_.velocity_traj_.evaluateDeBoorT(t),
                    local_data_.acceleration_traj_.evaluateDeBoorT(t), local_target_pt, local_target_vel,
                    Eigen::Vector3d::Zero(), poly_time);

                for (t = ts; t < poly_time; t += ts)
                {
                    if (!pseudo_arc_length.empty())
                    {
                        segment_point.push_back(gl_traj.evaluate(t));
                        pseudo_arc_length.push_back(
                            (segment_point.back() - segment_point[segment_point.size() - 2]).norm() +
                            pseudo_arc_length.back());
                    }
                    else
                    {
                        ROS_ERROR("pseudo_arc_length is empty, return!");
                        continous_failures_count_++;
                        return false;
                    }
                }
            }

            double sample_length = 0;
            double cps_dist = pp_.ctrl_pt_dist * 1.5;  // cps_dist will be divided by 1.5 in the next
            size_t id = 0;
            do
            {
                // 不断减小采样间隔，直到点数足够多。
                cps_dist /= 1.5;
                point_set.clear();
                sample_length = 0;
                id = 0;
                while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
                {
                    if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
                    {
                        point_set.push_back(
                            (sample_length - pseudo_arc_length[id]) /
                                (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                            (pseudo_arc_length[id + 1] - sample_length) /
                                (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
                        sample_length += cps_dist;
                    }
                    else
                        id++;
                }
                point_set.push_back(local_target_pt);
            } while (point_set.size() < 7);  // If the start point is very close to end point, this will help

            start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
            start_end_derivatives.push_back(local_target_vel);
            start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
            start_end_derivatives.push_back(Eigen::Vector3d::Zero());

            // 如果初始路径过长，强制回到多项式初始化重试一次。
            if (point_set.size() >
                pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3)  // The initial path is unnormally too long!
            {
                flag_force_polynomial = true;
                flag_regenerate = true;
            }
        }
    } while (flag_regenerate);

    {
        Eigen::Vector3d astar_start = start_pt;
        Eigen::Vector3d astar_goal = local_target_pt;
        astar_start.z() = astar_height_;
        astar_goal.z() = astar_height_;

        if (bspline_optimizer_rebound_->a_star_->AstarSearch(0.1, astar_start, astar_goal))
        {
            const vector<PathNode> raw_path = bspline_optimizer_rebound_->a_star_->getPath();
            vector<PathNode> sampled_path;
            const double sample_dist = std::max(0.05, direct_astar_sample_dist_);

            if (!raw_path.empty())
            {
                sampled_path.push_back(raw_path.front());
                double next_sample_dist = sample_dist;
                double accumulated_dist = 0.0;

                for (size_t i = 1; i < raw_path.size(); ++i)
                {
                    const Eigen::Vector3d seg_start = raw_path[i - 1].pos;
                    const Eigen::Vector3d seg_end = raw_path[i].pos;
                    const double seg_len = (seg_end - seg_start).norm();
                    const TRAJ_MODE seg_mode =
                        (raw_path[i - 1].mode == JUMP || raw_path[i].mode == JUMP) ? JUMP : raw_path[i].mode;

                    if (seg_len < 1e-6) continue;

                    while (next_sample_dist <= accumulated_dist + seg_len + 1e-6)
                    {
                        const double ratio = (next_sample_dist - accumulated_dist) / seg_len;
                        PathNode node;
                        node.pos = seg_start * (1.0 - ratio) + seg_end * ratio;
                        node.mode = seg_mode;
                        sampled_path.push_back(node);
                        next_sample_dist += sample_dist;
                    }

                    accumulated_dist += seg_len;
                }

                if ((sampled_path.back().pos - raw_path.back().pos).norm() > 1e-3)
                {
                    sampled_path.push_back(raw_path.back());
                }

                vector<PathNode> protected_path;
                protected_path.reserve(raw_path.size() * 3);
                protected_path.push_back(raw_path.front());

                const double roll_bspline_sample_dist = std::max(0.10, direct_astar_roll_sample_dist_);
                const double jump_bspline_sample_dist = std::max(0.05, direct_astar_jump_sample_dist_);
                for (size_t i = 1; i < raw_path.size(); ++i)
                {
                    const Eigen::Vector3d seg_start = raw_path[i - 1].pos;
                    const Eigen::Vector3d seg_end = raw_path[i].pos;
                    const double seg_len = (seg_end - seg_start).norm();
                    const TRAJ_MODE seg_mode =
                        (raw_path[i - 1].mode == JUMP || raw_path[i].mode == JUMP) ? JUMP : raw_path[i].mode;
                    const double protected_step = seg_mode == JUMP ? jump_bspline_sample_dist : roll_bspline_sample_dist;
                    const int sample_num = std::max(1, static_cast<int>(std::ceil(seg_len / protected_step)));

                    for (int sid = 1; sid <= sample_num; ++sid)
                    {
                        const double ratio = static_cast<double>(sid) / static_cast<double>(sample_num);
                        PathNode node;
                        node.pos = seg_start * (1.0 - ratio) + seg_end * ratio;
                        node.mode = seg_mode;
                        protected_path.push_back(node);
                    }
                }
                sampled_path.swap(protected_path);
            }

            if (sampled_path.size() >= 7)
            {
                point_set.clear();
                point_modes.clear();
                start_end_derivatives.clear();
                point_set.reserve(sampled_path.size());
                point_modes.reserve(sampled_path.size());

                for (size_t sample_id = 0; sample_id < sampled_path.size(); ++sample_id)
                {
                    const auto& node = sampled_path[sample_id];
                    Eigen::Vector3d pos = node.pos;
                    if (node.mode == ROLL)
                    {
                        if (direct_astar_roll_interpolate_z_ && sampled_path.size() > 1)
                        {
                            const double ratio = static_cast<double>(sample_id) /
                                                 static_cast<double>(sampled_path.size() - 1);
                            pos.z() = start_pt.z() * (1.0 - ratio) + local_target_pt.z() * ratio;
                        }
                        else
                        {
                            pos.z() = astar_height_;
                        }
                    }
                    point_set.push_back(pos);
                    point_modes.push_back(node.mode);
                }

                auto isHighInflatedObstacle = [&](const Eigen::Vector3d& pos) {
                    if (!grid_map_->getInflateOccupancy(pos))
                    {
                        return false;
                    }
                    const double raw_top = grid_map_->getRawObstacleHeight(pos);
                    const double inflated_top = grid_map_->getObstacleHeight(pos);
                    const double obstacle_top = raw_top > 1e-3 ? raw_top : inflated_top;
                    return obstacle_top > astar_height_ + roll_over_height_ + 1e-3;
                };

                const double map_res = grid_map_->getResolution();
                for (int iter = 0; iter < 3 && point_set.size() >= 3; ++iter)
                {
                    vector<Eigen::Vector3d> adjusted = point_set;
                    for (size_t i = 1; i + 1 < point_set.size(); ++i)
                    {
                        if (point_modes[i] != ROLL) continue;

                        Eigen::Vector3d pos = point_set[i];
                        pos.z() = astar_height_;
                        Eigen::Vector2d push = Eigen::Vector2d::Zero();

                        for (int ox = -2; ox <= 2; ++ox)
                        {
                            for (int oy = -2; oy <= 2; ++oy)
                            {
                                if (ox == 0 && oy == 0) continue;
                                Eigen::Vector3d probe = pos;
                                probe.x() += static_cast<double>(ox) * map_res;
                                probe.y() += static_cast<double>(oy) * map_res;
                                probe.z() = astar_height_;
                                if (!isHighInflatedObstacle(probe)) continue;

                                Eigen::Vector2d away(pos.x() - probe.x(), pos.y() - probe.y());
                                const double dist2 = std::max(away.squaredNorm(), 1e-4);
                                push += away / dist2;
                            }
                        }

                        if (push.norm() < 1e-6) continue;
                        const Eigen::Vector2d dir = push.normalized();
                        for (double step = map_res; step <= 3.0 * map_res + 1e-6; step += map_res)
                        {
                            Eigen::Vector3d candidate = pos;
                            candidate.x() += dir.x() * step;
                            candidate.y() += dir.y() * step;
                            candidate.z() = astar_height_;
                            if (!isHighInflatedObstacle(candidate))
                            {
                                adjusted[i] = candidate;
                                break;
                            }
                        }
                    }
                    point_set.swap(adjusted);
                }

                auto isRollPointSafe = [&](const Eigen::Vector3d& pos) {
                    if (!grid_map_->getInflateOccupancy(pos))
                    {
                        return true;
                    }

                    const double raw_top = grid_map_->getRawObstacleHeight(pos);
                    const double inflated_top = grid_map_->getObstacleHeight(pos);
                    const double obstacle_top = raw_top > 1e-3 ? raw_top : inflated_top;
                    return std::abs(pos.z() - astar_height_) <= 0.15 && obstacle_top > 0.0 &&
                           obstacle_top <= astar_height_ + roll_over_height_ + 1e-3;
                };

                auto isRollSegmentSafe = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
                    const double len = (p1 - p0).norm();
                    const double step = std::max(0.02, grid_map_->getResolution() * 0.5);
                    const int sample_num = std::max(1, static_cast<int>(std::ceil(len / step)));
                    for (int sid = 0; sid <= sample_num; ++sid)
                    {
                        const double ratio = static_cast<double>(sid) / static_cast<double>(sample_num);
                        Eigen::Vector3d pos = p0 * (1.0 - ratio) + p1 * ratio;
                        pos.z() = astar_height_;
                        if (!isRollPointSafe(pos))
                        {
                            return false;
                        }
                    }
                    return true;
                };

                const int smooth_iter = std::max(0, direct_astar_smooth_iter_);
                const double smooth_weight = std::max(0.0, std::min(0.8, direct_astar_smooth_weight_));
                for (int iter = 0; iter < smooth_iter && point_set.size() >= 3; ++iter)
                {
                    vector<Eigen::Vector3d> smoothed = point_set;
                    for (size_t i = 1; i + 1 < point_set.size(); ++i)
                    {
                        if (point_modes[i - 1] != ROLL || point_modes[i] != ROLL || point_modes[i + 1] != ROLL)
                        {
                            continue;
                        }

                        Eigen::Vector3d target = point_set[i] * (1.0 - smooth_weight) +
                                                 0.5 * smooth_weight * (point_set[i - 1] + point_set[i + 1]);
                        target.z() = astar_height_;
                        if (isRollSegmentSafe(smoothed[i - 1], target) && isRollSegmentSafe(target, smoothed[i + 1]))
                        {
                            smoothed[i] = target;
                        }
                    }
                    point_set.swap(smoothed);
                }

                if (point_set.size() >= 3)
                {
                    vector<Eigen::Vector3d> anchored_points;
                    vector<TRAJ_MODE> anchored_modes;
                    anchored_points.reserve(point_set.size() * 3);
                    anchored_modes.reserve(point_modes.size() * 3);

                    for (size_t i = 0; i < point_set.size(); ++i)
                    {
                        anchored_points.push_back(point_set[i]);
                        anchored_modes.push_back(point_modes[i]);

                        if (point_modes[i] == JUMP)
                        {
                            const int jump_anchor_repeat = std::max(0, direct_astar_jump_anchor_repeat_);
                            for (int repeat = 0; repeat < jump_anchor_repeat; ++repeat)
                            {
                                anchored_points.push_back(point_set[i]);
                                anchored_modes.push_back(point_modes[i]);
                            }
                            continue;
                        }

                        if (i == 0 || i + 1 >= point_set.size()) continue;
                        if (point_modes[i - 1] != ROLL || point_modes[i] != ROLL || point_modes[i + 1] != ROLL)
                        {
                            continue;
                        }

                        Eigen::Vector3d prev_dir = point_set[i] - point_set[i - 1];
                        Eigen::Vector3d next_dir = point_set[i + 1] - point_set[i];
                        prev_dir.z() = 0.0;
                        next_dir.z() = 0.0;
                        if (prev_dir.norm() < 1e-4 || next_dir.norm() < 1e-4) continue;

                        const double turn_cos = prev_dir.normalized().dot(next_dir.normalized());
                        if (turn_cos < 0.98)
                        {
                            const int roll_anchor_repeat = std::max(0, direct_astar_roll_anchor_repeat_);
                            for (int repeat = 0; repeat < roll_anchor_repeat; ++repeat)
                            {
                                anchored_points.push_back(point_set[i]);
                                anchored_modes.push_back(point_modes[i]);
                            }
                        }
                    }

                    point_set.swap(anchored_points);
                    point_modes.swap(anchored_modes);
                }

                Eigen::Vector3d direct_start_vel = start_vel;
                Eigen::Vector3d direct_target_vel = local_target_vel;
                Eigen::Vector3d direct_start_acc = start_acc;
                direct_start_vel.z() = 0.0;
                direct_target_vel.z() = 0.0;
                direct_start_acc.z() = 0.0;
                start_end_derivatives.push_back(direct_start_vel);
                start_end_derivatives.push_back(direct_target_vel);
                start_end_derivatives.push_back(direct_start_acc);
                start_end_derivatives.push_back(Eigen::Vector3d::Zero());
                init_a_star_pathes.push_back(raw_path);

                const bool has_jump = std::any_of(point_modes.begin(), point_modes.end(), [](const TRAJ_MODE mode) {
                    return mode == JUMP;
                });
                if (has_jump && point_modes.size() > 1)
                {
                    auto last_jump_it = std::find(point_modes.rbegin(), point_modes.rend(), JUMP);
                    const size_t last_jump_id = point_modes.size() - 1 - std::distance(point_modes.rbegin(), last_jump_it);
                    direct_astar_jump_end_ratio = static_cast<double>(last_jump_id) /
                                                  static_cast<double>(point_modes.size() - 1);
                }
                
                // max_sample_gap 表示 direct A* 采样点之间的最大几何距离。
                // 后面会用它估算 uniform B-spline 的参数时间间隔 ts。
                // 注意：这里的 ts 是 B-spline 参数时间间隔；在原始 EGO-Planner 中，traj_server 会直接把它当作物理执行时间使用。
                double max_sample_gap = 0.0;
                size_t max_gap_id = 0;
                constexpr int kDebugTopGapNum = 5;
                double top_gaps[kDebugTopGapNum] = {0.0, 0.0, 0.0, 0.0, 0.0};
                size_t top_gap_ids[kDebugTopGapNum] = {0, 0, 0, 0, 0};
                for (size_t i = 1; i < point_set.size(); ++i)
                {
                    const double gap = (point_set[i] - point_set[i - 1]).norm();
                    if (gap > max_sample_gap)
                    {
                        max_sample_gap = gap;
                        max_gap_id = i;
                    }
                    for (int rank = 0; rank < kDebugTopGapNum; ++rank)
                    {
                        if (gap > top_gaps[rank])
                        {
                            for (int j = kDebugTopGapNum - 1; j > rank; --j)
                            {
                                top_gaps[j] = top_gaps[j - 1];
                                top_gap_ids[j] = top_gap_ids[j - 1];
                            }
                            top_gaps[rank] = gap;
                            top_gap_ids[rank] = i;
                            break;
                        }
                    }
                }

                // vel_dt：从速度约束出发估算的最小时间间隔。
                // 近似含义：相邻采样点最大距离 / 最大速度。
                // direct_astar_vel_dt_weight_ 是人为系数，越大轨迹越慢，越小轨迹越快。
                const double vel_dt_weight = std::max(0.1, direct_astar_vel_dt_weight_);

                // acc_dt：从加速度约束出发估算的最小时间间隔。
                // 来源近似是 s = 0.5 * a * t^2，因此 t = sqrt(2s/a)。
                // 这一项通常比 vel_dt 更保守，可能把整条轨迹时间轴拉长。
                const double acc_dt_weight = std::max(0.1, direct_astar_acc_dt_weight_);
                const double vel_dt = pp_.max_vel_ > 1e-3 ? vel_dt_weight * max_sample_gap / pp_.max_vel_ : ts;
                const double acc_dt = pp_.max_acc_ > 1e-3 ? acc_dt_weight * std::sqrt(2.0 * max_sample_gap / pp_.max_acc_) : ts;

                // direct_astar_ts 是最终用于整条 direct A* B-spline 的统一参数时间间隔。
                // 这里取 max(ts, vel_dt, acc_dt)，所以只要某一项很大，整条轨迹都会变慢。
                // 这也是当前中期版本的局限：ROLL/JUMP 仍然共用一个 uniform ts，没有真正分段时间分配。
                const double direct_astar_ts = std::max(ts, std::max(vel_dt, acc_dt)) * (has_jump ? direct_astar_time_scale_ : 1.0);
                const double selected_dt_before_scale = std::max(ts, std::max(vel_dt, acc_dt));
                const char* selected_dt_source = "base_ts";
                if (vel_dt >= ts && vel_dt >= acc_dt)
                {
                    selected_dt_source = "vel_dt";
                }
                else if (acc_dt >= ts && acc_dt >= vel_dt)
                {
                    selected_dt_source = "acc_dt";
                }

                const TRAJ_MODE max_gap_mode =
                    (max_gap_id > 0 && max_gap_id < point_modes.size() &&
                     (point_modes[max_gap_id - 1] == JUMP || point_modes[max_gap_id] == JUMP))
                        ? JUMP
                        : ROLL;
                const Eigen::Vector3d max_gap_start =
                    max_gap_id > 0 ? point_set[max_gap_id - 1] : Eigen::Vector3d::Zero();
                const Eigen::Vector3d max_gap_end =
                    max_gap_id < point_set.size() ? point_set[max_gap_id] : Eigen::Vector3d::Zero();
                ROS_WARN("[EGOPlannerManager] direct A* ts debug: base_ts=%.3fs, vel_dt=%.3fs, acc_dt=%.3fs, selected=%s %.3fs, time_scale=%.3f, final_ts=%.3fs",
                         ts, vel_dt, acc_dt, selected_dt_source, selected_dt_before_scale,
                         has_jump ? direct_astar_time_scale_ : 1.0, direct_astar_ts);
                ROS_WARN("[EGOPlannerManager] direct A* max gap: id=%zu/%zu, mode=%s, gap=%.3fm, p0=(%.2f %.2f %.2f), p1=(%.2f %.2f %.2f), max_vel=%.2f, max_acc=%.2f",
                         max_gap_id, point_set.size(), max_gap_mode == JUMP ? "JUMP" : "ROLL", max_sample_gap,
                         max_gap_start.x(), max_gap_start.y(), max_gap_start.z(),
                         max_gap_end.x(), max_gap_end.y(), max_gap_end.z(), pp_.max_vel_, pp_.max_acc_);
                for (int rank = 0; rank < kDebugTopGapNum; ++rank)
                {
                    if (top_gaps[rank] <= 1e-6) continue;
                    const size_t gap_id = top_gap_ids[rank];
                    const double gap_vel_dt = pp_.max_vel_ > 1e-3 ? vel_dt_weight * top_gaps[rank] / pp_.max_vel_ : ts;
                    const double gap_acc_dt = pp_.max_acc_ > 1e-3 ? acc_dt_weight * std::sqrt(2.0 * top_gaps[rank] / pp_.max_acc_) : ts;
                    const char* gap_selected = "base_ts";
                    if (gap_vel_dt >= ts && gap_vel_dt >= gap_acc_dt)
                        gap_selected = "vel_dt";
                    else if (gap_acc_dt >= ts && gap_acc_dt >= gap_vel_dt)
                        gap_selected = "acc_dt";
                    const TRAJ_MODE gap_mode =
                        (gap_id > 0 && gap_id < point_modes.size() &&
                         (point_modes[gap_id - 1] == JUMP || point_modes[gap_id] == JUMP))
                            ? JUMP
                            : ROLL;
                    const Eigen::Vector3d gap_start = gap_id > 0 ? point_set[gap_id - 1] : Eigen::Vector3d::Zero();
                    const Eigen::Vector3d gap_end = gap_id < point_set.size() ? point_set[gap_id] : Eigen::Vector3d::Zero();
                    ROS_WARN("[EGOPlannerManager] direct A* gap rank %d: id=%zu/%zu, mode=%s, gap=%.3fm, base_ts=%.3fs, vel_dt=%.3fs, acc_dt=%.3fs, selected=%s, p0=(%.2f %.2f %.2f), p1=(%.2f %.2f %.2f)",
                             rank + 1, gap_id, point_set.size(), gap_mode == JUMP ? "JUMP" : "ROLL", top_gaps[rank],
                             ts, gap_vel_dt, gap_acc_dt, gap_selected,
                             gap_start.x(), gap_start.y(), gap_start.z(),
                             gap_end.x(), gap_end.y(), gap_end.z());
                }
                if (direct_astar_ts > ts)
                {
                    // ROS_INFO("[EGOPlannerManager] direct A* init: enlarge ts %.3f -> %.3f for dynamic feasibility, max_gap=%.3f",

                    //          ts, direct_astar_ts, max_sample_gap);
                    ts = direct_astar_ts;
                }

                // ROS_INFO("[EGOPlannerManager] use direct A* init path: raw_points=%zu, sampled_points=%zu, sample_dist=%.2f, mode=%s, jump_end=%.2f",


                //          raw_path.size(), point_set.size(), sample_dist, has_jump ? "JUMP" : "ROLL",


                //          direct_astar_jump_end_ratio);
            }
            else
            {
                // ROS_WARN("[EGOPlannerManager] direct A* init path has too few sampled points (%zu), keep polynomial init.",

                //          sampled_path.size());
            }
        }
        else
        {
            // ROS_WARN("[EGOPlannerManager] direct A* init failed, keep polynomial init.");
        }
    }

    // 将离散点集和起终点导数参数化为 B 样条控制点control points。
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // 通过Astar来使轨迹无碰撞。若已拿到 direct A* 初始路径，则直接使用该路径参数化出的控制点，
    // 避免旧的碰撞段 A* 再次用可能越界的控制点端点搜索。
    vector<vector<PathNode>> a_star_pathes;
    vector<TRAJ_MODE> direct_ctrl_point_modes;
    const bool use_direct_astar_init = !init_a_star_pathes.empty() && !point_modes.empty();
    if (use_direct_astar_init)
    {
        bspline_optimizer_rebound_->setControlPoints(ctrl_pts);
        a_star_pathes = init_a_star_pathes;
    }
    else
    {
        a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);
    }

    if (!point_modes.empty() && point_modes.size() >= 2)
    {
        vector<TRAJ_MODE> ctrl_point_modes(ctrl_pts.cols(), ROLL);
        const double denom = static_cast<double>(point_modes.size() - 1);
        const int ctrl_last = static_cast<int>(ctrl_pts.cols()) - 1;

        for (size_t i = 0; i < point_modes.size(); ++i)
        {
            if (point_modes[i] != JUMP) continue;

            const int ctrl_id = static_cast<int>(std::round(static_cast<double>(i) / denom * ctrl_last));
            for (int offset = -6; offset <= 6; ++offset)
            {
                const int id = ctrl_id + offset;
                if (id >= 0 && id <= ctrl_last)
                {
                    ctrl_point_modes[id] = JUMP;
                }
            }
        }
        if (use_direct_astar_init)
        {
            for (int i = 0; i < ctrl_pts.cols(); ++i)
            {
                if (ctrl_point_modes[i] == ROLL)
                {
                    if (direct_astar_roll_interpolate_z_ && ctrl_pts.cols() > 1)
                    {
                        const double ratio = static_cast<double>(i) / static_cast<double>(ctrl_pts.cols() - 1);
                        ctrl_pts(2, i) = start_pt.z() * (1.0 - ratio) + local_target_pt.z() * ratio;
                    }
                    else
                    {
                        ctrl_pts(2, i) = astar_height_;
                    }
                }
            }
            bspline_optimizer_rebound_->setControlPoints(ctrl_pts);
        }
        bspline_optimizer_rebound_->setControlPointModes(ctrl_point_modes);
        direct_ctrl_point_modes = ctrl_point_modes;
    }

    // 记录初始化阶段耗时，并显示初始路径和 A* 路径。
    t_init = ros::Time::now() - t_start;

    if (!astar_only_)
    {
        visualization_->displayInitPathList(point_set, 0.2, 0);
    }
    visualization_->displayAStarList(a_star_pathes, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE - 对 B 样条控制点做避障和平滑优化 ***/
    bool flag_step_1_success = true;
    if (use_direct_astar_init && !optimize_direct_astar_)
    {
        // ROS_INFO("[EGOPlannerManager] direct A* init: skip rebound optimizer and keep A* B-spline control points.");
    }
    else
    {
        flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    }
    // cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
        // visualization_->displayOptimalList( ctrl_pts, vis_id );
        continous_failures_count_++;
        return false;
    }
    // visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY - 检查动力学约束 ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);

    if (use_direct_astar_init && direct_astar_mode_time_allocation_ &&
        direct_ctrl_point_modes.size() == static_cast<size_t>(ctrl_pts.cols()) && ctrl_pts.cols() >= 2)
    {
        const int order = 3;
        const int ctrl_num = ctrl_pts.cols();
        const int knot_num = ctrl_num + order + 1;
        Eigen::VectorXd non_uniform_knots(knot_num);

        vector<double> ctrl_dt(ctrl_num - 1, ts);
        const double min_dt = std::max(0.01, direct_astar_min_dt_);
        const double roll_scale = std::max(0.1, direct_astar_roll_time_scale_);
        const double jump_scale = std::max(0.1, direct_astar_jump_time_scale_);
        const double jump_speed = std::max(0.2, direct_astar_jump_speed_);

        for (int i = 0; i + 1 < ctrl_num; ++i)
        {
            const double dist = (ctrl_pts.col(i + 1) - ctrl_pts.col(i)).norm();
            const bool jump_interval = direct_ctrl_point_modes[i] == JUMP || direct_ctrl_point_modes[i + 1] == JUMP;
            if (jump_interval)
            {
                ctrl_dt[i] = std::max(min_dt, jump_scale * dist / jump_speed);
            }
            else
            {
                ctrl_dt[i] = std::max(min_dt, roll_scale * dist / std::max(0.1, pp_.max_vel_));
            }
        }

        const double first_dt = ctrl_dt.empty() ? ts : ctrl_dt.front();
        non_uniform_knots(0) = -order * first_dt;
        for (int i = 1; i < knot_num; ++i)
        {
            double dt = first_dt;
            if (i > order)
            {
                const int dt_id = std::min(static_cast<int>(ctrl_dt.size()) - 1, std::max(0, i - order - 1));
                dt = ctrl_dt[dt_id];
            }
            non_uniform_knots(i) = non_uniform_knots(i - 1) + dt;
        }
        pos.setKnot(non_uniform_knots);
    }

    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    const bool direct_astar_has_jump =
        use_direct_astar_init &&
        std::any_of(direct_ctrl_point_modes.begin(), direct_ctrl_point_modes.end(), [](const TRAJ_MODE mode) {
            return mode == JUMP;
        });

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
        // 若速度/加速度超限，就通过拉长时间来重新分配轨迹参数。
        // 这里的 ratio 来自 UniformBspline::checkFeasibility：
        // ratio = max(最大速度/速度上限, sqrt(最大加速度/加速度上限))。
        // ratio 越大，说明当前样条越不满足动力学约束，后面的 lengthenTime 会把整条时间轴拉长。
        ROS_WARN("[EGOPlannerManager] feasibility failed before reallocation: duration=%.2fs, ratio=%.3f, direct_astar=%s",
                 pos.getTimeSum(), ratio, use_direct_astar_init ? "true" : "false");

        if (use_direct_astar_init && direct_astar_allow_jump_impulse_ && direct_astar_has_jump)
        {
            // 跳跃球的 JUMP 段由弹性机构释放能量，起跳/落地附近允许出现冲量型高加速度。
            // 因此这里不再用无人机连续加速度约束 lengthenTime，否则会把整条 ROLL+JUMP 轨迹拉得过慢。
            ROS_WARN("[EGOPlannerManager] direct A* jump impulse accepted: skip feasibility time reallocation, duration=%.2fs, ratio=%.3f",
                     pos.getTimeSum(), ratio);
        }
        else if (use_direct_astar_init)
        {
            const int max_time_scale_num = std::max(0, direct_astar_time_realloc_max_iter_);
            bool direct_feasible = false;

            for (int scale_id = 0; scale_id < max_time_scale_num; ++scale_id)
            {
                const double duration_before = pos.getTimeSum();
                const double scale_ratio = std::max(1.01, ratio * 1.05);
                pos.lengthenTime(scale_ratio);
                const double duration_after = pos.getTimeSum();
                ROS_WARN("[EGOPlannerManager] direct A* lengthenTime iter=%d, scale_ratio=%.3f, duration %.2fs -> %.2fs",
                         scale_id + 1, scale_ratio, duration_before, duration_after);

                if (pos.checkFeasibility(ratio, false))
                {
                    direct_feasible = true;
                    ROS_WARN("[EGOPlannerManager] direct A* feasible after lengthenTime: iter=%d, duration=%.2fs, next_ratio=%.3f",
                             scale_id + 1, pos.getTimeSum(), ratio);
                    break;
                }
            }

            if (!direct_feasible)
            {
                // 中期展示采用 direct A* 全局轨迹时，允许在限定重分配轮数后继续发布。
                // 这样不会为了满足无人机模型的严格加速度检查，把整条跳跃球轨迹拉到过长。
                ROS_WARN("[EGOPlannerManager] direct A* bounded time reallocation accepted: iter=%d, duration=%.2fs, remaining_ratio=%.3f",
                         max_time_scale_num, pos.getTimeSum(), ratio);
            }
        }
        else
        {
            Eigen::MatrixXd optimal_control_points;
            // 时间重分配
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
            if (flag_step_2_success) pos = UniformBspline(optimal_control_points, 3, ts);
        }
    }

    if (!flag_step_2_success)
    {
        // printf(

        //     "\033[34mThis refined trajectory hits obstacles. It does not matter if appeares occasionally. But if "

        //     "continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
    }

    Eigen::Vector3d hit_pos = Eigen::Vector3d::Zero();
    const double collision_check_dt = std::max(0.01, std::min(0.05, ts * 0.2));

    if (!isTrajectoryCollisionFree(pos, collision_check_dt, &hit_pos))
    {
        ROS_WARN("[EGOPlannerManager] final B-spline hits inflated obstacle, reject traj. hit=(%.2f, %.2f, %.2f), dt=%.3f",
                 hit_pos.x(), hit_pos.y(), hit_pos.z(), collision_check_dt);
        static int rejected_bspline_vis_id = 0;
        if (visualization_)
        {
            visualization_->displayBsplineTrajectory(pos, 0.05, 9000 + rejected_bspline_vis_id++);
        }
        continous_failures_count_++;
        return false;
    }

    t_refine = ros::Time::now() - t_start;

    // 保存最终轨迹，供后续跟踪控制器使用。
    updateTrajInfo(pos, ros::Time::now());
    direct_astar_jump_active_ = use_direct_astar_init && direct_astar_jump_end_ratio > 0.0;
    if (direct_astar_jump_active_)
    {
        direct_astar_jump_hold_time_ = std::min(local_data_.duration_,
                                                local_data_.duration_ * direct_astar_jump_end_ratio + 0.4);
        // ROS_INFO("[EGOPlannerManager] hold direct A* jump traj until %.2fs, duration=%.2fs",

        //          direct_astar_jump_hold_time_, local_data_.duration_);
    }

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec()


    //      << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
}

bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
{
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
        control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
}

bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                                const Eigen::Vector3d& start_acc,
                                                const std::vector<Eigen::Vector3d>& waypoints,
                                                const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc)
{
    // generate global reference trajectory

    vector<Eigen::Vector3d> points;  // 存放全局轨迹 起始点+中间点+终点
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
        points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
        total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        inter_points.push_back(points.at(i));
        double dist = (points.at(i + 1) - points.at(i)).norm();

        if (dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;

            for (int j = 1; j < id_num; ++j)
            {
                Eigen::Vector3d inter_pt =
                    points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                inter_points.push_back(inter_pt);  // 在中间插点
            }
        }
    }

    inter_points.push_back(points.back());  // 终点

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix 位置矩阵
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i) pos.col(i) = inter_points[i];

    // 为每一段分配速度
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
        time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // 定义多项式轨迹，通过minisnap求解
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
        gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
        gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc,
                                                       time(0));
    else
        return false;

    auto time_now = ros::Time::now();
    // 传入全局轨迹的时间
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
}

bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                       const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pos,
                                       const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc)
{
    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        inter_points.push_back(points.at(i));
        double dist = (points.at(i + 1) - points.at(i)).norm();

        if (dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;

            for (int j = 1; j < id_num; ++j)
            {
                Eigen::Vector3d inter_pt =
                    points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                inter_points.push_back(inter_pt);
            }
        }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i) pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
        time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
        gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
        gl_traj =
            PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
        return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
}

bool EGOPlannerManager::refineTrajAlgo(UniformBspline& traj, vector<Eigen::Vector3d>& start_end_derivative,
                                       double ratio, double& ts, Eigen::MatrixXd& optimal_control_points)
{
    double t_inc;

    Eigen::MatrixXd ctrl_pts;  // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    // 得到控制点
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
        bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    // b样条轨迹重优化
    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
}

void EGOPlannerManager::updateTrajInfo(const UniformBspline& position_traj, const ros::Time time_now)
{
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();

    double max_vel = 0.0;
    double mean_vel = 0.0;
    int sample_num = 0;
    const double dt = std::max(0.02, std::min(0.10, local_data_.duration_ / 200.0));
    for (double t = 0.0; t <= local_data_.duration_ + 1e-6; t += dt)
    {
        const double v = local_data_.velocity_traj_.evaluateDeBoorT(std::min(t, local_data_.duration_)).norm();
        max_vel = std::max(max_vel, v);
        mean_vel += v;
        ++sample_num;
    }
    if (sample_num > 0) mean_vel /= static_cast<double>(sample_num);
    ROS_INFO("[EGOPlannerManager] traj speed stats: duration=%.2fs, mean_vel=%.2fm/s, max_vel=%.2fm/s, limit=%.2fm/s",
             local_data_.duration_, mean_vel, max_vel, pp_.max_vel_);

    local_data_.traj_id_ += 1;
}

void EGOPlannerManager::reparamBspline(UniformBspline& bspline, vector<Eigen::Vector3d>& start_end_derivative,
                                       double ratio, Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc)
{
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
        point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
}

}  // namespace ego_planner
