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
    std::cout << "des manager" << std::endl;
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
    nh.param("manager/astar_height", astar_height_, 0.25);
    nh.param("manager/astar_wall_x", astar_wall_x_, -13.5);
    nh.param("manager/astar_wall_thickness", astar_wall_thickness_, 0.4);
    nh.param("manager/astar_wall_y_half_width", astar_wall_y_half_width_, 9.0);
    nh.param("manager/astar_wall_height", astar_wall_height_, 0.35);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
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
    static int count = 0;
    std::cout << endl << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose()
         << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose() << endl;

    // 起点已经非常接近目标点时，直接认为没有必要继续规划。
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
        cout << "Close to goal" << endl;
        continous_failures_count_++;
        return false;
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
                ROS_WARN("[EGOPlannerManager] astar_only inserted complex test obstacles, voxels=%d", occupied_points);
            }
            else if (astar_test_scene_ == "bend")
            {
                add_box(-16.0, -15.6, -0.45, 0.55, 0.0, 0.30);
                add_box(-14.45, -13.85, -0.45, 0.25, 0.0, 1.10);
                add_box(-12.6, -12.2, -0.15, 0.85, 0.0, 0.30);
                add_box(-13.5, -12.8, 1.35, 2.20, 0.0, 0.45);
                add_box(-11.8, -11.2, -1.80, -1.10, 0.0, 0.45);
                ROS_WARN("[EGOPlannerManager] astar_only inserted bend test obstacles, voxels=%d", occupied_points);
            }
            else if (astar_test_scene_ == "detour")
            {
                add_box(-16.8, -16.4, -0.45, 0.45, 0.0, 0.30);
                add_box(-14.7, -13.3, -0.65, 0.65, 0.0, 1.25);
                add_box(-11.8, -11.4, -0.45, 0.45, 0.0, 0.30);
                ROS_WARN("[EGOPlannerManager] astar_only inserted detour test obstacles, voxels=%d", occupied_points);
            }
            else
            {
                add_box(astar_wall_x_ - astar_wall_thickness_ * 0.5, astar_wall_x_ + astar_wall_thickness_ * 0.5,
                        -astar_wall_y_half_width_, astar_wall_y_half_width_, 0.0, astar_wall_height_);
                ROS_WARN("[EGOPlannerManager] astar_only inserted test wall into occupancy: x=%.2f, y=[%.2f, %.2f], h=%.2f, voxels=%d",
                         astar_wall_x_, -astar_wall_y_half_width_, astar_wall_y_half_width_, astar_wall_height_, occupied_points);
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
            ROS_WARN("[EGOPlannerManager] A* jump span: first=(%.2f %.2f %.2f), last=(%.2f %.2f %.2f)",
                     first_jump.x(), first_jump.y(), first_jump.z(), last_jump.x(), last_jump.y(), last_jump.z());
        }

        for (size_t path_id = 0; path_id < a_star_pathes.size(); ++path_id)
        {
            const auto& path = a_star_pathes[path_id];
            ROS_WARN("[EGOPlannerManager] A* path %zu points:", path_id);
            for (size_t node_id = 0; node_id < path.size(); ++node_id)
            {
                const auto& node = path[node_id];
                ROS_WARN("  [%03zu] %s  x=%.2f y=%.2f z=%.2f", node_id, node.mode == JUMP ? "JUMP" : "ROLL",
                         node.pos.x(), node.pos.y(), node.pos.z());
            }
        }

        visualization_->displayAStarList(a_star_pathes, 0);
        ROS_WARN("[EGOPlannerManager] astar_only direct A*: paths=%zu, nodes=%d, jump_nodes=%d, z=%.2f. Skip bspline optimization and trajectory publishing.",
                 a_star_pathes.size(), total_nodes, jump_nodes, astar_height_);
        continous_failures_count_ = 0;
        return false;
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
            vector<PathNode> dense_path;
            const double max_step = std::max(0.05, pp_.ctrl_pt_dist);

            for (size_t i = 0; i < raw_path.size(); ++i)
            {
                if (i == 0)
                {
                    dense_path.push_back(raw_path[i]);
                    continue;
                }

                const Eigen::Vector3d last_pos = raw_path[i - 1].pos;
                const Eigen::Vector3d cur_pos = raw_path[i].pos;
                const double dist = (cur_pos - last_pos).norm();
                const int sample_num = std::max(1, static_cast<int>(std::ceil(dist / max_step)));
                const TRAJ_MODE seg_mode =
                    (raw_path[i - 1].mode == JUMP || raw_path[i].mode == JUMP) ? JUMP : raw_path[i].mode;

                for (int sample = 1; sample <= sample_num; ++sample)
                {
                    const double ratio = static_cast<double>(sample) / sample_num;
                    PathNode node;
                    node.pos = last_pos * (1.0 - ratio) + cur_pos * ratio;
                    node.mode = seg_mode;
                    dense_path.push_back(node);
                }
            }

            if (dense_path.size() >= 7)
            {
                point_set.clear();
                point_modes.clear();
                start_end_derivatives.clear();
                point_set.reserve(dense_path.size());
                point_modes.reserve(dense_path.size());

                for (const auto& node : dense_path)
                {
                    point_set.push_back(node.pos);
                    point_modes.push_back(node.mode);
                }

                start_end_derivatives.push_back(start_vel);
                start_end_derivatives.push_back(local_target_vel);
                start_end_derivatives.push_back(start_acc);
                start_end_derivatives.push_back(Eigen::Vector3d::Zero());
                init_a_star_pathes.push_back(dense_path);

                const bool has_jump = std::any_of(point_modes.begin(), point_modes.end(), [](const TRAJ_MODE mode) {
                    return mode == JUMP;
                });
                ROS_INFO("[EGOPlannerManager] use direct A* init path: points=%zu, mode=%s", point_set.size(),
                         has_jump ? "JUMP" : "ROLL");
            }
            else
            {
                ROS_WARN("[EGOPlannerManager] direct A* init path has too few points (%zu), keep polynomial init.",
                         dense_path.size());
            }
        }
        else
        {
            ROS_WARN("[EGOPlannerManager] direct A* init failed, keep polynomial init.");
        }
    }

    // 将离散点集和起终点导数参数化为 B 样条控制点control points。
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // 通过Astar来使轨迹无碰撞
    vector<vector<PathNode>> a_star_pathes;
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    if (!point_modes.empty() && point_modes.size() >= 2)
    {
        vector<TRAJ_MODE> ctrl_point_modes(ctrl_pts.cols(), ROLL);
        const double denom = static_cast<double>(point_modes.size() - 1);
        const int ctrl_last = static_cast<int>(ctrl_pts.cols()) - 1;

        for (size_t i = 0; i < point_modes.size(); ++i)
        {
            if (point_modes[i] != JUMP) continue;

            const int ctrl_id = static_cast<int>(std::round(static_cast<double>(i) / denom * ctrl_last));
            for (int offset = -1; offset <= 1; ++offset)
            {
                const int id = ctrl_id + offset;
                if (id >= 0 && id <= ctrl_last)
                {
                    ctrl_point_modes[id] = JUMP;
                }
            }
        }
        bspline_optimizer_rebound_->setControlPointModes(ctrl_point_modes);
    }

    // 记录初始化阶段耗时，并显示初始路径和 A* 路径。
    t_init = ros::Time::now() - t_start;

    static int vis_id = 0;
    if (!astar_only_)
    {
        visualization_->displayInitPathList(point_set, 0.2, 0);
    }
    visualization_->displayAStarList(init_a_star_pathes.empty() ? a_star_pathes : init_a_star_pathes, vis_id);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE - 对 B 样条控制点做避障和平滑优化 ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
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
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
        // 若速度/加速度超限，就通过拉长时间来重新分配轨迹参数。
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        // 时间重分配
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success) pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success)
    {
        printf(
            "\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if "
            "continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
    }

    t_refine = ros::Time::now() - t_start;

    // 保存最终轨迹，供后续跟踪控制器使用。
    updateTrajInfo(pos, ros::Time::now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec()
         << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

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
