#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                delete GridNodeMap_[i][j][k];
            }
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr**[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr*[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

void AStar::setJumpParams(ros::NodeHandle& nh)
{
    nh.param("a_star/max_jump_h", max_jump_h_, 0.6);
    nh.param("a_star/max_jump_d", max_jump_d_, 1.5);
    nh.param("a_star/jump_penalty", jump_penalty_, 10.0);
    nh.param("a_star/jump_takeoff_clearance", jump_takeoff_clearance_, 0.30);
    nh.param("a_star/jump_landing_clearance", jump_landing_clearance_, 0.30);
    nh.param("a_star/roll_over_height", roll_over_height_, 0.15);
    nh.param("a_star/roll_over_penalty", roll_over_penalty_, 0.5);
    nh.param("a_star/min_roll_after_jump", min_roll_after_jump_, 0.8);
    nh.param("a_star/detour_exit_deviation", detour_exit_deviation_, 0.25);
    nh.param("a_star/frontal_jump_cos", frontal_jump_cos_, 0.75);
    nh.param("a_star/jumpable_detour_penalty", jumpable_detour_penalty_, 4.0);
    nh.param("a_star/line_deviation_weight", line_dev_weight_, 0.5);
    nh.param("a_star/max_line_deviation", max_line_deviation_, -1.0);
    nh.param("a_star/debug_decisions", debug_decisions_, true);
    nh.param("a_star/debug_decision_limit", debug_decision_limit_, 80);
    nh.param("a_star/use_inflate_for_jump", use_inflate_for_jump_, false);
    nh.param("a_star/jump_from_inflated", jump_from_inflated_, true);

    nh.param("planner/max_jump_h", max_jump_h_, max_jump_h_);
    nh.param("planner/max_jump_d", max_jump_d_, max_jump_d_);
    nh.param("planner/jump_penalty", jump_penalty_, jump_penalty_);
    nh.param("planner/jump_takeoff_clearance", jump_takeoff_clearance_, jump_takeoff_clearance_);
    nh.param("planner/jump_landing_clearance", jump_landing_clearance_, jump_landing_clearance_);
    nh.param("planner/roll_over_height", roll_over_height_, roll_over_height_);
    nh.param("planner/roll_over_penalty", roll_over_penalty_, roll_over_penalty_);
    nh.param("planner/min_roll_after_jump", min_roll_after_jump_, min_roll_after_jump_);
    nh.param("planner/detour_exit_deviation", detour_exit_deviation_, detour_exit_deviation_);
    nh.param("planner/frontal_jump_cos", frontal_jump_cos_, frontal_jump_cos_);
    nh.param("planner/jumpable_detour_penalty", jumpable_detour_penalty_, jumpable_detour_penalty_);
    nh.param("planner/line_deviation_weight", line_dev_weight_, line_dev_weight_);
    nh.param("planner/max_line_deviation", max_line_deviation_, max_line_deviation_);
    nh.param("planner/debug_decisions", debug_decisions_, debug_decisions_);
    nh.param("planner/debug_decision_limit", debug_decision_limit_, debug_decision_limit_);
    nh.param("planner/use_inflate_for_jump", use_inflate_for_jump_, use_inflate_for_jump_);
    nh.param("planner/jump_from_inflated", jump_from_inflated_, jump_from_inflated_);

    ROS_INFO("[AStar] jump params: h=%.2f, d=%.2f, penalty=%.2f, takeoff_clear=%.2f, landing_clear=%.2f, roll_h=%.2f, roll_penalty=%.2f, min_roll_after_jump=%.2f, detour_exit=%.2f, frontal_cos=%.2f, jumpable_detour_penalty=%.2f, corridor=%.2f, debug=%s/%d, jump_from_inflated=%s, use_inflate_for_jump=%s",
             max_jump_h_, max_jump_d_, jump_penalty_, jump_takeoff_clearance_, jump_landing_clearance_,
             roll_over_height_, roll_over_penalty_, min_roll_after_jump_,
             detour_exit_deviation_, frontal_jump_cos_, jumpable_detour_penalty_, max_line_deviation_,
             debug_decisions_ ? "true" : "false", debug_decision_limit_, jump_from_inflated_ ? "true" : "false",
             use_inflate_for_jump_ ? "true" : "false");
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i& start_idx,
                                                  Vector3i& end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx)) return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx)) return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx)) return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    // TODO：更改A*算法的寻路逻辑 roll模式不考虑Z轴，只有在前方有障碍物的时候再考虑Jump模式
    ros::Time time_1 = ros::Time::now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    const Eigen::Vector2d line_start = start_pt.head<2>();
    const Eigen::Vector2d line_end = end_pt.head<2>();
    const Eigen::Vector2d line_vec = line_end - line_start;
    const double line_len = line_vec.norm();
    auto lineDeviation = [&](const Eigen::Vector3d& pos) {
        if (line_len < 1e-6)
        {
            return 0.0;
        }
        const Eigen::Vector2d rel = pos.head<2>() - line_start;
        const double cross = fabs(line_vec.x() * rel.y() - line_vec.y() * rel.x());
        return cross / line_len;
    };
    auto lineDeviationCost = [&](const Eigen::Vector3d& pos) {
        if (line_dev_weight_ <= 1e-6)
        {
            return 0.0;
        }
        const double deviation = lineDeviation(pos);
        return line_dev_weight_ * deviation * deviation;
    };
    auto insideLineCorridor = [&](const Eigen::Vector3d& pos) {
        if (max_line_deviation_ <= 1e-6)
        {
            return true;
        }
        return lineDeviation(pos) <= max_line_deviation_;
    };
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET;  // put start node in open set
    startPtr->mode = ego_planner::ROLL;
    startPtr->dist_since_jump = 1e9;
    startPtr->detouring = false;
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr);  // put start in open set

    endPtr->index = end_idx;
    endPtr->mode = ego_planner::ROLL;
    endPtr->dist_since_jump = 1e9;
    endPtr->detouring = false;

    double tentative_gScore;

    int num_iter = 0;
    int occupied_neighbor_count = 0;
    int jump_candidate_count = 0;
    int feasible_jump_count = 0;
    int roll_node_count = 0;
    int roll_over_node_count = 0;
    int jump_node_count = 0;
    int detour_required_count = 0;
    int jump_preferred_count = 0;
    int debug_decision_count = 0;
    jump_fail_map_ = 0;
    jump_fail_landing_occ_ = 0;
    jump_fail_range_ = 0;
    jump_fail_peak_map_ = 0;
    jump_fail_arc_map_ = 0;
    jump_fail_arc_occ_ = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();
        if (current->state == GridNode::CLOSEDSET) continue;

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) &&
            current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            ROS_INFO("[AStar] success: iter=%d, path_nodes=%zu, occ_neighbors=%d, roll_nodes=%d, roll_over_nodes=%d, jump_nodes=%d, detour_required=%d, jump_preferred=%d, jump_candidates=%d, feasible_jumps=%d",
                     num_iter, gridPath_.size(), occupied_neighbor_count, roll_node_count, roll_over_node_count,
                     jump_node_count, detour_required_count, jump_preferred_count, jump_candidate_count,
                     feasible_jump_count);
            return true;
        }
        current->state = GridNode::CLOSEDSET;  // move current node from open set to closed set.

        const Eigen::Vector3d current_expand_pos = Index2Coord(current->index);
        const bool current_detouring = current->detouring && lineDeviation(current_expand_pos) > detour_exit_deviation_;

        auto obstacleHeightForMode = [&](const Eigen::Vector3d& pos) {
            const double raw_top = grid_map_->getRawObstacleHeight(pos);
            if (raw_top > 0.0)
            {
                return raw_top;
            }

            // If the cell is occupied only by inflation, use the inflated top for mode classification.
            // Otherwise a low obstacle's inflated shell looks like raw_top=0 and is incorrectly forced to detour.
            return grid_map_->getObstacleHeight(pos);
        };

        auto canRollOver = [&](const Eigen::Vector3d& pos) {
            const double obstacle_top = obstacleHeightForMode(pos);
            return obstacle_top > 0.0 && obstacle_top <= pos.z() + roll_over_height_;
        };

        auto localJumpDir = [&](const int step_x, const int step_y) -> Eigen::Vector3d {
            Eigen::Vector3d dir(double(step_x), double(step_y), 0.0);
            if (dir.norm() < 1e-6)
            {
                return Eigen::Vector3d::Zero();
            }
            return dir.normalized();
        };

        auto currentGoalDir = [&]() -> Eigen::Vector3d {
            Eigen::Vector3d dir = Index2Coord(endPtr->index) - current_expand_pos;
            dir.z() = 0.0;
            if (dir.norm() < 1e-6)
            {
                return Eigen::Vector3d::Zero();
            }
            return dir.normalized();
        };

        auto isFrontalJump = [&](const Eigen::Vector3d& jump_dir) {
            const Eigen::Vector3d goal_dir = currentGoalDir();
            return goal_dir.norm() < 1e-6 || jump_dir.dot(goal_dir) >= frontal_jump_cos_;
        };

        auto isJumpableObstacle = [&](const Eigen::Vector3d& obstacle_pos) {
            const double obstacle_top = obstacleHeightForMode(obstacle_pos);
            return obstacle_top > obstacle_pos.z() + roll_over_height_ && obstacle_top <= obstacle_pos.z() + max_jump_h_;
        };

        auto tryAddNode = [&](GridNodePtr node, const Eigen::Vector3i& idx, const double tentative_g,
                              const ego_planner::TRAJ_MODE mode, GridNodePtr parent,
                              const double dist_since_jump, const bool detouring) {
            const bool explored = node->rounds == rounds_;
            if (explored && node->state == GridNode::CLOSEDSET)
            {
                return false;
            }
            if (!explored || tentative_g < node->gScore)
            {
                node->rounds = rounds_;
                node->index = idx;
                node->state = GridNode::OPENSET;
                node->cameFrom = parent;
                node->dist_since_jump = dist_since_jump;
                node->detouring = detouring;
                node->gScore = tentative_g;
                node->fScore = tentative_g + getHeu(node, endPtr);
                node->mode = mode;
                openSet_.push(node);
                return true;
            }
            return false;
        };

        auto canJumpToward = [&](const Eigen::Vector3d& jump_dir, const double min_landing_dist, Eigen::Vector3d* landing_pos_out) {
            if (current_detouring || current->dist_since_jump < min_roll_after_jump_ || jump_dir.norm() < 1e-6 ||
                !isFrontalJump(jump_dir))
            {
                return false;
            }

            const double first_landing_dist = std::max(0.5, min_landing_dist);
            for (double dist = first_landing_dist; dist <= max_jump_d_; dist += step_size_)
            {
                Eigen::Vector3d landing_pos = current_expand_pos + jump_dir * dist;
                ++jump_candidate_count;

                if (!insideLineCorridor(landing_pos))
                {
                    continue;
                }
                if (!isJumpFeasible(current_expand_pos, landing_pos))
                {
                    continue;
                }

                Eigen::Vector3i landing_idx;
                if (!Coord2IndexNoWarn(landing_pos, landing_idx))
                {
                    continue;
                }
                GridNodePtr landing_node = GridNodeMap_[landing_idx(0)][landing_idx(1)][landing_idx(2)];
                const bool landing_explored = landing_node->rounds == rounds_;
                if (landing_explored && landing_node->state == GridNode::CLOSEDSET)
                {
                    continue;
                }

                if (landing_pos_out) *landing_pos_out = landing_pos;
                ++feasible_jump_count;
                return true;
            }
            return false;
        };

        auto findJumpTrigger = [&](const Eigen::Vector3d& jump_dir, Eigen::Vector3d* obstacle_pos_out,
                                   double* obstacle_dist_out) {
            if (jump_dir.norm() < 1e-6 || !isFrontalJump(jump_dir))
            {
                return false;
            }

            const double lookahead = std::min(max_jump_d_ - jump_landing_clearance_,
                                              jump_takeoff_clearance_ + 2.0 * step_size_);
            for (double dist = step_size_; dist <= lookahead + 1e-6; dist += step_size_)
            {
                const Eigen::Vector3d scan_pos = current_expand_pos + jump_dir * dist;
                if (!insideLineCorridor(scan_pos) || !grid_map_->isInMap(scan_pos))
                {
                    continue;
                }
                if (!checkOccupancy(scan_pos) || canRollOver(scan_pos))
                {
                    continue;
                }
                if (!isJumpableObstacle(scan_pos))
                {
                    return false;
                }
                if (!jump_from_inflated_ && !checkRawOccupancy(scan_pos))
                {
                    continue;
                }
                if (dist + 1e-6 < jump_takeoff_clearance_)
                {
                    return false;
                }
                if (obstacle_pos_out) *obstacle_pos_out = scan_pos;
                if (obstacle_dist_out) *obstacle_dist_out = dist;
                return true;
            }
            return false;
        };

        auto printDecision = [&](const char* tag, const Eigen::Vector3d& neighbor_pos, const int step_x, const int step_y,
                                const double raw_top, const double inflated_top, const bool roll_possible,
                                const bool jumpable, const bool frontal, const bool jump_possible,
                                const double roll_cost, const double jump_cost, const double detour_extra,
                                const bool detour_required_now) {
            if (!debug_decisions_ || debug_decision_count >= debug_decision_limit_)
            {
                return;
            }
            ++debug_decision_count;
            ROS_INFO("[AStar][decision %d] %s cur=(%.2f %.2f %.2f) nb=(%.2f %.2f %.2f) step=(%d,%d) raw_top=%.2f infl_top=%.2f roll=%s roll_cost=%.2f jumpable=%s frontal=%s jump_possible=%s jump_cost=%.2f detour_extra=%.2f detouring=%s detour_required=%s dist_since_jump=%.2f g=%.2f",
                     debug_decision_count, tag, current_expand_pos.x(), current_expand_pos.y(), current_expand_pos.z(),
                     neighbor_pos.x(), neighbor_pos.y(), neighbor_pos.z(), step_x, step_y, raw_top, inflated_top,
                     roll_possible ? "Y" : "N", roll_cost, jumpable ? "Y" : "N", frontal ? "Y" : "N",
                     jump_possible ? "Y" : "N", jump_cost, detour_extra, current_detouring ? "Y" : "N",
                     detour_required_now ? "Y" : "N", current->dist_since_jump, current->gScore);
        };

        bool detour_required = current_detouring;
        bool jump_preferred = false;
        for (int sx = -1; sx <= 1 && !detour_required; ++sx)
            for (int sy = -1; sy <= 1 && !detour_required; ++sy)
            {
                if (sx == 0 && sy == 0) continue;

                Eigen::Vector3i scan_idx;
                scan_idx(0) = current->index(0) + sx;
                scan_idx(1) = current->index(1) + sy;
                scan_idx(2) = current->index(2);

                if (scan_idx(0) < 1 || scan_idx(0) >= POOL_SIZE_(0) - 1 || scan_idx(1) < 1 ||
                    scan_idx(1) >= POOL_SIZE_(1) - 1 || scan_idx(2) < 1 || scan_idx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                const Eigen::Vector3d scan_pos = Index2Coord(scan_idx);
                if (!insideLineCorridor(scan_pos) || !checkOccupancy(scan_pos)) continue;

                const Eigen::Vector3d scan_jump_dir = localJumpDir(sx, sy);
                if (!isFrontalJump(scan_jump_dir)) continue;

                if (canRollOver(scan_pos)) continue;
                if (!isJumpableObstacle(scan_pos))
                {
                    detour_required = true;
                    ++detour_required_count;
                    continue;
                }
                if (!jump_from_inflated_ && !checkRawOccupancy(scan_pos)) continue;

                Eigen::Vector3d unused_landing;
                if (canJumpToward(scan_jump_dir, jump_landing_clearance_ + step_size_, &unused_landing))
                {
                    jump_preferred = true;
                }
                else
                {
                    detour_required = true;
                    ++detour_required_count;
                }
            }

        if (jump_preferred) ++jump_preferred_count;

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            // 滚动模式不考虑Z轴的变化
            // for (int dz = -1; dz <= 1; dz++)
            {
                int dz = 0;
                if (dx == 0 && dy == 0 && dz == 0) continue;

                Vector3i neighborIdx;
                neighborIdx(0) = current->index(0) + dx;
                neighborIdx(1) = current->index(1) + dy;
                neighborIdx(2) = current->index(2) + dz;

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 ||
                    neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                const Vector3d neighbor_pos = Index2Coord(neighborIdx);
                if (!insideLineCorridor(neighbor_pos))
                {
                    continue;
                }

                const double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                const bool next_detouring = detour_required && lineDeviation(neighbor_pos) > detour_exit_deviation_;
                const double jumpable_detour_cost = jump_preferred ? jumpable_detour_penalty_ : 0.0;

                // 普通 free cell -> ROLL。若前方短距离内有可跳障碍，也在这里提前生成 JUMP successor，
                // 避免走到障碍边缘才起跳。
                if (!checkOccupancy(neighbor_pos))
                {
                    Eigen::Vector3d trigger_obstacle_pos = Eigen::Vector3d::Zero();
                    double trigger_obstacle_dist = 0.0;
                    Eigen::Vector3d early_landing_pos = Eigen::Vector3d::Zero();
                    bool early_jump_possible = false;
                    double early_jump_cost = -1.0;
                    const Eigen::Vector3d early_jump_dir = localJumpDir(dx, dy);

                    if (!detour_required && findJumpTrigger(early_jump_dir, &trigger_obstacle_pos, &trigger_obstacle_dist))
                    {
                        const double min_landing_dist = trigger_obstacle_dist + jump_landing_clearance_;
                        early_jump_possible = canJumpToward(early_jump_dir, min_landing_dist, &early_landing_pos);
                        if (early_jump_possible)
                        {
                            const double jump_dist = (early_landing_pos - current_expand_pos).norm();
                            early_jump_cost = jump_dist + jump_penalty_ + lineDeviationCost(early_landing_pos);
                            Eigen::Vector3i early_landing_idx;
                            if (Coord2IndexNoWarn(early_landing_pos, early_landing_idx))
                            {
                                GridNodePtr jumpNodePtr = GridNodeMap_[early_landing_idx(0)][early_landing_idx(1)][early_landing_idx(2)];
                                printDecision("JUMP_EARLY", trigger_obstacle_pos, dx, dy,
                                              grid_map_->getRawObstacleHeight(trigger_obstacle_pos),
                                              grid_map_->getObstacleHeight(trigger_obstacle_pos), false, true,
                                              true, true, static_cost, early_jump_cost, jumpable_detour_cost,
                                              detour_required);
                                if (tryAddNode(jumpNodePtr, early_landing_idx, current->gScore + early_jump_cost,
                                               ego_planner::JUMP, current, 0.0, false))
                                {
                                    ++jump_node_count;
                                }
                            }
                        }
                    }

                    tentative_gScore = current->gScore + static_cost + lineDeviationCost(neighbor_pos) +
                                       jumpable_detour_cost;
                    if (tryAddNode(neighborPtr, neighborIdx, tentative_gScore, ego_planner::ROLL, current,
                                   current->dist_since_jump + static_cost * step_size_, next_detouring))
                    {
                        ++roll_node_count;
                    }
                    continue;
                }

                ++occupied_neighbor_count;
                const double raw_top = grid_map_->getRawObstacleHeight(neighbor_pos);
                const double inflated_top = grid_map_->getObstacleHeight(neighbor_pos);
                const double mode_top = raw_top > 0.0 ? raw_top : inflated_top;
                const bool roll_possible = mode_top > 0.0 && mode_top <= neighbor_pos.z() + roll_over_height_;
                const bool jumpable = mode_top > neighbor_pos.z() + roll_over_height_ &&
                                      mode_top <= neighbor_pos.z() + max_jump_h_;
                const Eigen::Vector3d jump_dir = localJumpDir(dx, dy);
                const bool frontal = isFrontalJump(jump_dir);
                Eigen::Vector3d landing_pos = Eigen::Vector3d::Zero();
                bool jump_possible = false;
                double jump_cost = -1.0;

                if (!detour_required && jumpable && (jump_from_inflated_ || checkRawOccupancy(neighbor_pos)))
                {
                    jump_possible = canJumpToward(jump_dir, jump_landing_clearance_ + step_size_, &landing_pos);
                    if (jump_possible)
                    {
                        const double jump_dist = (landing_pos - current_expand_pos).norm();
                        jump_cost = jump_dist + jump_penalty_ + lineDeviationCost(landing_pos);
                    }
                }

                const double roll_cost = static_cost + roll_over_penalty_ + lineDeviationCost(neighbor_pos) +
                                         jumpable_detour_cost;

                // 低障碍可滚越 -> ROLL，增加代价
                if (roll_possible)
                {
                    tentative_gScore = current->gScore + roll_cost;
                    printDecision("ROLL_OVER", neighbor_pos, dx, dy, raw_top, inflated_top, roll_possible, jumpable,
                                  frontal, jump_possible, roll_cost, jump_cost, jumpable_detour_cost,
                                  detour_required);
                    if (tryAddNode(neighborPtr, neighborIdx, tentative_gScore, ego_planner::ROLL, current,
                                   current->dist_since_jump + static_cost * step_size_, next_detouring))
                    {
                        ++roll_over_node_count;
                    }
                    continue;
                }

                // 低障碍不可滚但可跳 -> JUMP
                if (jump_possible)
                {
                    Eigen::Vector3i landing_idx;
                    if (Coord2IndexNoWarn(landing_pos, landing_idx))
                    {
                        GridNodePtr jumpNodePtr = GridNodeMap_[landing_idx(0)][landing_idx(1)][landing_idx(2)];
                        const double tentative_jump_g = current->gScore + jump_cost;
                        printDecision("JUMP_ADD", neighbor_pos, dx, dy, raw_top, inflated_top, roll_possible,
                                      jumpable, frontal, jump_possible, roll_cost, jump_cost, jumpable_detour_cost,
                                      detour_required);
                        if (tryAddNode(jumpNodePtr, landing_idx, tentative_jump_g, ego_planner::JUMP, current,
                                       0.0, false))
                        {
                            ++jump_node_count;
                        }
                    }
                }
                else
                {
                    printDecision("NO_JUMP_DETOUR", neighbor_pos, dx, dy, raw_top, inflated_top, roll_possible,
                                  jumpable, frontal, jump_possible, roll_cost, jump_cost, jumpable_detour_cost,
                                  detour_required);
                }

                // 跳不过且可绕 -> 不加入 occupied neighbor；free-cell 扩展会形成 ROLL detour。
                // 跳不过且绕不过 -> openSet 最终耗尽，A* fail。
                continue;
            }  // end of for loop of neighbor expansion
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded. iter=%d, occ_neighbors=%d, jump_candidates=%d, feasible_jumps=%d",
                     num_iter, occupied_neighbor_count, jump_candidate_count, feasible_jump_count);
            ROS_WARN("A star jump infeasible reasons: map=%d, landing_occ=%d, range=%d, peak_map=%d, arc_map=%d, arc_occ=%d",
                     jump_fail_map_, jump_fail_landing_occ_, jump_fail_range_, jump_fail_peak_map_, jump_fail_arc_map_,
                     jump_fail_arc_occ_);
            return false;
        }
    }

    ros::Time time_2 = ros::Time::now();

    ROS_WARN("A star failed: iter=%d, occ_neighbors=%d, jump_candidates=%d, feasible_jumps=%d, time=%.3fs",
             num_iter, occupied_neighbor_count, jump_candidate_count, feasible_jump_count, (time_2 - time_1).toSec());
    ROS_WARN("A star jump infeasible reasons: map=%d, landing_occ=%d, range=%d, peak_map=%d, arc_map=%d, arc_occ=%d",
             jump_fail_map_, jump_fail_landing_occ_, jump_fail_range_, jump_fail_peak_map_, jump_fail_arc_map_,
             jump_fail_arc_occ_);

    return false;
}  // end AstarSearch

vector<PathNode> AStar::getPath()
{
    vector<PathNode> path;

    vector<GridNodePtr> forward_path = gridPath_;
    reverse(forward_path.begin(), forward_path.end());

    for (size_t i = 0; i < forward_path.size(); ++i)
    {
        auto ptr = forward_path[i];
        PathNode node;
        node.pos = Index2Coord(ptr->index);
        node.mode = ptr->mode;

        if (i > 0 && ptr->mode == ego_planner::JUMP)
        {
            Eigen::Vector3d start_pos = Index2Coord(forward_path[i - 1]->index);
            Eigen::Vector3d landing_pos = node.pos;
            Eigen::Vector2d delta_xy = landing_pos.head<2>() - start_pos.head<2>();
            double horizontal_dist = delta_xy.norm();

            if (horizontal_dist > 1e-3)
            {
                double start_z = start_pos.z();
                double end_z = landing_pos.z();
                int num_samples = max(2, static_cast<int>(ceil(horizontal_dist / step_size_)));

                for (int sample = 1; sample <= num_samples; ++sample)
                {
                    double ratio = static_cast<double>(sample) / num_samples;
                    PathNode jump_node;
                    jump_node.pos.head<2>() = start_pos.head<2>() + delta_xy * ratio;
                    jump_node.pos.z() = (1.0 - ratio) * start_z + ratio * end_z +
                                        4.0 * max_jump_h_ * ratio * (1.0 - ratio);
                    jump_node.mode = ego_planner::JUMP;
                    path.push_back(jump_node);
                }
                continue;
            }
        }

        path.push_back(node);
    }

    return path;
}

bool AStar::isJumpFeasible(const Vector3d& start_pos, const Vector3d& landing_pos)
{
    if (!grid_map_->isInMap(start_pos) || !grid_map_->isInMap(landing_pos))
    {
        ++jump_fail_map_;
        return false;
    }

    if (checkOccupancy(landing_pos))
    {
        ++jump_fail_landing_occ_;
        return false;
    }

    double dist = (landing_pos - start_pos).norm();
    int num_checks = max(2, static_cast<int>(dist / (step_size_ / 2.0)));  // 检验点数量

    // 3. 沿途高度校验（抛物线轨迹模拟）
    // 计算抛物线参数
    double horizontal_dist = (landing_pos.head<2>() - start_pos.head<2>()).norm();
    double start_z = start_pos.z();
    double end_z = landing_pos.z();
    if (horizontal_dist < 1e-3 || horizontal_dist > max_jump_d_ || fabs(end_z - start_z) > max_jump_h_)
    {
        ++jump_fail_range_;
        return false;
    }

    // 假设跳跃最高点在水平距离的中点，且高于起始点和落点
    double peak_h = fmax(start_z, end_z) + max_jump_h_;  // 调整峰值高度，确保在max_jump_h_范围内

    // 计算抛物线系数 z = a * x^2 + b * x + c
    // 简化处理，假设水平轴为x，垂直轴为z
    // 我们知道三个点: (0, start_z), (horizontal_dist / 2, peak_h), (horizontal_dist, end_z)
    // a * 0^2 + b * 0 + c = start_z  => c = start_z
    // a * (horizontal_dist / 2)^2 + b * (horizontal_dist / 2) + start_z = peak_h
    // a * horizontal_dist^2 + b * horizontal_dist + start_z = end_z

    if (!grid_map_->isInMap(Vector3d((start_pos.x() + landing_pos.x()) * 0.5,
                                     (start_pos.y() + landing_pos.y()) * 0.5, peak_h)))
    {
        ++jump_fail_peak_map_;
        return false;
    }

    double a = (2 * peak_h - start_z - end_z) / (-(horizontal_dist * horizontal_dist / 2));
    double b = (end_z - start_z - a * horizontal_dist * horizontal_dist) / horizontal_dist;
    double c = start_z;

    for (int i = 1; i < num_checks; i++)
    {
        // 在水平方向上插值
        double ratio = static_cast<double>(i) / num_checks;
        Vector2d check_pos_2d = start_pos.head<2>() + (landing_pos.head<2>() - start_pos.head<2>()) * ratio;
        double current_h_on_parabola = a * pow(horizontal_dist * ratio, 2) + b * (horizontal_dist * ratio) + c;

        Vector3d check_pos(check_pos_2d.x(), check_pos_2d.y(), current_h_on_parabola);

        // 检查路径点是否在地图范围内
        if (!grid_map_->isInMap(check_pos))
        {
            ++jump_fail_arc_map_;
            return false;  // 如果路径点超出地图范围，则认为不可行
        }

        // 如果该点在障碍物中，且高度低于跳跃轨迹的高度，则认为碰撞
        if (checkJumpOccupancy(check_pos))
        {
            ++jump_fail_arc_occ_;
            return false;  // 轨迹与障碍物碰撞
        }
    }

    return true;  // 跳跃路径可行
}
