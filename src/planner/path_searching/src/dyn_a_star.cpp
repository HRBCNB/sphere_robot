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
    nh.param("a_star/jump_penalty", jump_penalty_, 5.0);
    nh.param("a_star/line_deviation_weight", line_dev_weight_, 2.0);
    nh.param("a_star/max_line_deviation", max_line_deviation_, -1.0);
    nh.param("a_star/use_inflate_for_jump", use_inflate_for_jump_, false);

    nh.param("planner/max_jump_h", max_jump_h_, max_jump_h_);
    nh.param("planner/max_jump_d", max_jump_d_, max_jump_d_);
    nh.param("planner/jump_penalty", jump_penalty_, jump_penalty_);
    nh.param("planner/line_deviation_weight", line_dev_weight_, line_dev_weight_);
    nh.param("planner/max_line_deviation", max_line_deviation_, max_line_deviation_);
    nh.param("planner/use_inflate_for_jump", use_inflate_for_jump_, use_inflate_for_jump_);
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
    const Eigen::Vector3d preferred_jump_dir =
        line_len > 1e-6 ? Eigen::Vector3d(line_vec.x() / line_len, line_vec.y() / line_len, 0.0)
                         : Eigen::Vector3d::Zero();

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
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr);  // put start in open set

    endPtr->index = end_idx;
    endPtr->mode = ego_planner::ROLL;

    double tentative_gScore;

    int num_iter = 0;
    int occupied_neighbor_count = 0;
    int jump_candidate_count = 0;
    int feasible_jump_count = 0;
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
            return true;
        }
        current->state = GridNode::CLOSEDSET;  // move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            // 滚动模式不考虑Z轴的变化
            // for (int dz = -1; dz <= 1; dz++)
            {
                int dz = 0;
                if (dx == 0 && dy == 0 && dz == 0) continue;
                // 拓展邻居节点的索引
                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = (current->index)(2) + dz;
                // 越界处理
                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 ||
                    neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }
                // 获取邻居节点指针
                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                neighborPtr->index = neighborIdx;
                const Vector3d neighbor_pos = Index2Coord(neighborIdx);
                if (!insideLineCorridor(neighbor_pos))
                {
                    continue;
                }
                // 判断邻居节点是否已经被探索过，如果已经被探索过且在闭集里，则跳过
                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue;  // in closed set.
                }

                neighborPtr->rounds = rounds_;

                // 检测到邻居节点在障碍物中
                if (checkOccupancy(Index2Coord(neighborPtr->index)))
                {
                    ++occupied_neighbor_count;
                    // Prefer jumping along the start-goal line, so a frontal low obstacle is crossed straight.
                    Vector3d jump_dir = preferred_jump_dir.norm() > 1e-6
                                            ? preferred_jump_dir
                                            : Vector3d(double(dx), double(dy), 0.0).normalized();
                    Vector3d start_pos = Index2Coord(current->index);

                    // 在最大跳跃跨度内搜索落脚点
                    for (double dist = 0.5; dist <= max_jump_d_; dist += step_size_)
                    {
                        Vector3d landing_pos = start_pos + jump_dir * dist;
                        ++jump_candidate_count;

                        if (isJumpFeasible(start_pos, landing_pos))
                        // TODO：实现isJumpFeasible函数，判断从start_pos跳跃到landing_pos的路径上是否有障碍物
                        {
                            ++feasible_jump_count;
                            // 落点
                            Vector3i landing_idx;
                            if (!Coord2IndexNoWarn(landing_pos, landing_idx))
                            {
                                // 落点在当前 A* 局部搜索池外，跳过这个候选。
                                continue;
                            }
                            if (!insideLineCorridor(landing_pos))
                            {
                                continue;
                            }
                            GridNodePtr jumpNodePtr = GridNodeMap_[landing_idx(0)][landing_idx(1)][landing_idx(2)];
                            jumpNodePtr->index = landing_idx;

                            // 计算跳跃代价：物理距离 + 创新点惩罚
                            double jump_cost = dist + jump_penalty_ + lineDeviationCost(landing_pos);
                            double tentative_gScore = current->gScore + jump_cost;

                            bool jump_explored = (jumpNodePtr->rounds == rounds_);

                            // 检查该节点是否已在 ClosedSet 中
                            if (jump_explored && jumpNodePtr->state == GridNode::CLOSEDSET) continue;

                            // 更新或发现 Jump 节点
                            if (!jump_explored || tentative_gScore < jumpNodePtr->gScore)
                            {
                                jumpNodePtr->rounds = rounds_;
                                jumpNodePtr->state = GridNode::OPENSET;
                                jumpNodePtr->cameFrom = current;
                                jumpNodePtr->gScore = tentative_gScore;
                                jumpNodePtr->fScore = tentative_gScore + getHeu(jumpNodePtr, endPtr);

                                // 标记为 JUMP 模式，用于 retrievePath 生成拱形轨迹
                                jumpNodePtr->mode = ego_planner::JUMP;
                                openSet_.push(jumpNodePtr);
                            }
                            // 找到第一个最优落脚点后，跳出当前方向的 dist 循环
                            break;
                        }
                    }
                    // 处理完跳跃尝试后，跳过当前这个被占用的 ROLL 邻居
                    continue;
                }
                else
                {
                    // 代价
                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost + lineDeviationCost(neighbor_pos);

                    if (!flag_explored)  // 没有拓展过，加入open set
                    {
                        // discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        neighborPtr->mode = ego_planner::ROLL;
                        openSet_.push(neighborPtr);  // put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)  // 已经拓展过，更新
                    {                                                 // in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        neighborPtr->mode = ego_planner::ROLL;
                        openSet_.push(neighborPtr);
                    }
                }
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
