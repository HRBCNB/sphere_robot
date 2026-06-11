#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh)
{
    node = nh;

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20, true);
}

// // real ids used: {id, id+1000}
void PlanningVisualization::displayMarkerList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale,
                                              Eigen::Vector4d color, int id)
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = std::max(0.015, scale / 3.0);
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
    }
    pub.publish(sphere);
    pub.publish(line_strip);
}

// real ids used: {id, id+1}
void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray& array,
                                                     const vector<Eigen::Vector3d>& list, double scale,
                                                     Eigen::Vector4d color, int id)
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
}

// real ids used: {1000*id ~ (arrow nums)+1000*id}
void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray& array,
                                                      const vector<Eigen::Vector3d>& list, double scale,
                                                      Eigen::Vector4d color, int id)
{
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
        // arrow.color.r = color(0) / (1+i);
        // arrow.color.g = color(1) / (1+i);
        // arrow.color.b = color(2) / (1+i);

        start.x = list[2 * i](0);
        start.y = list[2 * i](1);
        start.z = list[2 * i](2);
        end.x = list[2 * i + 1](0);
        end.y = list[2 * i + 1](1);
        end.z = list[2 * i + 1](2);
        arrow.points.clear();
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.id = i + id * 1000;

        array.markers.push_back(arrow);
    }
}

void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale,
                                             int id)
{
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
}

void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
{
    if (global_list_pub.getNumSubscribers() == 0)
    {
        return;
    }

    Eigen::Vector4d color(0.0, 0.35, 0.9, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
}

void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
{
    if (init_list_pub.getNumSubscribers() == 0)
    {
        return;
    }

    Eigen::Vector4d color(0.0, 0.20, 0.85, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
}

void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
{
    if (optimal_list_pub.getNumSubscribers() == 0)
    {
        return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
        Eigen::Vector3d pt = optimal_pts.col(i).transpose();
        list.push_back(pt);
    }
    Eigen::Vector4d color(0.35, 0.18, 0.95, 0.75);
    displayMarkerList(optimal_list_pub, list, 0.045, color, id);
}

void PlanningVisualization::displayBsplineTrajectory(UniformBspline& traj, double sample_step, int id)
{
    if (optimal_list_pub.getNumSubscribers() == 0)
    {
        return;
    }

    const int line_id = 100000 + id * 10000;

    visualization_msgs::Marker clear_old;
    clear_old.header.frame_id = "world";
    clear_old.header.stamp = ros::Time::now();
    clear_old.action = visualization_msgs::Marker::DELETE;
    clear_old.id = line_id;
    optimal_list_pub.publish(clear_old);
    clear_old.id = line_id + 1000;
    optimal_list_pub.publish(clear_old);

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "bspline_trajectory";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = line_id;
    line_strip.pose.orientation.w = 1.0;
    const bool rejected_candidate = id >= 9000;
    line_strip.color.r = rejected_candidate ? 1.0 : 0.0;
    line_strip.color.g = rejected_candidate ? 0.18 : 0.32;
    line_strip.color.b = rejected_candidate ? 0.02 : 1.0;
    line_strip.color.a = rejected_candidate ? 0.9 : 1.0;
    line_strip.scale.x = rejected_candidate ? 0.045 : 0.065;

    const double duration = traj.getTimeSum();
    const double dt = std::max(0.02, sample_step);
    geometry_msgs::Point pt;
    for (double t = 0.0; t < duration; t += dt)
    {
        const Eigen::Vector3d p = traj.evaluateDeBoorT(t);
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z() + 0.02;
        line_strip.points.push_back(pt);
    }
    const Eigen::Vector3d end_pt = traj.evaluateDeBoorT(duration);
    pt.x = end_pt.x();
    pt.y = end_pt.y();
    pt.z = end_pt.z() + 0.02;
    line_strip.points.push_back(pt);

    optimal_list_pub.publish(line_strip);
}

void PlanningVisualization::displayAStarList(std::vector<std::vector<PathNode>> a_star_paths,
                                             int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
{
    visualization_msgs::Marker clear_old;
    clear_old.header.frame_id = "world";
    clear_old.header.stamp = ros::Time::now();
    clear_old.ns = "a_star_points";
    clear_old.action = visualization_msgs::Marker::DELETE;
    clear_old.id = id;
    a_star_list_pub.publish(clear_old);

    visualization_msgs::Marker roll_points, jump_points;
    roll_points.header.frame_id = jump_points.header.frame_id = "world";
    roll_points.header.stamp = jump_points.header.stamp = ros::Time::now();
    roll_points.ns = "a_star_roll_points";
    jump_points.ns = "a_star_jump_points";
    roll_points.type = jump_points.type = visualization_msgs::Marker::SPHERE_LIST;
    roll_points.action = jump_points.action = visualization_msgs::Marker::ADD;
    roll_points.id = id;
    jump_points.id = id + 1000;
    roll_points.pose.orientation.w = jump_points.pose.orientation.w = 1.0;

    roll_points.scale.x = 0.080;
    roll_points.scale.y = 0.080;
    roll_points.scale.z = 0.080;
    jump_points.scale.x = 0.100;
    jump_points.scale.y = 0.100;
    jump_points.scale.z = 0.100;

    roll_points.color.r = 0.95;
    roll_points.color.g = 0.0;
    roll_points.color.b = 0.85;
    roll_points.color.a = 0.75;
    jump_points.color.r = 1.0;
    jump_points.color.g = 0.22;
    jump_points.color.b = 0.02;
    jump_points.color.a = 0.95;

    geometry_msgs::Point pt;
    for (const auto& block : a_star_paths)
    {
        for (const auto& node : block)
        {
            pt.x = node.pos.x();
            pt.y = node.pos.y();
            pt.z = node.pos.z() + 0.035;
            if (node.mode == JUMP)
            {
                jump_points.points.push_back(pt);
            }
            else
            {
                roll_points.points.push_back(pt);
            }
        }
    }

    a_star_list_pub.publish(roll_points);
    a_star_list_pub.publish(jump_points);
}

void PlanningVisualization::displayArrowList(ros::Publisher& pub, const vector<Eigen::Vector3d>& list, double scale,
                                             Eigen::Vector4d color, int id)
{
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
}

// PlanningVisualization::
}  // namespace ego_planner
