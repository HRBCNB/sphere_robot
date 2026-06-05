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
    line_strip.scale.x = scale / 2;
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

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
}

void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
{
    if (init_list_pub.getNumSubscribers() == 0)
    {
        return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
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
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
}

void PlanningVisualization::displayAStarList(std::vector<std::vector<PathNode>> a_star_paths,
                                             int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "world";
    points.header.stamp = ros::Time::now();
    points.ns = "a_star_points";
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    points.id = id;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.18;
    points.scale.y = 0.18;
    points.scale.z = 0.18;
    points.color.a = 1.0;

    geometry_msgs::Point pt;
    std_msgs::ColorRGBA color;
    for (const auto& block : a_star_paths)
    {
        for (const auto& node : block)
        {
            pt.x = node.pos.x();
            pt.y = node.pos.y();
            pt.z = node.pos.z();
            points.points.push_back(pt);

            if (node.mode == JUMP)
            {
                color.r = 1.0;
                color.g = 0.05;
                color.b = 0.05;
                color.a = 1.0;
            }
            else
            {
                color.r = 0.05;
                color.g = 0.85;
                color.b = 0.1;
                color.a = 1.0;
            }
            points.colors.push_back(color);
        }
    }

    a_star_list_pub.publish(points);
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
