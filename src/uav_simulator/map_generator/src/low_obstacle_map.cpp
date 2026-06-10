#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <cmath>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "low_obstacle_map");
    ros::NodeHandle nh("~");

    double resolution, wall_x, wall_thickness, wall_y_half_width, wall_height;
    std::string scenario;
    nh.param("resolution", resolution, 0.1);
    nh.param("scenario", scenario, std::string("single"));
    nh.param("wall_x", wall_x, -10.0);
    nh.param("wall_thickness", wall_thickness, 0.4);
    nh.param("wall_y_half_width", wall_y_half_width, 9.0);
    nh.param("wall_height", wall_height, 1.15);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "world";

    auto add_box = [&](double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) {
        const int x_num = std::max(1, static_cast<int>(std::ceil((x_max - x_min) / resolution)));
        const int y_num = std::max(1, static_cast<int>(std::ceil((y_max - y_min) / resolution)));
        const int z_num = std::max(1, static_cast<int>(std::ceil((z_max - z_min) / resolution)));
        for (int ix = 0; ix < x_num; ++ix)
        {
            const double x = x_min + (ix + 0.5) * resolution;
            for (int iy = 0; iy < y_num; ++iy)
            {
                const double y = y_min + (iy + 0.5) * resolution;
                for (int iz = 0; iz < z_num; ++iz)
                {
                    const double z = z_min + (iz + 0.5) * resolution;
                    cloud.points.emplace_back(x, y, z);
                }
            }
        }
    };

    if (scenario == "complex")
    {
        add_box(-15.8, -15.4, -0.85, 0.25, 0.0, 0.30);
        add_box(-14.2, -13.8, -0.20, 0.95, 0.0, 0.35);
        add_box(-12.6, -12.2, -0.95, 0.15, 0.0, 0.30);
        add_box(-13.4, -13.0, 1.40, 2.20, 0.0, 0.45);
        add_box(-11.7, -11.3, -1.80, -1.05, 0.0, 0.45);
    }
    else if (scenario == "bend")
    {
        // Mixed demo scene: roll-over bump, jump wall, detour block, second jump wall, and side clutter.
        add_box(-16.75, -16.35, -0.95, 0.95, 0.0, 0.10);
        add_box(-15.35, -15.05, -0.85, 0.85, 0.0, 0.38);
        add_box(-15.55, -14.85, 1.10, 1.75, 0.0, 0.80);
        add_box(-15.55, -14.85, -1.75, -1.10, 0.0, 0.80);
        add_box(-13.65, -13.00, -0.60, 0.60, 0.0, 1.20);
        add_box(-11.70, -11.40, -0.65, 0.65, 0.0, 0.35);
        add_box(-13.95, -13.45, 1.45, 2.20, 0.0, 0.55);
        add_box(-12.70, -12.10, -2.00, -1.20, 0.0, 0.55);
        add_box(-10.80, -10.25, 1.15, 1.85, 0.0, 0.40);
    }
    else if (scenario == "detour")
    {
        add_box(-16.8, -16.4, -0.45, 0.45, 0.0, 0.30);
        add_box(-14.7, -13.3, -0.65, 0.65, 0.0, 1.25);
        add_box(-11.8, -11.4, -0.45, 0.45, 0.0, 0.30);
    }
    else
    {
        add_box(wall_x - 0.5 * wall_thickness, wall_x + 0.5 * wall_thickness,
                -wall_y_half_width, wall_y_half_width, 0.0, wall_height);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "world";

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1, true);
    ros::Rate rate(1.0);

    ROS_WARN("[low_obstacle_map] scenario=%s, points=%zu", scenario.c_str(), cloud.points.size());

    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();
        map_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
