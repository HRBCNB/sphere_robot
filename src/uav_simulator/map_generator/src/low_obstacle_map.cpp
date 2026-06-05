#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "low_obstacle_map");
    ros::NodeHandle nh("~");

    double resolution, wall_x, wall_thickness, wall_y_half_width, wall_height;
    nh.param("resolution", resolution, 0.1);
    nh.param("wall_x", wall_x, -10.0);
    nh.param("wall_thickness", wall_thickness, 0.4);
    nh.param("wall_y_half_width", wall_y_half_width, 9.0);
    nh.param("wall_height", wall_height, 1.15);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "world";

    const int x_num = std::max(1, static_cast<int>(std::ceil(wall_thickness / resolution)));
    const int y_num = std::max(1, static_cast<int>(std::ceil(2.0 * wall_y_half_width / resolution)));
    const int z_num = std::max(1, static_cast<int>(std::ceil(wall_height / resolution)));

    for (int ix = 0; ix < x_num; ++ix)
    {
        const double x = wall_x - 0.5 * wall_thickness + (ix + 0.5) * resolution;
        for (int iy = 0; iy < y_num; ++iy)
        {
            const double y = -wall_y_half_width + (iy + 0.5) * resolution;
            for (int iz = 0; iz < z_num; ++iz)
            {
                const double z = (iz + 0.5) * resolution;
                cloud.points.emplace_back(x, y, z);
            }
        }
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "world";

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1, true);
    ros::Rate rate(1.0);

    ROS_WARN("[low_obstacle_map] fixed wall: x=%.2f, y=[%.2f, %.2f], height=%.2f, points=%zu", wall_x,
             -wall_y_half_width, wall_y_half_width, wall_height, cloud.points.size());

    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();
        map_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
