#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

ros::Publisher map_pub;

void inflateRegion(nav_msgs::OccupancyGrid& map)
{
    int width = map.info.width;
    int height = map.info.height;
    float resolution = map.info.resolution;

    // Center of the high-cost region (in world coordinates)
    float cx = 5.0;
    float cy = 5.0;
    float radius = 2.0;

    // Convert world coordinates to map grid index
    int mx = (cx - map.info.origin.position.x) / resolution;
    int my = (cy - map.info.origin.position.y) / resolution;
    int radius_cells = radius / resolution;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            int x = mx + dx;
            int y = my + dy;

            if (x >= 0 && x < width && y >= 0 && y < height) {
                float dist = std::hypot(dx, dy);
                if (dist <= radius_cells) {
                    int index = y * width + x;
                    if (map.data[index] >= 0 && map.data[index] < 90) {
                        map.data[index] = 90; // Mark with high cost (0–100 scale)
                    }
                }
            }
        }
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::OccupancyGrid modified_map = *msg;

    inflateRegion(modified_map);

    modified_map.header.stamp = ros::Time::now();
    map_pub.publish(modified_map);
    ROS_INFO("Published modified map with inflated region.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_cost_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/ML/Map", 1, mapCallback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 1, true);

    ros::spin();
    return 0;
}

