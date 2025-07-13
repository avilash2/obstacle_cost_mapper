#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

ros::Publisher costmap_pub;

void setCostOnObstacles(nav_msgs::OccupancyGrid& map, int cost_value, int inflation_radius)
{
    int width = map.info.width;
    int height = map.info.height;
    nav_msgs::OccupancyGrid original = map;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int idx = y * width + x;

            if (original.data[idx] == 100)  // Obstacle cell
            {
                // Set cost on the obstacle itself
                map.data[idx] = cost_value;

                // Inflate cost around it
                for (int dx = -inflation_radius; dx <= inflation_radius; ++dx)
                {
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                        {
                            float dist = std::hypot(dx, dy);
                            if (dist <= inflation_radius)
                            {
                                int nidx = ny * width + nx;
                                if (map.data[nidx] != 100 && map.data[nidx] != -1)
                                {
                                    // Add decreasing cost (e.g., linear falloff)
                                    int inflation_cost = std::max(1, static_cast<int>(cost_value - (dist*10)));
                                    map.data[nidx] = static_cast<int8_t>(std::max(static_cast<int>(map.data[nidx]), inflation_cost));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::OccupancyGrid cost_map = *msg;
    int cost_value = 100;         // Set 100 for obstacles
    int inflation_radius = 5;     // In cells

    setCostOnObstacles(cost_map, cost_value, inflation_radius);

    cost_map.header.stamp = ros::Time::now();
    costmap_pub.publish(cost_map);

    ROS_INFO("Published map with updated obstacle costs.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_obstacle_cost_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/ML/Map", 1, mapCallback);
    costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap_with_obstacles", 1, true);

    ros::spin();
    return 0;
}

