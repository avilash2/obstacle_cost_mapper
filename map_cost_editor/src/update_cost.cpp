#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>


ros::Publisher costmap_pub;

void setObstacleCost(nav_msgs::OccupancyGrid& map, int cost_value, int inflation_radius) {
 int width_ = map.info.width;
 int height_ = map.info.height;
 nav_msgs::OccupancyGrid original = map;
 
 for (int y=0; y < height_; ++y) {
  
  for (int x = 0; x < width_; ++x){
   
   int idx = y * width_ + x;
   
   if (original.data[idx] == 100) {
    
    map.data[idx] = cost_value;
    
    // adding inflation
    for (int dy = -inflation_radius; dy < inflation_radius; ++dy){
     
     for (int dx = -inflation_radius; dx < inflation_radius; ++dx){
      
      int nx = x + dx;
      int ny = y + dy;
      
      if (nx > 0 && nx < width_ && ny > 0 && ny < height_) { 
      
       
      
       float dist = std::hypot(dx, dy);
      
       if (dist <= inflation_radius) {
        int nidx = ny * width_ + nx;
        
        if (map.data[nidx] != 100 && map.data[nidx] != -1){
         int inflation_cost = std::max(1, cost_value - static_cast<int>(dist*10));
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

void mapCallback (const nav_msgs::OccupancyGrid::ConstPtr& msg) {
 nav_msgs::OccupancyGrid cost_map = *msg;
 
 int inflation_radius = 5;
 
 int cost_value = 100;
 
 setObstacleCost(cost_map, cost_value, inflation_radius);
 
 cost_map.header.stamp = ros::Time::now();
 
 costmap_pub.publish(cost_map);
 
 ROS_INFO("costmap published");
}


int main (int argc, char** argv) 
{
 ros::init(argc, argv, "obstacle_cost_node");
 ros::NodeHandle nh_;
 
 ros::Subscriber map_sub = nh_.subscribe("/ML/Map", 1, mapCallback);
 
 costmap_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/obstacle_cost", 1, true);
 
 ros::spin();
 return 0;
}
