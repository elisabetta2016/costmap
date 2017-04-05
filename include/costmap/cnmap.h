#ifndef CNMAP_H
#define CNMAP_H
#include "costmap.h"
#include <geometry_msgs/Pose.h>

class cnmap
{
public:
  cnmap(costmap *base_map_ptr, int scale_, bool debug_);
  cnmap(ros::NodeHandle *n_ptr,std::string map_topic_name, int scale_, bool debug_);
  cnmap();
  ~cnmap();
  void test();
  void update();
  void set_home(float wx,float wy);
  void publish_ROS(ros::Publisher *pubPtr_);
  void publish_ROS();
  bool is_initialized();
  bool is_home_set();
  geometry_msgs::Pose find_discovered(float wx,float wy);

protected:
  float resolution; // meters/cell
  std::vector < std::pair<unsigned int,unsigned int> > DUL;  //Discovered Unchecked Cells
  std::vector < std::pair<unsigned int,unsigned int> > DCL;  //Discovered Checkd    Cells
  std::pair <unsigned int,unsigned int> home_cell;
  bool debug;
  costmap *base_ptr;
  costmap *self_ptr;
  bool init_;
  bool home_;
  bool DUL_init;
  int scale;
  ros::Publisher map_pub;
  ros::Subscriber map_sub;

private:
  void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  unsigned char find_cell_status(int a,int b);
  void check_surrouding_cells(int a,int b);
  bool is_in_vector(std::pair <unsigned int,unsigned int> e,std::vector < std::pair <unsigned int,unsigned int> > ls);
};

#endif // CNMAP_H
