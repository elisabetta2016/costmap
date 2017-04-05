#ifndef COSTMAP_H
#define COSTMAP_H

//#includes
 //Standard
  #include <ros/ros.h>
  #include <ros/timer.h>
  #include <math.h>
  #include <iostream>
  #include <string>
 //Messages
  #include <nav_msgs/OccupancyGrid.h>

class costmap{
    public:
        costmap(double x_orig_,double y_orig_,unsigned int cell_x_size_,unsigned int cell_y_size_,float resolution_,std::string frame_id_,bool show_debug_);
        costmap(nav_msgs::OccupancyGrid m, bool show_debug_);
        costmap(nav_msgs::OccupancyGrid::ConstPtr m, bool show_debug_);
        costmap();
        ~costmap();
        bool is_valid();
        float getOriginX();
        float getOriginY();
        float getResolution();
        void  get_costmap_size_in_meter(float& x_size_,float& y_size_);
        void  get_costmap_size_in_cell(unsigned int& mx, unsigned int& my);
        bool  worldToMap (float wx, float wy,unsigned int& mx, unsigned int& my);
        bool  worldToMap (float wx, float wy,size_t& mx, size_t& my);
        void  worldToMapEnforceBounds (double wx, double wy, int &mx, int &my);
        bool  Is_in_map(int mx,int my);
        bool  Is_in_map_WC(float wx,float wy);
        void  mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
        signed char getCost(unsigned int mx,unsigned int my);
        signed char getCost_WC(float wx,float wy); //cost from world coordinates
        nav_msgs::OccupancyGrid getROSmsg();
        void UpdateFromMap(nav_msgs::OccupancyGrid m);
        void  setCost(unsigned int mx,unsigned int my,signed char cost);

        void setCost_WC(float wx,float wy, signed char cost);
        void resetMap();
        void resetMap (unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);
        void setCost_v(std::vector<signed char> vector,int cols);
        bool is_initialized();
        void inflate(float Lethal_rad, float safeside_rad);
        //void publish(ros::Publisher *pub, std::string topic_name);
        std::string frame_id;
        bool show_debug;

    protected:
        float resolution; // meters/cell
        float x_orig;
        float y_orig;
        float x_size;
        float y_size;
        bool valid;
        unsigned int cell_x_size;
        unsigned int cell_y_size;
        std::vector< std::vector<signed char> > mat;
        bool init_;


    private:
        signed char default_cost;
        std::vector<signed char> MatrixToVector();
        void VectorToMatrix(std::vector<signed char> Vector);
        void Lethal_inf(int mx, int my, float rad);
        void SafeSide_inf(int mx, int my, float rad);

};
#endif
