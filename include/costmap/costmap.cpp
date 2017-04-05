#include "costmap.h"
#define LETHAL 100
#define SAFESIDE 90

costmap::costmap(double x_orig_,double y_orig_,unsigned int cell_x_size_,unsigned int cell_y_size_,float resolution_,std::string frame_id_,bool show_debug_)
{
    default_cost = -1; //unknown cell
    resolution = resolution_;
    x_orig = x_orig_;
    y_orig = y_orig_;
    cell_x_size = cell_x_size_;
    cell_y_size = cell_y_size_;
    x_size = cell_x_size*resolution;
    y_size = cell_y_size*resolution;
    ROS_INFO("cell_x_size: %d   and    cell_y_size:   %d   ",cell_x_size,cell_y_size);
    mat.resize(cell_x_size, std::vector<signed char> (cell_y_size, default_cost));
    frame_id = frame_id_;
    show_debug = show_debug_;
    if (show_debug) ROS_INFO("costmap prints debug msgs");
    init_ = true;
    if (show_debug) ROS_INFO("costmap initialized successfully");
    valid = false;
    //pub_ = 0;
}
costmap::costmap(nav_msgs::OccupancyGrid m, bool show_debug_)
{
  default_cost = -1; //unknown cell
  resolution = m.info.resolution;
  x_orig = m.info.origin.position.x;
  y_orig = m.info.origin.position.y;
  cell_x_size = m.info.width;
  cell_y_size = m.info.height;
  x_size = cell_x_size*resolution;
  y_size = cell_y_size*resolution;
  mat.resize(cell_x_size, std::vector<signed char> (cell_y_size, default_cost));
  VectorToMatrix(m.data);
  frame_id = m.header.frame_id;
  valid = false;
  show_debug = show_debug_;
  if (show_debug) ROS_INFO("costmap prints debug msgs");
  init_ = true;
  if (show_debug) ROS_INFO("costmap initialized successfully");
}
costmap::costmap(nav_msgs::OccupancyGrid::ConstPtr m, bool show_debug_)
{
  default_cost = -1; //unknown cell
  resolution = m->info.resolution;
  x_orig = m->info.origin.position.x;
  y_orig = m->info.origin.position.y;
  cell_x_size = m->info.width;
  cell_y_size = m->info.height;
  x_size = cell_x_size*resolution;
  y_size = cell_y_size*resolution;
  mat.resize(cell_x_size, std::vector<signed char> (cell_y_size, default_cost));
  VectorToMatrix(m->data);
  frame_id = m->header.frame_id;
  valid = false;
  show_debug = show_debug_;
  if (show_debug) ROS_INFO("costmap prints debug msgs");
  init_ = true;
  if (show_debug) ROS_INFO("costmap initialized successfully");
}
costmap::costmap()
{
   init_ = false;
   //pub_ = 0;
}

costmap::~costmap()
{
  ROS_WARN("costmap instant destructed");
  init_ = false;
}

bool costmap::is_valid()
{
  return valid;
}
void costmap::Lethal_inf(int mx,int my, float rad)
{
   int cellrad = (int) fabs(rad/resolution);
   for(int i=-cellrad;i <= cellrad;i++)
     for(int j=-cellrad;j <= cellrad;j++)
       if(mx+i >= 0 && mx+i<= cell_x_size && my+j >= 0 && my+j<= cell_y_size) mat[mx+i][my+j] = LETHAL;
       //if (Is_in_map(i,j)) index_vec.push_back(std::make_pair(i,j));
}
void costmap::SafeSide_inf(int mx,int my, float rad)
{
   int cellrad = (int) fabs(rad/resolution);
   for(int i=-cellrad;i <= cellrad;i++)
     for(int j=-cellrad;j <= cellrad;j++)
       if(mx+i >= 0 && mx+i<= cell_x_size && my+j >= 0 && my+j<= cell_y_size)
         if(mat[mx+i][my+j] != LETHAL) mat[mx+i][my+j] = SAFESIDE;

}
void costmap::inflate(float Lethal_rad,float safeside_rad)
{
  std::vector<std::pair<unsigned int,unsigned int> > Lethal_index;
  // LETHAL INDEX Find
  int count = 0;
  for(int i=0;i<cell_x_size;i++)
    for(int j=0;j<cell_y_size;j++)
    {
      if(mat[i][j] == LETHAL)
      {
        count++;
        Lethal_index.push_back(std::make_pair(i,j));
        //Lethal_inf(i,j,Lethatl_rad,Lethal_index);
      }
    }

  // APPLY COST
  for(int m=0;m<Lethal_index.size();m++)
    Lethal_inf(Lethal_index[m].first,Lethal_index[m].second,Lethal_rad);

   // SAFESIDE INDEX
  if(safeside_rad == 0.000) return;
//  std::vector<std::pair<unsigned int,unsigned int> > SafeSide_index;
//  for(int i=0;i<cell_x_size;i++)
//    for(int j=0;j<cell_y_size;j++)
//    {
//      if(mat[i][j] == LETHAL)
//      {
//        SafeSide_inf(i,j,safeside_rad);
//      }
//    }
  //APPLY COST
  for(int m=0;m<Lethal_index.size();m++)
    SafeSide_inf(Lethal_index[m].first,Lethal_index[m].second, Lethal_rad + safeside_rad);
}

void costmap::UpdateFromMap(nav_msgs::OccupancyGrid m)
{
  if(m.info.width != cell_x_size || m.info.height != cell_y_size)
  {
    ROS_WARN("map size does not match costmap size, Reinitializing the costmap");
    default_cost = -1; //unknown cell
    resolution = m.info.resolution;
    x_orig = m.info.origin.position.x;
    y_orig = m.info.origin.position.y;
    cell_x_size = m.info.width;
    cell_y_size = m.info.height;
    ROS_INFO_STREAM("cell_x_size: " << cell_x_size << "   cell_y_size: " <<cell_y_size);
    x_size = cell_x_size*resolution;
    y_size = cell_y_size*resolution;
    mat.resize(cell_x_size, std::vector<signed char> (cell_y_size, default_cost));
    init_ = true;
    if (show_debug) ROS_INFO("costmap initialized successfully");
  }
  VectorToMatrix(m.data);
}

bool costmap::is_initialized()
{
  bool res = false;
  if (init_) res = true;
  return res;
}

float costmap::getOriginX()
{
    float out = x_orig;
    return out;
}
float costmap::getOriginY()
{
    float out = y_orig;
    return out;
}
float costmap::getResolution()
{
    float out = resolution;
    return out;
}
void costmap::get_costmap_size_in_meter(float& x_size_,float& y_size_)
{
    x_size_ = x_size;
    y_size_ = y_size;
}
void costmap::get_costmap_size_in_cell(unsigned int& mx, unsigned int& my)
{
    mx = cell_x_size;
    my = cell_y_size;
}

bool costmap::worldToMap (float wx, float wy,unsigned int& mx, unsigned int& my)
{
    bool is_valid = true;
	 mx = (int)((wx - x_orig) / resolution);
     my = (int)((wy - y_orig) / resolution);
     if(mx > cell_x_size || my > cell_y_size ) is_valid = false;
     return is_valid;
}
bool costmap::worldToMap (float wx, float wy,size_t& mx, size_t& my)
{
    bool is_valid = true;
     mx = (size_t) abs((wx - x_orig) / resolution);
     my = (size_t) abs((wy - y_orig) / resolution);
     if(mx > cell_x_size || my > cell_y_size ) is_valid = false;
     return is_valid;
}

void costmap::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    wx = x_orig + (mx + 0.5) * resolution;
    wy = y_orig + (my + 0.5) * resolution;
}

void costmap::worldToMapEnforceBounds (double wx, double wy, int &mx, int &my)
{
	 mx = (int)((wx - x_orig) / resolution);
     my = (int)((wy - y_orig) / resolution);
     mx = std::min(mx,(int) cell_x_size);
     mx = std::max(mx,0);
     my = std::min(my,(int) cell_x_size);
     my = std::max(my,0);

}
signed char costmap::getCost(unsigned int mx,unsigned int my)
{
    
    signed char cost;
    if(mx < cell_x_size && my < cell_y_size)
       cost = mat[my][mx];
    else
    {
        cost = default_cost;
        if(show_debug) ROS_ERROR("Out of range cost request");
    }
    return cost;
}
signed char costmap::getCost_WC(float wx,float wy) //cost from world coordinates
{
    signed char cost;
    unsigned int mx,my;
    if(!worldToMap(wx,wy,mx,my))
    {
        cost = default_cost;
        if(show_debug) ROS_ERROR("Out of range cost request");
    }
    else cost = getCost(mx,my);
    return cost;
}
nav_msgs::OccupancyGrid costmap::getROSmsg()
{
    nav_msgs::OccupancyGrid msg;
    if(!init_)
    {
      ROS_ERROR("costmap: instant is yet to be initialized, no ROS msg for you!");
      return msg;
    }

    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now(); 
    msg.info.resolution = resolution;
    msg.info.width = cell_x_size;
    msg.info.height = cell_y_size;
    msg.info.origin.position.x = x_orig;
    msg.info.origin.position.y = y_orig;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = MatrixToVector();
    return msg;
}
void costmap::setCost(unsigned int mx,unsigned int my,signed char cost)
{
    if(mx < cell_x_size && my < cell_y_size)
    {
        mat[my][mx] = cost;
        valid = true;
    }
    else
        if(show_debug) ROS_ERROR("Out of range cell request");
}
void costmap::setCost_WC(float wx,float wy, signed char cost)
{
     unsigned mx,my;
     bool k = worldToMap(wx,wy,mx,my);
     setCost(mx,my,cost);
     valid = true;
}
void costmap::setCost_v(std::vector<signed char> vector,int cols)
{
    //ROS_ERROR_STREAM("vector size is   "<<vector.size());
    
    if (vector.size()%cols != 0)
    {
        ROS_ERROR("Number of columns:%d is incompatible with vector size:%d",cols,(int) vector.size());
        return;
    }
    for(int i = 0;i< vector.size();i++)
    {
        mat[floor(i/cols)][i%cols] = vector[i];
        //ROS_WARN_STREAM("cost:  " << (int) vector[i] << "is set to i and j:  " <<floor(i/cols) <<"   and   "<< i%cols );
    }
    valid = false;
}
void costmap::resetMap() 
{
    int rows = mat.size();
    int cols = mat[0].size();
    for(int i = 0;i< rows; i++)
    {
        for(int j = 0; j< cols; j++)
        {
            mat[i][j] = 0;
        }
    }
    valid = false;
}

void costmap::resetMap (unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
    for(int i = x0;i< xn; i++)
    {
        for(int j = y0; j< yn; j++)
        {
            mat[i][j] = 0;
        }
    }
    valid = false;
}
std::vector<signed char> costmap::MatrixToVector()
{
    if (!init_)
    {
      ROS_ERROR("costmap: costmap yet to be initialized!");
      std::vector<signed char> temp;
      return temp;

    }
    unsigned int rows = cell_x_size;//mat.size();
    unsigned int cols = cell_y_size;//mat[0].size(); cause seg fault
    size_t size = rows*cols;
    if(show_debug) ROS_WARN_STREAM("Vector size is " << rows*cols<<"   cell_x_size: " << cell_x_size << "   cell_y_size: " <<cell_y_size);
    std::vector<signed char> Vector (size);
    //ROS_INFO("rows:%d     cols:%d   size:%d",rows,cols,rows*cols);
    for(unsigned int i = 0;i< rows; i++)
    {
        for(unsigned int j = 0; j< cols; j++)
        {
            Vector[j + i*cols ] = mat[i][j];
            //ROS_WARN_STREAM("vector[ " <<j + i*cols<<" ] = Matrix["<<i<<"] ["<<j<<"]  =  "<< (int) Vector[j + i*cols ]);
            
        }
    }
    return Vector;
}
void costmap::VectorToMatrix(std::vector<signed char> Vector)
{
  for(int i =0; i < cell_x_size;i++)
  {
    for(int j=0;j < cell_y_size;j++)
    {
        mat[i][j] = Vector[j+i*cell_x_size];
    }
  }
}
bool costmap::Is_in_map(int mx,int my){
  if (mx > cell_x_size || mx < 0)
    return false;
  if (my > cell_y_size || my < 0)
    return false;
  return true;
}
bool costmap::Is_in_map_WC(float wx,float wy){
  size_t mx,my;
  return worldToMap(wx,wy,mx,my);
}
