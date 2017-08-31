#ifndef COSTMAP_H_MWH_20170829
#define COSTMAP_H_MWH_20170829

#include<vector>
#include<queue>
#include"geometry_msgs/Point.h"
#include<boost/thread.hpp>

namespace costmap
{
struct MapLocation
{
    unsigned int x;
    unsigned int y;
};

class Costmap
{
public:
    Costmap(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
        double origin_x, double origin_y, unsigned char default_value = 0);
    Costmap(const Costmap& map);
    Costmap& operator=(const Costmap& map);
    bool copyCostmapWindow(const Costmap& map,
                           double win_origin_x,
                           double win_origin_y
                           double win_size_x,
                           double win_size_y);
    Costmap();
    virtual ~Costmap();

    unsigned char getCost(unsigned int mx,unsigned int my)const;
    void setCost(unsigned int mx, unsigned int my, unsigned char cost);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
    void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;
    void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;

    inline unsigned int getIndex(unsigned int mx, unsigned int my) const
    {
      return my * size_x_ + mx;
    }

    inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
    {
      my = index / size_x_;
      mx = index - (my * size_x_);
    }

    unsigned char* getCharMap() const;
    unsigned int getSizeInCellsX() const;
    unsigned int getSizeInCellsY() const;
    double getSizeInMetersX() const;
    double getSizeInMetersY() const;
    double getOriginX() const;
    double getOriginY() const;
    double getResolution() const;
    
    void setDefaultValue(unsigned char c)
    {
      default_value_ = c;
    }

    unsigned char getDefaultValue()
    {
      return default_value_;
    }

    bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, 
                              unsigned char cost_value);
    void polygonOutlineCells(const std::vector<MapLocation>& polygon, 
                             std::vector<MapLocation>& polygon_cells);
    void convexFillCells(const std::vector<MapLocation>& polygon, 
                         std::vector<MapLocation>& polygon_cells);
    virtual void updateOrigin(double new_origin_x, double new_origin_y);
    bool saveMap(std::string file_name);
    void resizeMap(unsigned int size_x, 
                    unsigned int size_y, 
                    double resolution, 
                    double origin_x,
                    double origin_y);
    void resetMap(unsigned int x0, 
                    unsigned int y0, 
                    unsigned int xn, 
                    unsigned int yn);
    unsigned int cellDistance(double world_dist);
    typedef boost::recursive_mutex mutex_t;
    
    mutex_t* getMutex()
    {
      return access_;
    }
protected:
    template<typename data_type>
    void copyMapRegion(data_type* source_map, 
                       unsigned int sm_lower_left_x, 
                       unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, 
                       data_type* dest_map, 
                       unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, 
                       unsigned int dm_size_x, 
                       unsigned int region_size_x,
                       unsigned int region_size_y)
    {
      data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
      data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

      for (unsigned int i = 0; i < region_size_y; ++i)
      {
        memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
        sm_index += sm_size_x;
        dm_index += dm_size_x;
      }
    }

    virtual void deleteMaps();
    virtual void resetMaps();
    virtual void initMaps(unsigned int size_x, unsigned int size_y);

    template<class ActionType>
    inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                             unsigned int max_length = UINT_MAX)
    {
      int dx = x1 - x0;
      int dy = y1 - y0;

      unsigned int abs_dx = abs(dx);
      unsigned int abs_dy = abs(dy);

      int offset_dx = sign(dx);
      int offset_dy = sign(dy) * size_x_;

      unsigned int offset = y0 * size_x_ + x0;

      // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
      double dist = hypot(dx, dy);
      double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

      // if x is dominant
      if (abs_dx >= abs_dy)
      {
        int error_y = abs_dx / 2;
        bresenham(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
      }

      // otherwise y is dominant
      int error_x = abs_dy / 2;
      bresenham(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
    }

    template<class ActionType>
    inline void bresenham(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                            int offset_b, unsigned int offset, unsigned int max_length)
    {
      unsigned int end = std::min(max_length, abs_da);
      for (unsigned int i = 0; i < end; ++i)
      {
        at(offset);
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
          offset += offset_b;
          error_b -= abs_da;
        }
      }
      at(offset);
    }

    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

    mutex_t* access_;

protected:
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    unsigned char* costmap_;
    unsigned char default_value_;  

    class MarkCell
    {
    public:
      MarkCell(unsigned char* costmap, unsigned char value) :
          costmap_(costmap), value_(value)
      {
      }
      inline void operator()(unsigned int offset)
      {
        costmap_[offset] = value_;
      }
    private:
      unsigned char* costmap_;
      unsigned char value_;
    };

    class PolygonOutlineCells
    {
    public:
      PolygonOutlineCells(const Costmap& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells) :
          costmap_(costmap), char_map_(char_map), cells_(cells)
      {
      }
  
      // just push the relevant cells back onto the list
      inline void operator()(unsigned int offset)
      {
        MapLocation loc;
        costmap_.indexToCells(offset, loc.x, loc.y);
        cells_.push_back(loc);
      }

    private:
      const Costmap& costmap_;
      const unsigned char* char_map_;
      std::vector<MapLocation>& cells_;
    };
};

}

#endif