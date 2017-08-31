#ifndef COSTMAP_LAYER_H_MWH_20170831
#define COSTMAP_LAYER_H_MWH_20170831

#include"costmap.h"
#include "layered_costmap.h"

#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace costmap
{
class LayeredCostmap;

class Layer
{
public:
    Layer();
    void initialize(LayeredCostmap* parent, std::string name, tf::TransformListener *tf);
    virtual void updateBounds(double robot_x, 
                              double robot_y, 
                              double robot_yaw, 
                              double* min_x, 
                              double* min_y,
                              double* max_x, 
                              double* max_y) {}
    
    virtual void updateCosts(Costmap& master_grid, 
                             int min_i, int min_j, int max_i, int max_j) {}
    
    virtual void deactivate() {}    
    
    virtual void activate() {}

    virtual void reset() {}

    virtual ~Layer() {}

    bool isCurrent() const
    {
      return current_;
    }

    virtual void matchSize() {}

    std::string getName() const
    {
      return name_;
    }

    const std::vector<geometry_msgs::Point>& getFootprint() const;

    virtual void onFootprintChanged() {}

protected:
    virtual void onInitialize() {}

    LayeredCostmap* layered_costmap_;
    bool current_;
    bool enabled_;
    std::string name_;
    tf::TransformListener* tf_;

private:
    std::vector<geometry_msgs::Point> footprint_spec_;

};

}

#endif