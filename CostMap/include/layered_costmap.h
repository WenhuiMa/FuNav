#ifndef COSTMAP_LAYERED_COSTMAP_H_MWH_20170831
#define COSTMAP_LAYERED_COSTMAP_H_MWH_20170831

#include "cost_values.h"
#include "layer.h"
#include "costmap.h"
#include <vector>
#include <string>

namespace costmap
{
class Layer;

class LayeredCostmap
{

public:
    LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);
    ~LayeredCostmap();

    void updateMap(double robot_x, double robot_y, double robot_yaw);
    
    std::string getGlobalFrameID() const
    {
      return global_frame_;
    }

    void resizeMap(unsigned int size_x, 
                   unsigned int size_y, 
                   double resolution, 
                   double origin_x, 
                   double origin_y,
                   bool size_locked = false);
    
    void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy)
    {
        minx = minx_;
        miny = miny_;
        maxx = maxx_;
        maxy = maxy_;
    }

    bool isCurrent();

    Costmap* getCostmap()
    {
      return &costmap_;
    }

    bool isRolling()
    {
      return rolling_window_;
    }

    bool isTrackingUnknown()
    {
      return costmap_.getDefaultValue() == costmap::NO_INFORMATION;
    }

    std::vector<boost::shared_ptr<Layer> >* getPlugins()
    {
      return &plugins_;
    }

    void addPlugin(boost::shared_ptr<Layer> plugin)
    {
      plugins_.push_back(plugin);
    }

    bool isSizeLocked()
    {
      return size_locked_;
    }

    void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
    {
      *x0 = bx0_;
      *xn = bxn_;
      *y0 = by0_;
      *yn = byn_;
    }

    bool isInitialized()
    {
        return initialized_;
    }

    void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);
    const std::vector<geometry_msgs::Point>& getFootprint() { return footprint_; }
    double getCircumscribedRadius() { return circumscribed_radius_; }
    double getInscribedRadius() { return inscribed_radius_; }

private:
    Costmap costmap_;
    std::string global_frame_;

    bool rolling_window_;
    bool current_;
    double minx_,miny_,maxx_,maxy_;
    unsigned int bx0_,bxn_,by0_,byn_;

    std::vector<boost::shared_ptr<Layer> > plugins_;

    bool initialized_;
    bool size_locked_;
    double circumscribed_radius_, inscribed_radius_;
    std::vector<geometry_msgs::Point> footprint_;
};

}

#endif