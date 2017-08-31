#ifndef INFLATION_LAYER_H_MWH_20170831
#define INFLATION_LAYER_H_MWH_20170831

#include<ros/ros.h>
#include"layer.h"
#include"layered_costmap.h"
#include<boost/thread.hpp>

namespace costmap
{
    class CellData
    {
    public:
        CellData(double i, 
                 unsigned int x, 
                 unsigned int y, 
                 unsigned int sx, 
                 unsigned int sy) :index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy){}

        unsigned int index_;
        unsigned int x_, y_;
        unsigned int src_x_, src_y_;
    };

    class InflationLayer: public Layer
    {
    public:
        InflationLayer();

        virtual ~InflationLayer()
        {
            deleteKernels();
            if(dsrv_)
                delete dsrv_;
        }

        virtual void onInitialize();

        virtual void updateBounds(double robot_x, 
                                  double robot_y, 
                                  double robot_yaw, 
                                  double* min_x, 
                                  double* min_y,
                                  double* max_x, 
                                  double* max_y);

        virtual void updateCosts(costmap::Costmap& master_grid, 
                                 int min_i, int min_j, int max_i, int max_j);          
             
        virtual bool isDiscretized()
        {
            return true;
        }      
        
        virtual void matchSize();

        virtual void reset() { onInitialize(); }

        inline unsigned char computeCost(double distance) const
        {
          unsigned char cost = 0;
          if (distance == 0)
            cost = LETHAL_OBSTACLE;
          else if (distance * resolution_ <= inscribed_radius_)
            cost = INSCRIBED_INFLATED_OBSTACLE;
          else
          {
            // make sure cost falls off by Euclidean distance
            double euclidean_distance = distance * resolution_;
            double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
            cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
          }
          return cost;
        }

        void setInflationParameters(double inflation_radius, double cost_scaling_factor);

    protected:
        virtual void onFootprintChanged();
        boost::recursive_mutex* inflation_access_;

        inline double distanceLookup(int mx, int my, int src_x, int src_y)
        {
          unsigned int dx = abs(mx - src_x);
          unsigned int dy = abs(my - src_y);
          return cached_distances_[dx][dy];
        }

        inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
        {
          unsigned int dx = abs(mx - src_x);
          unsigned int dy = abs(my - src_y);
          return cached_costs_[dx][dy];
        }

        void computeCaches();
        void deleteKernels();
        void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);
        
        unsigned int cellDistance(double world_dist)
        {
          return layered_costmap_->getCostmap()->cellDistance(world_dist);
        }

        inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
            unsigned int src_x, unsigned int src_y);

        double inflation_radius_, inscribed_radius_, weight_;
        unsigned int cell_inflation_radius_;
        unsigned int cached_cell_inflation_radius_;
        std::map<double, std::vector<CellData> > inflation_cells_;
        double resolution_;

        bool* seen_;
        int seen_size_;
      
        unsigned char** cached_costs_;
        double** cached_distances_;
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

        bool need_reinflation_;
    } ;
}


#endif