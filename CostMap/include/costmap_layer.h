#ifndef COSTMAP_LAYER_H_MWH_20170831
#define COSTMAP_LAYER_H_MWH_20170831

#include"layer.h"
#include"layered_costmap.h"

namespace costmap
{

    class CostmapLayer:public Layer,public Costmap
    {
    public:
        CostmapLayer():has_extra_bounds_(false),
                       extra_min_x_(1e6),
                       extra_max_x_(-1e6),
                       extra_min_y_(1e6),
                       extra_max_y_(-1e6){}
        bool isDiscretized()
        {
            return true;
        }

        virtual void matchSize();

        void addExtraBounds(double mx0, double my0, double mx1, double my1);

    protected:
        void updateWithTrueOverwrite(costmap::Costmap& master_grid, 
                                     int min_i, int min_j, int max_i, int max_j);
        void updateWithOverwrite(costmap::Costmap& master_grid, 
                                 int min_i, int min_j, int max_i, int max_j);
        void updateWithMax(costmap::Costmap& master_grid, 
                            int min_i, int min_j, int max_i, int max_j);
        void updateWithAddition(costmap::Costmap& master_grid, 
                                int min_i, int min_j, int max_i, int max_j);
        void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);
        
        void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);

        bool has_extra_bounds_;

    private:
        double extra_min_x_,extra_min_y_,extra_max_x_,extra_max_y_;

    };

}


#endif