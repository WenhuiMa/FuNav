#include "move_base.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_node");
  tf::TransformListener tf(ros::Duration(10));
  move_base::MoveBase move_base( tf );
  std::cout<<"working........."<<std::endl;
  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}