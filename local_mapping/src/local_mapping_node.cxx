#include "local_mapper.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "local_mapper_node");
  ros::NodeHandle nh;

  LocalMapper mapper(nh);

  ros::spin();
  return 0;
}
