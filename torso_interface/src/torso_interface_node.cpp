#include <torso_interface/torso_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "torso_interface");
  ros::NodeHandle n;

  TorsoInterface torso_interface(n);
  torso_interface.start();
}