#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <vector>
namespace auto_aim
{

struct Command
{
  bool control; //是否瞄准
  bool shoot;   //是否射击
  double yaw;   
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

}  // namespace io

#endif  // COMMAND_HPP