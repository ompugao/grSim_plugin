
#include <boost/config.hpp>
#include <boost/dll/alias.hpp>

#include <grsim/robot.h>
#include <iostream>

namespace scramble {
class AttackerRobot : public Robot {
  public:
    AttackerRobot():
      Robot() {
      std::cout << "Attacker Robot loaded!" << std::endl;
    }
    virtual void initialize(PWorld* world,PBall* ball,ConfigWidget* _cfg,dReal x,dReal y,dReal z,dReal r,dReal g,dReal b,int rob_id,int wheeltexid,int dir) {
      Robot::initialize(world, ball, _cfg, x, y, z, r, g, b, rob_id, wheeltexid, dir);
    }
    virtual ~AttackerRobot() {
      std::cout << "Attacker Robot unloaded" << std::endl;
    }
};

// boost::shared_ptr<AttackerRobot> createAttackerRobot() {
//   return boost::shared_ptr<AttackerRobot>(new AttackerRobot());
// }
AttackerRobot* createAttackerRobot() {        
  return new AttackerRobot(); 
}                                                               

BOOST_DLL_ALIAS(scramble::createAttackerRobot, create_robot__scramble__attacker)

}
