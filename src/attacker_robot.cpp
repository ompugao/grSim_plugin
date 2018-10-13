
#include <boost/config.hpp>
#include <boost/dll/alias.hpp>

#include <grsim/robot.h>
#include <iostream>

namespace scramble {

class AttackerRobot : public Robot {
  public:
    class AttackerKicker: public Kicker {
      private:
        int rolling;
        int kickstate;
        dReal m_kickspeed,m_kicktime;
        PCylinder* disc;
        Robot* rob;
        dJointID joint;
      public:
        AttackerKicker(Robot* robot) : Kicker(robot) {
          rob = robot;

          dReal x, y, z;
          robot->chassis->getBodyPosition(x,y,z);
          dReal z_offset = -(rob->cfg->robotSettings.RobotHeight)*0.5f+rob->cfg->robotSettings.WheelRadius-rob->cfg->robotSettings.BottomHeight;
          disc = new PCylinder(x, y, z + z_offset,
                  rob->cfg->robotSettings.RobotRadius + rob->cfg->robotSettings.KickerThickness,
                  rob->cfg->robotSettings.KickerHeight, rob->cfg->robotSettings.KickerMass*0.99f,
                  0.9, 0.9, 0.9, -1, false);
          disc->setBodyPosition(0, 0, z_offset, true);
          disc->space = rob->space;
          rob->getWorld()->addObject(disc);

          joint = dJointCreateFixed (rob->getWorld()->world,0);
          dJointAttach (joint, rob->chassis->body, disc->body);

          rolling = 0;
          kickstate = 0;
        }

        virtual ~AttackerKicker() {
          dJointDestroy(joint);
        }
        virtual void step() {
          if (kickstate > 0)
          {
              disc->setColor(1,0.3,0);
              kickstate--;
          } else if (rolling!=0) {
              disc->setColor(1,0.7,0);
              if (isTouchingBall())
              {
                  // ** this kicker cannot catch the ball! **
                  // dReal fx,fy,fz;
                  // rob->chassis->getBodyDirection(fx,fy,fz);
                  // fz = sqrt(fx*fx + fy*fy);
                  // fx/=fz;fy/=fz;

                  //rob->getBall()->tag = rob->getID();

                  //dReal vx,vy,vz;
                  //dReal bx,by,bz;
                  //dReal kx,ky,kz;
                  //rob->chassis->getBodyDirection(vx,vy,vz);
                  //rob->getBall()->getBodyPosition(bx,by,bz);
                  //disc->getBodyPosition(kx,ky,kz);
                  //dReal yy = -((-(kx-bx)*vy + (ky-by)*vx)) / rob->cfg->robotSettings.KickerWidth;
                  //dBodySetAngularVel(rob->getBall()->body,
                      //fy*rob->cfg->robotSettings.RollerTorqueFactor*1400,
                      //-fx*rob->cfg->robotSettings.RollerTorqueFactor*1400,0);
                  //dBodyAddTorque(rob->getBall()->body,
                      //yy * fx * rob->cfg->robotSettings.RollerPerpendicularTorqueFactor,
                      //yy * fy * rob->cfg->robotSettings.RollerPerpendicularTorqueFactor, 0);

              }
          } else {
            disc->setColor(0.9,0.9,0.9);
          }
        }
        virtual void kick(dReal kickspeedx, dReal kickspeedz) {
          if (isTouchingBall()) {
            dReal rx,ry,rz;
            dReal bx,by,bz;
            dReal dx,dy,dz;
            dReal vx,vy,vz;
            //rob->chassis->getBodyDirection(dx,dy,dz);dz = 0;
            rob->chassis->getBodyPosition(rx, ry, rz);
            rob->getBall()->getBodyPosition(bx, by, bz);
            dx = bx - rx;
            dy = by - ry;
            dz = 0;
            dReal zf = kickspeedz;
            dReal dlen = sqrt(dx*dx+dy*dy+dz*dz);
            vx = dx*kickspeedx/dlen;
            vy = dy*kickspeedx/dlen;
            vz = zf;
            const dReal* vball = dBodyGetLinearVel(rob->getBall()->body);
            dReal vn = -(vball[0]*dx + vball[1]*dy)*rob->cfg->robotSettings.KickerDampFactor;
            dReal vt = -(vball[0]*dy - vball[1]*dx);
            vx += vn * dx - vt * dy;
            vy += vn * dy + vt * dx;
            dBodySetLinearVel(rob->getBall()->body,vx,vy,vz);
          }

          kickstate = 10;
        }
        void setRoller(int roller) {
          rolling = roller;
        }
        int getRoller() {
          return rolling;
        }
        void toggleRoller() {
          setRoller(!rolling);
        }

        virtual bool isTouchingBall() {
          //dReal vx,vy,vz;
          dReal bx,by,bz;
          dReal kx,ky,kz;
          disc->getBodyPosition(kx,ky,kz);
          rob->getBall()->getBodyPosition(bx,by,bz);
          dReal dist = hypot(kx - bx, ky - by);
          if (dist < (disc->getRadius() + rob->cfg->BallRadius())
              && bz < rob->cfg->robotSettings.KickerHeight) {
            return true;
          } else {
            return false;
          }
        }

        virtual PObject* getKickObject() {
          return disc;
        }
        //< NOTE: you only need to think about the case when an user change the robot manually, i.e. robot is ,parallel to the ground.
        virtual void robotPoseChanged() {
          dReal robotx, roboty, robotz;
          rob->chassis->getBodyPosition(robotx, roboty, robotz);
          disc->setBodyPosition(robotx, roboty, robotz);
          disc->setBodyRotation(0, 0, 1, rob->getDir());
        }
    };


    AttackerRobot() : Robot() {
      std::cout << "Attacker Robot loaded!" << std::endl;
    }
    virtual void initialize(PWorld* world,PBall* ball,ConfigWidget* _cfg,dReal x,dReal y,dReal z,dReal r,dReal g,dReal b,int rob_id,int wheeltexid,int dir) {
      m_r = r;
      m_g = g;
      m_b = b;
      w = world;
      m_ball = ball;
      m_dir = dir;
      cfg = _cfg;
      m_rob_id = rob_id;

      space = w->space;

      chassis = new PCylinder(x,y,z,cfg->robotSettings.RobotRadius,cfg->robotSettings.RobotHeight,cfg->robotSettings.BodyMass*0.99f,r,g,b,rob_id,true);
      chassis->space = space;
      w->addObject(chassis);

      dummy   = new PBall(x,y,z,cfg->robotSettings.RobotCenterFromKicker,cfg->robotSettings.BodyMass*0.01f,0,0,0);
      dummy->setVisibility(false);
      dummy->space = space;
      w->addObject(dummy);

      dummy_to_chassis = dJointCreateFixed(world->world,0);
      dJointAttach (dummy_to_chassis,chassis->body,dummy->body);

      kicker = new AttackerKicker(this);

      drive = new DefaultDrive(this, wheeltexid);
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
