#ifndef GAZEBOACTIONSI_H
#define GAZEBOACTIONSI_H


#include <jderobot/gazeboActions.h>
#include <turtlebot/turtlebotcontrol.hh>

namespace turtlebot{
namespace interfaces{

class GazeboActionsI : virtual public jderobot::GazeboActions {
public:
    GazeboActionsI(turtlebot::TurtlebotControl *_control);
    virtual ~GazeboActionsI ();
    virtual void resetGazebo(const Ice::Current &c);

protected:
    turtlebot::TurtlebotControl* const control;
};

}}//NS
#endif // GAZEBOACTIONSI_H
