#include "turtlebot/interfaces/gazeboActionsi.h"

using namespace turtlebot::interfaces;
using namespace jderobot;

GazeboActionsI::GazeboActionsI( turtlebot::TurtlebotControl *_control):
  control(_control)
{}

GazeboActionsI::~GazeboActionsI()
{}

void GazeboActionsI::resetGazebo(Ice::Current const & c)
{
    control->resetModel();
}
