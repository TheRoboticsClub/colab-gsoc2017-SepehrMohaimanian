#include "turtlebot/interfaces/gazeboActionsi.h"

using namespace turtlebot::interfaces;
using namespace jderobot;

GazeboActionsI::GazeboActionsI(gazebo::physics::ModelPtr _model):
    model(_model)
{}

GazeboActionsI::~GazeboActionsI()
{}

void GazeboActionsI::reset()
{
    model->ResetPhysicsStates();
    model->Reset();
    model->ResetTime();
}
