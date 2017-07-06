#ifndef GAZEBOACTIONSI_H
#define GAZEBOACTIONSI_H


#include <jderobot/gazeboActions.h>
#include <gazebo/common/common.hh>

namespace turtlebot{
namespace interfaces{

class GazeboActionsI : public jderobot::GazeboActions {
public:
    GazeboActionsI ( gazebo::physics::ModelPtr _model);
    virtual ~GazeboActionsI ();

    void reset();
private:
    gazebo::physics::ModelPtr model;
};

}}//NS
#endif // GAZEBOACTIONSI_H
