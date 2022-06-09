#ifndef ROS2_UNIFORMROBOTCREATIONPOLICY_H_7IYR1XMH
#define ROS2_UNIFORMROBOTCREATIONPOLICY_H_7IYR1XMH

#include "ros2/RobotCreationPolicy.h"
#include <omnetpp/csimplemodule.h>

namespace ros2
{

class UniformRobotCreationPolicy :
    public RobotCreationPolicy, public omnetpp::cSimpleModule
{
public:
    void initialize() override;
    Instruction getInstruction(const RobotObject&) override;

private:
    Instruction m_instruction;
};

} // namespace ros2

#endif /* ROS2_UNIFORMROBOTCREATIONPOLICY_H_7IYR1XMH */
