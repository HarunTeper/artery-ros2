#include "ros2/UniformRobotCreationPolicy.h"
#include <omnetpp/ccomponenttype.h>

namespace ros2
{

Define_Module(UniformRobotCreationPolicy)

void UniformRobotCreationPolicy::initialize()
{
    // one instruction to rule them all ;-)
    m_instruction.setModuleType(omnetpp::cModuleType::get(par("moduleType")));
    m_instruction.setSinkPath(par("robotSinkPath"));
}

RobotCreationPolicy::Instruction UniformRobotCreationPolicy::getInstruction(const RobotObject&)
{
    return m_instruction;
}

} // namespace ros2
