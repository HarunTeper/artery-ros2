#ifndef ROS2_ROBOTCREATIONPOLICY_H_FDXRVCJF
#define ROS2_ROBOTCREATIONPOLICY_H_FDXRVCJF

#include "ros2/RobotObject.h"
#include <omnetpp/cmodule.h>
#include <string>

namespace ros2
{

class RobotCreationPolicy
{
public:
    class Instruction
    {
    public:
        void setModuleType(omnetpp::cModuleType* module_type) { m_module_type = module_type; }
        omnetpp::cModuleType* getModuleType() const { return m_module_type; }

        void setSinkPath(const std::string& path) { m_sink_path = path; }
        const char* getSinkPath() const { return m_sink_path.c_str(); }

    private:
        omnetpp::cModuleType* m_module_type = nullptr;
        std::string m_sink_path;
    };

    virtual Instruction getInstruction(const RobotObject&) = 0;
    virtual ~RobotCreationPolicy() = default;
};

} // namespace ros2

#endif /* ROS2_ROBOTCREATIONPOLICY_H_FDXRVCJF */

