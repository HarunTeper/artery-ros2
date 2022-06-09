#ifndef ROS2_BASICROBOTLIFECYCLECONTROLLER_H_FM8ENVOH
#define ROS2_BASICROBOTLIFECYCLECONTROLLER_H_FM8ENVOH

#include "ros2/RobotCreationPolicy.h"
#include "ros2/RobotObject.h"
#include "ros2/RobotSink.h"
#include <omnetpp/clistener.h>
#include <omnetpp/csimplemodule.h>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <ros2/Ros2Node.h>

namespace ros2
{

class BasicRobotLifecycleController : public omnetpp::cListener, public omnetpp::cSimpleModule
{
public:
    BasicRobotLifecycleController();
    virtual ~BasicRobotLifecycleController();

    void initialize() override;
    void handleMessage(omnetpp::cMessage*) override;

protected:
    using Initializer = std::function<void(omnetpp::cModule*)>;

    virtual omnetpp::cModule* addModule(const std::string&, omnetpp::cModuleType*, Initializer&);
    virtual RobotSink* getSink(const std::string&);
    virtual void createSink(const RobotObject&);

private:
    unsigned m_node_index = 0;
    std::map<std::string, omnetpp::cModule*> m_nodes;
    std::map<std::string, RobotSink*> m_robot_sinks;
    std::unordered_map<std::string, std::unique_ptr<RobotObject>> m_pending_robots;
    RobotCreationPolicy* m_creation_policy = nullptr;

    Ros2Node rosNode;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotNameSub;
    void model_callback(const std_msgs::msg::String::SharedPtr msg);
};

} // namespace ros2

#endif /* ROS2_BASICROBOTLIFECYCLECONTROLLER_H_FM8ENVOH */

