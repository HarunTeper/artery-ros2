#ifndef ROS2_CORE_QMT3LWQG
#define ROS2_CORE_QMT3LWQG

#include <omnetpp/csimplemodule.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gazebo_msgs/srv/get_model_list.hpp>
#include <std_msgs/msg/string.hpp>

#include <ros2/Ros2Node.h>

namespace ros2
{

class Core : public omnetpp::cSimpleModule
{
public:
    Core();
    virtual ~Core();

    void initialize() override;
    void handleMessage(omnetpp::cMessage*) override;

protected:
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

private:
    omnetpp::cMessage* m_step_event = nullptr;
    omnetpp::SimTime m_step_length;

    Ros2Node rosNode;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub;
};

} // namespace ros2

#endif /* OTS_CORE_QMT3LWQG */

