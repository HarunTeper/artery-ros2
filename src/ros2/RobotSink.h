#ifndef ROS2_ROBOTSINK_H_2RZI4UCS
#define ROS2_ROBOTSINK_H_2RZI4UCS

#include "ros2/RobotObject.h"

namespace ros2
{

class RobotSink
{
public:
    virtual void initialize(const RobotObject&) = 0;
    virtual void update(const RobotObject&) = 0;
    virtual ~RobotSink() = default;
};

} // namespace ros2

#endif /* ROS2_ROBOTSINK_H_2RZI4UCS */

