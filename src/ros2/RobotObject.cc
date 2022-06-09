#include <ros2/RobotObject.h>
#include <omnetpp.h>
#include <cmath>
#include <limits>

namespace ros2
{

Register_Abstract_Class(RobotObject)

namespace
{

const double nan = std::numeric_limits<double>::quiet_NaN();
const double pi = std::acos(-1.0);

} // namespace

RobotObject::RobotObject() :
    m_position(nan, nan, nan), m_heading(nan), m_acceleration(nan)
{
}

double RobotObject::getHeadingRad() const
{
    return m_heading*M_PI/180;
}

double RobotObject::getYawRateRad() const
{
    return m_yaw_rate*M_PI/180;
}

} // namespace ros2
