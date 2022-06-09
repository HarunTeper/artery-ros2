#ifndef ROS2_ROBOTOBJECT_H_Q7PI0NHO
#define ROS2_ROBOTOBJECT_H_Q7PI0NHO

#include <omnetpp/cobject.h>
#include <tuple>

namespace ros2
{

class RobotObject : public omnetpp::cObject
{
public:
    using Position = std::tuple<double, double, double>;
    RobotObject();

    void setId(const std::string& id) { m_id = id; }
    const std::string& getId() const { return m_id; }

    void setType(const std::string& type) { m_type = type; }
    const std::string& getType() const { return m_type; }

    void setPosition(const Position& pos) { m_position = pos; }
    const Position& getPosition() const { return m_position; }

    void setHeading(double h) { m_heading = h; }
    double getHeading() const { return m_heading; }

    double getHeadingRad() const;

    void setSpeed(double v) { m_speed = v; }
    double getSpeed() const { return m_speed; }

    void setDriveDirection(int d) { m_drive_direction = d; }
    double getDriveDirection() const { return m_drive_direction; }

    void setVehicleLength(int l) { m_vehicle_length = l; }
    double getVehicleLength() const { return m_vehicle_length; }

    void setVehicleWidth(int w) { m_vehicle_width = w; }
    double getVehicleWidth() const { return m_vehicle_width; }

    void setAcceleration(double a) { m_acceleration = a; }
    double getAcceleration() const { return m_acceleration; }

    void setCurvature(double c) { m_curvature = c; }
    double getCurvature() const { return m_curvature; }

    void setYawRate(double y) { m_yaw_rate = y; }
    double getYawRate() const { return m_yaw_rate; }

    double getYawRateRad() const;

private:
    std::string m_id;
    std::string m_type;
    Position m_position;
    double m_heading;
    double m_speed;
    int m_drive_direction;
    int m_vehicle_length;
    int m_vehicle_width;
    double m_acceleration;
    double m_curvature;
    double m_yaw_rate;
};

} // namespace ros2

#endif /* ROS2_ROBOTOBJECT_H_Q7PI0NHO */

