#ifndef ARTERY_ROBOTINETMOBILITY_H_T7FXXEAE
#define ARTERY_ROBOTINETMOBILITY_H_T7FXXEAE

#include "artery/utility/Geometry.h"
#include "ros2/RobotSink.h"
#include <inet/mobility/contract/IMobility.h>
#include <omnetpp/csimplemodule.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <etsi_its_msgs/msg/cam.hpp>
#include <auna_its_msgs/msg/cam.hpp>

#include <ros2/Ros2Node.h>

namespace inet {
    class CanvasProjection;
    class IGeographicCoordinateSystem;
} // inet

namespace artery
{

class RobotInetMobility :
    public inet::IMobility, public ros2::RobotSink,
    public omnetpp::cSimpleModule
{
public:
    RobotInetMobility();
    virtual ~RobotInetMobility();

    void initialize(int stage) override;
    int numInitStages() const override;

    // inet::IMobility interface
    double getMaxSpeed() const override;
    inet::Coord getCurrentPosition() override;
    inet::Coord getCurrentSpeed() override;
    inet::EulerAngles getCurrentAngularPosition() override;
    inet::EulerAngles getCurrentAngularSpeed() override;
    inet::Coord getConstraintAreaMax() const override;
    inet::Coord getConstraintAreaMin() const override;

    // ros2::RobotSink
    void initialize(const ros2::RobotObject&) override;
    void update(const ros2::RobotObject&) override;

    const ros2::RobotObject& getLastRobotObject() const { return mLastRobotObject; }
    const ros2::RobotObject& getLastSimulationRobotObject() const { return mLastSimulationRobotObject; }
    Position getPosition() const;
    GeoPosition getGeoPosition() const;
    double getSpeed() const;
    Angle getHeading() const;
    Angle getYawRate() const;

protected:
    virtual void setInetProperties(const ros2::RobotObject&);
    virtual void updateVisualRepresentation();

private:
    ros2::RobotObject mLastRobotObject;
    ros2::RobotObject mLastSimulationRobotObject;
    inet::Coord mSimulationPosition;
    inet::Coord mPosition;
    inet::Coord mSpeed;
    inet::EulerAngles mOrientation;
    inet::EulerAngles mYawRate;
    omnetpp::cModule* mVisualRepresentation = nullptr;
    const inet::CanvasProjection* mCanvasProjection = nullptr;
    const inet::IGeographicCoordinateSystem* mCoordinateSystem = nullptr;

    Ros2Node rosNode;
    rclcpp::Subscription<etsi_its_msgs::msg::CAM>::SharedPtr localizationSub;
    rclcpp::Subscription<auna_its_msgs::msg::CAM>::SharedPtr simulationSub;
    void localization_callback(const etsi_its_msgs::msg::CAM::SharedPtr);
    void simulation_callback(const auna_its_msgs::msg::CAM::SharedPtr);
};

} // namespace artery

#endif /* ARTERY_ROBOTINETMOBILITY_H_T7FXXEAE */
