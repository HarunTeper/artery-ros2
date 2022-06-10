#include "artery/ros2/RobotInetMobility.h"
#include <inet/common/ModuleAccess.h>
#include <inet/common/geometry/common/CanvasProjection.h>
#include <inet/common/geometry/common/CoordinateSystem.h>
#include <inet/features.h>
#include <cmath>

#ifdef WITH_VISUALIZERS
#   include <inet/visualizer/mobility/MobilityCanvasVisualizer.h>
#else
#   include <cstdio>
#endif

namespace artery
{

Define_Module(RobotInetMobility)

namespace
{

using namespace omnetpp;
const simsignal_t robotPositionChangedSignal = cComponent::registerSignal("robotPositionChanged");

} // namespace

RobotInetMobility::RobotInetMobility() : rosNode(Ros2Node::getInstance())
{
}

RobotInetMobility::~RobotInetMobility()
{
}

int RobotInetMobility::numInitStages() const
{
    return inet::INITSTAGE_PHYSICAL_ENVIRONMENT_2 + 1;
}

void RobotInetMobility::initialize(int stage)
{
    if (stage == inet::INITSTAGE_LOCAL) {
        mVisualRepresentation = inet::getModuleFromPar<cModule>(par("visualRepresentation"), this, false);
        mCoordinateSystem = inet::getModuleFromPar<inet::IGeographicCoordinateSystem>(par("coordinateSystemModule"), this);
        omnetpp::createWatch("position", mPosition);
        omnetpp::createWatch("simulationPosition", mSimulationPosition);
        omnetpp::createWatch("speed", mSpeed);
        omnetpp::createWatch("orientation", mOrientation);
        omnetpp::createWatch("ROBOT ID", mLastRobotObject.getId());
        localizationSub = rosNode.getRosNode()->create_subscription<etsi_its_msgs::msg::CAM>(mLastRobotObject.getId()+"/camTX",1,std::bind(&RobotInetMobility::localization_callback,this,std::placeholders::_1));
        simulationSub = rosNode.getRosNode()->create_subscription<ros_its_msgs::msg::CAM>(mLastRobotObject.getId()+"/simulation_pose",1,std::bind(&RobotInetMobility::simulation_callback,this,std::placeholders::_1));
    } else if (stage == inet::INITSTAGE_PHYSICAL_ENVIRONMENT_2) {
        if (mVisualRepresentation) {
            auto target = mVisualRepresentation->getParentModule();
            mCanvasProjection = inet::CanvasProjection::getCanvasProjection(target->getCanvas());
        }
        updateVisualRepresentation();
    }
}

void RobotInetMobility::localization_callback(const etsi_its_msgs::msg::CAM::SharedPtr msg)
{
    ros2::RobotObject robot;
    robot.setId(mLastRobotObject.getId());
    robot.setType("PASSENGER_CAR");
    robot.setPosition({msg->reference_position.longitude, msg->reference_position.latitude, 0.0 });
    // Scale back the received heading value to [0,360]
    robot.setHeading((double)msg->high_frequency_container.heading.value/10);
    // Scale back the received speed value to m/s
    robot.setSpeed((double)msg->high_frequency_container.speed.value/100);
    robot.setDriveDirection(msg->high_frequency_container.drive_direction.value);
    robot.setVehicleLength(msg->high_frequency_container.vehicle_length.value);
    robot.setVehicleWidth(msg->high_frequency_container.vehicle_width.value);
    // Scale the acceleration value to m/s^2
    robot.setAcceleration((double)msg->high_frequency_container.longitudinal_acceleration.value/10);
    robot.setCurvature(msg->high_frequency_container.curvature.value);
    // Scale the yaw rate value to degree/s
    robot.setYawRate((double)msg->high_frequency_container.yaw_rate.value/100);

    mLastRobotObject = robot;
    std::tie(mPosition.x, mPosition.y, mPosition.z) = robot.getPosition();
    mPosition.y *= -1.0; // flix y-axis (positive is up in Ros vs. negative is up in OMNeT++/INET)

    emit(robotPositionChangedSignal, &robot);
}

void RobotInetMobility::simulation_callback(const ros_its_msgs::msg::CAM::SharedPtr msg)
{
    ros2::RobotObject robot;
    robot.setId(mLastRobotObject.getId());
    robot.setType("PASSENGER_CAR");
    robot.setPosition({msg->x, msg->y, 0.0 });
    robot.setHeading((double)msg->theta);
    robot.setSpeed((double)msg->v);
    robot.setDriveDirection(msg->drive_direction);
    robot.setVehicleLength(msg->vehicle_length);
    robot.setVehicleWidth(msg->vehicle_width);
    robot.setAcceleration((double)msg->vdot);
    robot.setCurvature(msg->curv);
    robot.setYawRate((double)msg->thetadot);

    update(robot);
}

double RobotInetMobility::getMaxSpeed() const
{
    return NaN;
}

inet::Coord RobotInetMobility::getCurrentPosition()
{
    return mSimulationPosition;
}

inet::Coord RobotInetMobility::getCurrentSpeed()
{
    return mSpeed;
}

inet::EulerAngles RobotInetMobility::getCurrentAngularPosition()
{
    return mOrientation;
}

inet::EulerAngles RobotInetMobility::getCurrentAngularSpeed()
{
    return mYawRate;
}

inet::Coord RobotInetMobility::getConstraintAreaMax() const
{
    return inet::Coord::NIL;
}

inet::Coord RobotInetMobility::getConstraintAreaMin() const
{
    return inet::Coord::NIL;
}

void RobotInetMobility::initialize(const ros2::RobotObject& robot)
{
    setInetProperties(robot);
    mLastRobotObject = robot;
}

void RobotInetMobility::update(const ros2::RobotObject& robot)
{
    setInetProperties(robot);
    emit(inet::IMobility::mobilityStateChangedSignal, this);
    updateVisualRepresentation();
    mLastSimulationRobotObject = robot;
}

void RobotInetMobility::setInetProperties(const ros2::RobotObject& robot)
{
    const double headingRad = robot.getHeadingRad();
    const inet::Coord direction { std::cos(headingRad), -std::sin(headingRad) };
    mOrientation.alpha = -headingRad;
    mYawRate.alpha = robot.getYawRate();
    mSpeed = direction * pow(-1,robot.getDriveDirection()) * robot.getSpeed();
    std::tie(mSimulationPosition.x, mSimulationPosition.y, mSimulationPosition.z) = robot.getPosition();
    mSimulationPosition.y *= -1.0; // flix y-axis (positive is up in Ros vs. negative is up in OMNeT++/INET)
}

void RobotInetMobility::updateVisualRepresentation()
{
    // following code is taken from INET's MobilityBase::updateVisualRepresentation
    if (hasGUI() && mVisualRepresentation) {
#ifdef WITH_VISUALIZERS
        using inet::visualizer::MobilityCanvasVisualizer;
        MobilityCanvasVisualizer::setPosition(mVisualRepresentation, mCanvasProjection->computeCanvasPoint(getCurrentPosition()));
#else
        auto position = mCanvasProjection->computeCanvasPoint(getCurrentPosition());
        char buf[32];
        snprintf(buf, sizeof(buf), "%lf", position.x);
        buf[sizeof(buf) - 1] = 0;
        mVisualRepresentation->getDisplayString().setTagArg("p", 0, buf);
        snprintf(buf, sizeof(buf), "%lf", position.y);
        buf[sizeof(buf) - 1] = 0;
        mVisualRepresentation->getDisplayString().setTagArg("p", 1, buf);
#endif
    }
}

Position RobotInetMobility::getPosition() const
{
    return Position {mPosition.x, mPosition.y};
}

GeoPosition RobotInetMobility::getGeoPosition() const
{
    inet::GeoCoord inet = mCoordinateSystem->computeGeographicCoordinate(mPosition);
    GeoPosition pos;
    pos.latitude = inet.latitude * boost::units::degree::degree;
    pos.longitude = inet.longitude * boost::units::degree::degree;
    return pos;
}

double RobotInetMobility::getSpeed() const
{
    return double { mLastRobotObject.getSpeed() };
}

Angle RobotInetMobility::getHeading() const
{
    return Angle { mLastRobotObject.getHeadingRad() };
}

Angle RobotInetMobility::getYawRate() const
{
    return Angle { mLastRobotObject.getYawRateRad() };
}

} // namespace artery
