#include "artery/ros2/RobotMiddleware.h"
#include "artery/ros2/RobotInetMobility.h"
#include "artery/utility/InitStages.h"
#include "ros2/RobotObject.h"
#include <inet/common/ModuleAccess.h>

namespace artery
{

Define_Module(RobotMiddleware)

using namespace omnetpp;

namespace
{
const simsignal_t robotPositionChangedSignal = cComponent::registerSignal("robotPositionChanged");
} // namespace

RobotMiddleware::RobotMiddleware() :
    mVehicleDataProvider(0)
{
}

void RobotMiddleware::initialize(int stage)
{
    if (stage == InitStages::Self) {
        mMobility = inet::getModuleFromPar<RobotInetMobility>(par("mobilityModule"), this);
        mMobility->subscribe(robotPositionChangedSignal, this);
        initializeStationType(mMobility->getLastRobotObject().getType());
        mVehicleDataProvider.update(getKinematics(*mMobility));
        getFacilities().register_const(&mVehicleDataProvider);
        getFacilities().register_const(&mMobility->getLastRobotObject());

        auto& robot = getFacilities().getConst<ros2::RobotObject>();
        std::string mRobotId = robot.getId();
        
        // Identity not random anymore, last digits determine robot ID for Ros2
        Identity identity;
        identity.application = 1000000000 + (int)mRobotId.back() - '0';
        mVehicleDataProvider.setStationId(identity.application);
        emit(Identity::changeSignal, Identity::ChangeStationId, &identity);
    }

    Middleware::initialize(stage);
}

void RobotMiddleware::receiveSignal(cComponent*, simsignal_t signal, cObject*, cObject*)
{
    if (signal == robotPositionChangedSignal && mMobility) {
        mVehicleDataProvider.update(getKinematics(*mMobility));
    }
}

void RobotMiddleware::initializeStationType(const std::string& robotType)
{
    using vanetza::geonet::StationType;
    StationType gnStationType = StationType::Unknown;

    if (robotType == "CAR") {
        gnStationType = StationType::Passenger_Car;
    } else if (robotType == "EMERGENCY_VEHICLE") {
        gnStationType = StationType::Special_Vehicle;
    } else if (robotType == "BUS" || robotType == "SCHEDULED_BUS") {
        gnStationType = StationType::Bus;
    } else if (robotType == "VAN") {
        gnStationType = StationType::Light_Truck;
    } else if (robotType == "TRUCK") {
        gnStationType = StationType::Heavy_Truck;
    } else if (robotType == "BICYCLE") {
        gnStationType = StationType::Cyclist;
    } else if (robotType == "PEDESTRIAN") {
        gnStationType = StationType::Pedestrian;
    } else if (robotType == "MOPED") {
        gnStationType = StationType::Moped;
    }

    setStationType(gnStationType);
    mVehicleDataProvider.setStationType(gnStationType);
}

VehicleKinematics RobotMiddleware::getKinematics(const RobotInetMobility& mobility) const
{
    using namespace vanetza::units::si;

    VehicleKinematics kinematics;
    kinematics.position = mobility.getPosition();
    kinematics.geo_position = mobility.getGeoPosition();
    kinematics.speed = mobility.getLastRobotObject().getSpeed() * meter_per_second;
    kinematics.acceleration = mobility.getLastRobotObject().getAcceleration() * meter_per_second_squared;
    kinematics.heading = mobility.getLastRobotObject().getHeadingRad() * radian;
    kinematics.yaw_rate = mobility.getLastRobotObject().getYawRateRad() * radian_per_second;
    return kinematics;
}

} // namespace artery
