#include "artery/ros2/RobotPositionProvider.h"
#include "artery/ros2/RobotInetMobility.h"
#include "artery/networking/Runtime.h"
#include "artery/utility/InitStages.h"
#include <inet/common/ModuleAccess.h>

namespace artery
{

Define_Module(RobotPositionProvider)

using namespace omnetpp;

namespace {
    const simsignal_t positionFixSignal = cComponent::registerSignal("PositionFix");
    const simsignal_t robotPositionChangedSignal = cComponent::registerSignal("robotPositionChanged");
} // namespace


void RobotPositionProvider::initialize(int stage)
{
    if (stage == InitStages::Prepare) {
        mRuntime = inet::getModuleFromPar<Runtime>(par("runtimeModule"), this);
        mMobility = inet::getModuleFromPar<RobotInetMobility>(par("mobilityModule"), this);
        auto& mobilityPar = par("mobilityModule");
        auto* mobilityModule = getModuleByPath(mobilityPar);
    } else if (stage == InitStages::Self) {
        mMobility->subscribe(robotPositionChangedSignal, this);
    } else if (stage == InitStages::Propagate) {
        updatePositionFix();
        omnetpp::createWatch("latitude", mPositionFix.latitude.value());
        omnetpp::createWatch("longitude", mPositionFix.longitude.value());
    }
}

int RobotPositionProvider::numInitStages() const
{
    return InitStages::Total;
}

void RobotPositionProvider::receiveSignal(cComponent*, simsignal_t signal, cObject*, cObject*)
{
    if (signal == robotPositionChangedSignal) {
        updatePositionFix();
    }
}

void RobotPositionProvider::updatePositionFix()
{
    using namespace vanetza::units;
    static const TrueNorth north;

    auto geopos = mMobility->getGeoPosition();
    mPositionFix.timestamp = mRuntime->now();
    mPositionFix.latitude = geopos.latitude;
    mPositionFix.longitude = geopos.longitude;
    mPositionFix.confidence.semi_minor = 5.0 * si::meter;
    mPositionFix.confidence.semi_major = 5.0 * si::meter;
    mPositionFix.course.assign(
            north + GeoAngle { mMobility->getHeading().getTrueNorth() },
            north + 3.0 * degree);
    mPositionFix.speed.assign(
            mMobility->getLastRobotObject().getSpeed() * si::meter_per_second,
            1.0 * si::meter_per_second);

    // prevent signal listeners to modify our position data
    PositionFixObject tmp { mPositionFix };
    emit(positionFixSignal, &tmp);
}

Position RobotPositionProvider::getCartesianPosition() const
{
    return mMobility->getPosition();
}

GeoPosition RobotPositionProvider::getGeodeticPosition() const
{
    return mMobility->getGeoPosition();
}

} // namespace artery
