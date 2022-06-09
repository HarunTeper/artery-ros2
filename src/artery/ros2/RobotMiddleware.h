#ifndef ARTERY_ROBOTMIDDLEWARE_H_RNDER8I0
#define ARTERY_ROBOTMIDDLEWARE_H_RNDER8I0

#include "artery/application/Middleware.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/VehicleKinematics.h"

namespace artery
{

class RobotInetMobility;

class RobotMiddleware : public Middleware
{
public:
    RobotMiddleware();
    void initialize(int stage) override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

protected:
    void initializeStationType(const std::string&);
    VehicleKinematics getKinematics(const RobotInetMobility&) const;

private:
    VehicleDataProvider mVehicleDataProvider;
    RobotInetMobility* mMobility = nullptr;
};

} // namespace artery

#endif /* ARTERY_ROBOTMIDDLEWARE_H_RNDER8I0 */

