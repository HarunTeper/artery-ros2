//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef ROS2SERVICE_H_
#define ROS2SERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/NetworkInterface.h"

#
#include <rclcpp/rclcpp.hpp>
#include <etsi_its_msgs/msg/cam.hpp>

#include <ros2/Ros2Node.h>

namespace inet {
    class CanvasProjection;
    class IGeographicCoordinateSystem;
} // inet

namespace artery
{

class Ros2Service : public ItsG5Service
{
    public:
        Ros2Service();
        ~Ros2Service();

        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*, const NetworkInterface&) override;
        void trigger() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

    protected:
        void initialize() override;
        void finish() override;
        void handleMessage(omnetpp::cMessage*) override;

    private:
        omnetpp::cMessage* m_self_msg;
        const inet::IGeographicCoordinateSystem* mCoordinateSystem = nullptr;

        Ros2Node rosNode;
        rclcpp::Publisher<etsi_its_msgs::msg::CAM>::SharedPtr publisher;
};

} // namespace artery

#endif /* ROS2SERVICE_H_ */
