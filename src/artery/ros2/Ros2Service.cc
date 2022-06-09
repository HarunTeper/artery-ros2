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

#include "artery/ros2/Ros2Service.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>

#include <inet/common/ModuleAccess.h>
#include <inet/common/geometry/common/CanvasProjection.h>
#include <inet/common/geometry/common/CoordinateSystem.h>
#include <inet/features.h>

#include <ros2/RobotObject.h>
#include "artery/utility/IdentityRegistry.h"
#include "artery/envmod/sensor/CamSensor.h"

using namespace omnetpp;
using namespace vanetza;

namespace artery
{

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

Define_Module(Ros2Service)

Ros2Service::Ros2Service() : rosNode(Ros2Node::getInstance())
{
}

Ros2Service::~Ros2Service()
{
	cancelAndDelete(m_self_msg);
}

void Ros2Service::indicate(const btp::DataIndication& ind, cPacket* packet, const NetworkInterface& net)
{
	Enter_Method("indicate");

	if (packet->getByteLength() == 42) {
		EV_INFO << "packet indication on channel " << net.channel << "\n";
	}

	delete(packet);
}

void Ros2Service::initialize()
{
	ItsG5Service::initialize();
	m_self_msg = new cMessage("Example Service");
	subscribe(scSignalCamReceived);

	scheduleAt(simTime() + 3.0, m_self_msg);

	mCoordinateSystem = inet::getModuleFromPar<inet::IGeographicCoordinateSystem>(par("coordinateSystemModule"), this);
    
    auto& robot = getFacilities().getConst<ros2::RobotObject>();
    std::string mRobotId = robot.getId();

	publisher = rosNode.getRosNode()->create_publisher<etsi_its_msgs::msg::CAM>(mRobotId+"/camRX", 10);
}

void Ros2Service::finish()
{
	// you could record some scalars at this point
	ItsG5Service::finish();
}

void Ros2Service::handleMessage(cMessage* msg)
{
	Enter_Method("handleMessage");

	if (msg == m_self_msg) {
		EV_INFO << "self message\n";
	}
}

void Ros2Service::trigger()
{
	Enter_Method("trigger");

	// use an ITS-AID reserved for testing purposes
	static const vanetza::ItsAid example_its_aid = 16480;

	auto& mco = getFacilities().get_const<MultiChannelPolicy>();
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();

	for (auto channel : mco.allChannels(example_its_aid)) {
		auto network = networks.select(channel);
		if (network) {
			btp::DataRequestB req;
			// use same port number as configured for listening on this channel
			req.destination_port = host_cast(getPortNumber(channel));
			req.gn.transport_type = geonet::TransportType::SHB;
			req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
			req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
			req.gn.its_aid = example_its_aid;

			cPacket* packet = new cPacket("Example Service Packet");
			packet->setByteLength(42);

			// send packet on specific network interface
			request(req, packet, network.get());
		} else {
			EV_ERROR << "No network interface available for channel " << channel << "\n";
		}
	}
}

void Ros2Service::receiveSignal(cComponent* source, simsignal_t signal, cObject* object, cObject* details)
{
	Enter_Method("receiveSignal");

    auto& robot = getFacilities().getConst<ros2::RobotObject>();
    std::string mRobotId = robot.getId();
	int identity = (int)mRobotId.back() - '0';

	if (auto cam = dynamic_cast<CaObject*>(object)) {
		const auto id = cam->asn1()->header.stationID%1000000000;

		//only send message if cam received from preceding vehicle
		if(identity == id+1)
		{
			inet::GeoCoord geoPosition{
				(double)cam->asn1()->cam.camParameters.basicContainer.referencePosition.latitude/10000000,
				(double)cam->asn1()->cam.camParameters.basicContainer.referencePosition.longitude/10000000,
				(double)cam->asn1()->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue/10000000
			};
			inet::Coord rosCoord = mCoordinateSystem->computePlaygroundCoordinate(geoPosition);

			etsi_its_msgs::msg::CAM msg;

			msg.header.stamp = rosNode.getRosNode()->get_clock()->now();
			msg.header.frame_id = mRobotId;
			
			msg.its_header.message_id = msg.its_header.MESSAGE_ID_CAM;

			msg.station_type.value = msg.station_type.PASSENGER_CAR;

			msg.reference_position.longitude = std::round(rosCoord.x); //0.1m
			msg.reference_position.latitude = std::round(-rosCoord.y); //0.1m //flip y-axis
			msg.reference_position.altitude.value = 0; //cam->asn1()->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue; //0.01m

			msg.high_frequency_container.heading.value = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue;//0.1 degree
			msg.high_frequency_container.speed.value = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue;//0.01 m/s
			msg.high_frequency_container.drive_direction.value = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection;
			msg.high_frequency_container.vehicle_length.value = 0.49*10;
			msg.high_frequency_container.vehicle_width.value = 0.18*10;
			msg.high_frequency_container.longitudinal_acceleration.value = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue*10;
			msg.high_frequency_container.curvature.value = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue;
			msg.high_frequency_container.yaw_rate.value = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue;//0.01degree/s

			publisher->publish(msg);
		}
	}
}

} // namespace artery
