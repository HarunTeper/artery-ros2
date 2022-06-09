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

#ifndef ARTERY_ROS2SCHEDULER_H_
#define ARTERY_ROS2SCHEDULER_H_

#include <omnetpp/cmodule.h>
#include <omnetpp/cscheduler.h>
#include <omnetpp/simtime.h>

#include <chrono>
#include <memory>

#include "ros2/Ros2Node.h"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace artery
{

class Ros2Task;

class Ros2Scheduler : public omnetpp::cScheduler
{
	public:
		Ros2Scheduler();
		omnetpp::cEvent* guessNextEvent() override;
		omnetpp::cEvent* takeNextEvent() override;
		void putBackEvent(omnetpp::cEvent*) override;

		void endRun() override;
		void startRun() override;
		void executionResumed() override;

		void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

	protected:

	private:
		Ros2Node rosNode;
		rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub;
		omnetpp::SimTime gazebo_clock;
};

} // namespace artery

#endif /* ARTERY_ROS2SCHEDULER_H_ */
