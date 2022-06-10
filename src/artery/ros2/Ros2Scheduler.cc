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

#include "Ros2Scheduler.h"
#include <chrono>
#include <functional>

#include <omnetpp/cfutureeventset.h>
#include <omnetpp/cevent.h>

namespace artery
{

Register_Class(Ros2Scheduler)

using namespace omnetpp;

Ros2Scheduler::Ros2Scheduler() : rosNode(Ros2Node::getInstance())
{
    clockSub = rosNode.getRosNode()->create_subscription<rosgraph_msgs::msg::Clock>("/clock",1,std::bind(&Ros2Scheduler::clock_callback,this,std::placeholders::_1));
}

cEvent* Ros2Scheduler::guessNextEvent()
{
    return sim->getFES()->peekFirst();
}

cEvent* Ros2Scheduler::takeNextEvent()
{
    while (true) {
        cEvent* event = sim->getFES()->peekFirst();
        if(event)
        {
            if (event->isStale()) 
            {
                cEvent* tmp = sim->getFES()->removeFirst();
                ASSERT(tmp == event);
                delete tmp;
            }
            else
            {
                rosNode.spin_some();
                if (event->getArrivalTime() <= gazebo_clock) {
                    return sim->getFES()->removeFirst();
                }
                if(clockSub->get_publisher_count() == 0){
                    throw cTerminationException("Robot simulation not running or stopped.");
                }

            }
        }
        else 
        {
			throw cTerminationException(E_ENDEDOK);
		}
    }
}

void Ros2Scheduler::putBackEvent(cEvent* event)
{
	sim->getFES()->putBackFirst(event);
}

void Ros2Scheduler::startRun()
{
}

void Ros2Scheduler::endRun()
{
}

void Ros2Scheduler::executionResumed()
{
}

void Ros2Scheduler::clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    gazebo_clock = msg->clock.sec+(double)(msg->clock.nanosec/1000000)/1000;
}

} // namespace artery
