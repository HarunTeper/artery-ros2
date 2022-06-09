#include "ros2/Core.h"
#include "ros2/RobotObject.h"
#include <boost/functional/hash.hpp> // for hashing sim0mq::Identifier's strings
#include <omnetpp/cmessage.h>
#include <array>
#include <fstream>
#include <sstream>

namespace ros2
{

Define_Module(Core)

namespace
{

using namespace omnetpp;

} // namespace

Core::Core() : rosNode(Ros2Node::getInstance())
{
    m_step_event = new omnetpp::cMessage("Ros2 step");
}

Core::~Core()
{
}

void Core::initialize()
{
    scheduleAt(omnetpp::simTime(), m_step_event);
    m_step_length = par("stepLength");
}

void Core::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == m_step_event) {
        scheduleAt(omnetpp::simTime() + m_step_length, m_step_event);
    }
}

} // namespace ros2
