#include "ros2/BasicRobotLifecycleController.h"
#include <omnetpp/ccomponenttype.h>
#include <omnetpp/cmessage.h>

namespace ros2
{

Define_Module(BasicRobotLifecycleController)

BasicRobotLifecycleController::BasicRobotLifecycleController() : rosNode(Ros2Node::getInstance())
{
}

BasicRobotLifecycleController::~BasicRobotLifecycleController()
{
}

void BasicRobotLifecycleController::initialize()
{
    m_creation_policy = dynamic_cast<RobotCreationPolicy*>(getSubmodule("creationPolicy"));
    if (!m_creation_policy) {
        throw omnetpp::cRuntimeError("missing RobotCreationPolicy");
    }
    robotNameSub = rosNode.getRosNode()->create_subscription<std_msgs::msg::String>("/artery_model_list",10,std::bind(&BasicRobotLifecycleController::model_callback,this,std::placeholders::_1));
}

void BasicRobotLifecycleController::model_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RobotSink* sink = getSink(msg->data);
    if(!sink)
    {
        RobotObject robot;
        robot.setId(msg->data);
        robot.setType("PASSENGER_CAR");
        robot.setPosition({ 0, 0, 0 });
        robot.setHeading(0);
        robot.setSpeed(0);
        robot.setDriveDirection(0);
        robot.setVehicleLength(0);
        robot.setVehicleWidth(0);
        robot.setAcceleration(0);
        robot.setCurvature(0);
        robot.setYawRate(0);
        RobotSink* sink = getSink(robot.getId());
        if (!sink) 
        {
            createSink(robot);
        }
    }
}

void BasicRobotLifecycleController::handleMessage(omnetpp::cMessage* msg)
{
    // if (auto insertion_msg = dynamic_cast<InsertionMessage*>(msg)) {
    //     auto found = m_pending_robots.find(insertion_msg->getRobotId());
    //     if (found != m_pending_robots.end()) {
    //         ASSERT(found->second);
    //         createSink(*found->second);
    //         m_pending_robots.erase(found);
    //     }
    // }
    delete msg;
}

omnetpp::cModule* BasicRobotLifecycleController::addModule(const std::string& id, omnetpp::cModuleType* type, Initializer& init)
{
    omnetpp::cModule* mod = type->create("robot", getSystemModule(), m_node_index, m_node_index);
    ++m_node_index;
    mod->finalizeParameters();
    mod->buildInside();
    m_nodes[id] = mod;
    init(mod);
    mod->scheduleStart(omnetpp::simTime());
    mod->callInitialize();
    return mod;
}

RobotSink* BasicRobotLifecycleController::getSink(const std::string& id)
{
    auto found = m_robot_sinks.find(id);
    return found != m_robot_sinks.end() ? found->second : nullptr;
}

void BasicRobotLifecycleController::createSink(const RobotObject& obj)
{
    RobotSink* sink = nullptr;
    const RobotCreationPolicy::Instruction& instruction = m_creation_policy->getInstruction(obj);
    Initializer init = [&sink, &instruction, &obj](omnetpp::cModule* mod) {
        sink = dynamic_cast<RobotSink*>(mod->getModuleByPath(instruction.getSinkPath()));
        if (sink) {
            sink->initialize(obj);
        }
    };
    omnetpp::cModule* mod = addModule(obj.getId(), instruction.getModuleType(), init);

    if (sink) {
        auto insertion = m_robot_sinks.insert({ obj.getId(), sink });
        if (!insertion.second) {
            throw omnetpp::cRuntimeError("insertion of Robot sink failed");
        }
    } else {
        EV_ERROR << "could not find Robot sink for module " << mod->getFullPath() << "\n";
    }
}

} // namespace ros2
