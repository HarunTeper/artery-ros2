#ifndef SIMULATION_H
#define SIMULATION_H

#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <omnetpp/cobject.h>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class Ros2Node : omnetpp::cObject {
public:
	Ros2Node();
	Ros2Node(std::string name);
	~Ros2Node();

	static Ros2Node &getInstance();

	rclcpp::Node::SharedPtr getRosNode() ;

	void spin_some();
	void runNode();
	void stopNode();

private:
	static Ros2Node instance;

	std::string name;
	rclcpp::Node::SharedPtr nodePtr;
	std::thread* rosThread;
};

#endif /* SIMULATION_H */
