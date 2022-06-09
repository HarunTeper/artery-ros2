/*
 * Simulation.cpp
 *
 *  Created on: 18. 11. 2015
 *      Author: Vladimir Matena
 */

#include "Ros2Node.h"

#include <string>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

Ros2Node Ros2Node::instance;

Ros2Node::Ros2Node() {
	rclcpp::init(0,nullptr);
	nodePtr = rclcpp::Node::make_shared("rosNode");
	// rosThread = new std::thread([this](){rclcpp::spin(nodePtr);});
}

Ros2Node::Ros2Node(std::string nodeName) {
	// const char* args[1] = { nodeName.c_str() }; 
	// rclcpp::init(1,args);
	rclcpp::init(0,nullptr);
	nodePtr = rclcpp::Node::make_shared(nodeName);
	// rosThread = new std::thread([this](){rclcpp::spin(nodePtr);});
}

Ros2Node::~Ros2Node() {
}

Ros2Node& Ros2Node::getInstance() {
	return instance;
}

rclcpp::Node::SharedPtr Ros2Node::getRosNode() {
	return nodePtr;
}

void Ros2Node::spin_some(){
	rclcpp::spin_some(this->getRosNode());
}

void Ros2Node::runNode(){
}

void Ros2Node::stopNode() {
	rclcpp::shutdown();
	rosThread->join();
}

