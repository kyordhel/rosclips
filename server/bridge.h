/* ** *****************************************************************
* bridge.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file bridge.h
 * Definition of the Bridge class: A bridge between ROS2 and CLIPS
 * for the rosclips node
 */

#ifndef __BRIDGE_H__
#define __BRIDGE_H__
#pragma once

/** @cond */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
/** @endcond */

#include "clips_bridge.h"
#include "rosclips/srv/query_kdb.hpp"

using QueryKDB = rosclips::srv::QueryKDB;

class ClipsBridge;

/**
 * Implements a specialized bridge for the rosclips node
 *
 * Inherits from ClipsBridge
 */
class Bridge : public ClipsBridge{
public:
	/**
	 * Initializes a new instance of bridge
	 */
	Bridge();

// Disable copy and constructructor and assignment op
private:
	/**
	 * Copy constructor disabled
	 */
	Bridge(Bridge const& obj)        = delete;
	/**
	 * Copy assignment operator disabled
	 */
	Bridge& operator=(Bridge const&) = delete;


// Additional attributes
private:
	/**
	 * The name of the service for querying clips
	 */
	std::string serviceQuery;


// Overriden class members
protected:
	/**
	 * Initializes CLIPS.
	 * It calls clips::initialize(), clips::rerouteStdin(argc, argv)
	 * and clips::clear() in that order before loading the file
	 * specified by ClipsBridge::clips_file.
	 * @param argc The main's argc after being passed to rclcpp::init
	 * @param argv The main's argv after being passed to rclcpp::init
	 */
	virtual void initCLIPS(int argc, char **argv);

	/**
	 * Initializes all topic subscribers, including the topic
	 * specified by topicIn.
	 */
	virtual void initPublishers();

	/**
	 * Initializes all topic subscribers, including the topic
	 * specified by topicIn.
	 */
	virtual void initServices();

	/**
	 * Initializes all topic subscriptions, including the topic
	 * specified by topicIn.
	 */
	virtual void initSubscriptions();

	/**
	 * Services a ROS2 query request
	 * @param  req The request to be served
	 * @param  res The response for the request
	 * @return     true if the service was successfully served, false otherwise.
	 */
	bool srvQueryKDB(QueryKDB::Request& req, QueryKDB::Response& res);

private:
	// Additional class methods go here
	// E.g. special ROS2 callbacks for messages and services
};

#endif // __BRIDGE_H__
