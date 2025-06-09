/* ** *****************************************************************
* clips_bridge.cpp
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file clips_bridge.h
 * Definition of the ClipsBridge class: a base class for a bridge
 * between ROS2 and CLIPS.
 */

#ifndef __CLIPS_BRIDGE_H__
#define __CLIPS_BRIDGE_H__
#pragma once

/** @cond */
#include <memory>
#include <thread>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
/** @endcond */

#include "sync_queue.h"


/**
 * Implements a base class for a bridge between ROS2 and CLIPS
 */
class ClipsBridge{
protected:
	/**
	 * Stores the file that will be loaded into CLIPS during
	 * initialization.
	 */
	std::string clipsFile;

	/**
	 * Specifies the basepath where CLP files are located
	 */
	std::string clppath;

	/**
	 * When true, activates defrule watching during initialization
	 */
	bool flgRules;

	/**
	 * When true, activates fact watching during initialization
	 */
	bool flgFacts;

	/**
	 * Internal flag that keeps the bridge running.
	 * It is set to true by run() until changed to false by stop() or
	 * unless an external event modifies it.
	 */
	bool running;

	/**
	 * The syncrhonous queue used to pass messages between the ROS2 and
	 * the CLIPS threads.
	 * @remark CLIPS functions crash if called from a separate thread.
	 */
	sync_queue<std::string> queue;

	/**
	 * Thread used to asynchronously run the bridge
	 */
	std::thread asyncThread;

	/**
	 * Stores the name of the topic this bridge listens to
	 */
	std::string topicIn;
	/**
	 * Stores the name of the topic where default messages are sent
	 */
	std::string topicOut;
	/**
	 * Stores the name of the topic where the bridge status is published
	 */
	std::string topicStatus;
	/**
	 * Stores the name of the fact where messages received from the
	 * default input topic (topicIn) are asserted.
	 */
	std::string defaultMsgInFact;

	/**
	 * Pointer to a ROS2 node.
	 * Initialized once init is called by the node.
	 */
	std::shared_ptr<rclcpp::Node> nodePtr;
	/**
	 * Stores all publisher objects, accessible by topic name
	 */
	std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers;
	/**
	 * Stores all subscription objects, accessible by topic name
	 */
	std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions;
	/**
	 * Stores all serviceServer objects, accessible by service name
	 */
	// std::unordered_map<std::string, rclcpp::ServiceServer> srvServers;

	/**
	 * Stores the names/tags of the fact asserted when a message arrives
	 */
	std::unordered_map<std::string, std::string> topicFacts;

public:
	/**
	 * Initializes a new instance of ClipsBridge
	 */
	ClipsBridge();
	~ClipsBridge();

	// Disable copy constructor and assignment op.
private:
	/**
	 * Copy constructor disabled
	 */
	ClipsBridge(ClipsBridge const& obj)        = delete;
	/**
	 * Copy assignment operator disabled
	 */
	ClipsBridge& operator=(ClipsBridge const&) = delete;


public:
	/**
	 * Initializes the bridge. Must be called after
	 * @param  argc  main's argc
	 * @param  argv  main's argv
	 * @param  nh    A shared_ptr to a rclcpp::Node used to create subscribers, publishers, etc.
	 * @param  delay Connection stabilization delay in milliseconds.
	 *               The amount of time to wait before initializing
	 *               CLIPS once ROS2 infrastructure has been created.
	 *               Default is 500ms.
	 * @return       true if initialization completed successfully, false otherwise
	 */
	bool init(int argc, char **argv, std::shared_ptr<rclcpp::Node> np, int delay=500);

	/**
	 * Loads a file
	 * @remark       Works only with clp or dat file extensions.
	 *               A dat file contains several clp files.
	 * @param  fpath The path of the file to load
	 * @return       true if the file was loaded successfully, false otherwise
	 */
	bool loadFile(const std::string& fpath);

	/**
	 * Loads a clp file
	 * @param  fpath The path of the file to load
	 * @return       true if the file was loaded successfully, false otherwise
	 */
	bool loadClp(const std::string& fpath);

	/**
	 * Loads a dat file
	 * @param  fpath The path of the file to load
	 * @return       true if the file was loaded successfully, false otherwise
	 */
	bool loadDat(const std::string& fpath);

	/**
	 * Runs the bridge, blocking the calling thread until ROS2 is shutdown
	 */
	void run();

	/**
	 * Runs the bridge in a separate thread, returning immediatelly
	 */
	void runAsync();

	/**
	 * Stops the bridge, regardless of whether it is running synchronously or asynchronously
	 */
	void stop();

	/**
	 * Handles all incomming messages from all subscriptions
	 * @remark      This method is passed to all created subscriber
	 *              objects during the initialization and allows to
	 *              dynamically subscribe to new topics using the
	 *              (rossub topic) CLISP user-function.
	 * @param msg   The received message
	 * @param topic The topic from where the message comes from
	 */
	void subscriptionCallback(const std_msgs::msg::String::SharedPtr msg, const std::string& topic);

protected:

	/**
	 * Asserts the input string as a fact as (assert (network s))
	 * @param s                    The string to be asserted
	 * @param fact                 Optional. The fact under which \p s
	 *                             will be asserted. When empty it
	 *                             defaults to whatever defaultMsgInFact
	 *                             is set. Default: an empty string.
	 * @param resetFactListChanged Optional. When true
	 *                             clips::setFactListChanged(0) is reset,
	 *                             telling clips that no changes to the
	 *                             facts list have been done.
	 *                             Default: true
	 */
	void assertFact(const std::string& s, const std::string& fact = "", bool resetFactListChanged = true);

	/**
	 * Injects a command into CLIPS by calling clisp::sendCommand(s)
	 * @param s The CLIPS command to be injected
	 */
	void sendCommand(const std::string& s);

	/**
	 * Clears CLIPS by calling clips::clear()
	 */
	void clearCLIPS();
	/**
	 * Resets CLIPS by calling clips::reset()
	 */
	void resetCLIPS();
	// std::string& eval(const std::string& s); // Unsupported in 6.0


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
	 * When overriden in a derived class, it must initialize all
	 * subscribers and register them into ClipsBridge::subscribers.
	 */
	virtual void initPublishers();

	/**
	 * Initializes all topic subscribers, including the topic
	 * specified by topicIn.
	 * When overriden in a derived class, it must initialize all
	 * subscribers and register them into ClipsBridge::subscribers.
	 */
	virtual void initServices();

	/**
	 * Initializes all topic subscribers, including the topic
	 * specified by topicIn.
	 * When overriden in a derived class, it must initialize all
	 * subscribers and register them into ClipsBridge::subscribers.
	 */
	virtual void initSubscriptions();

private:
	/**
	 * Parses messages received via topicIn.
	 * Two types of messages are accepted: facts and commands.
	 * Any non-command string is considered a fact and thus is asserted
	 * with ClipsBridge::assertFact().
	 * Commands are string with start with a NULL character ('\\0').
	 * The following commands are supported:
	 * assert      calls clips::assertString()
	 * reset       calls clips::reset()
	 * clear       calls clips::clear()
	 * raw         Injects a code via sendCommand()
	 * print what  Prints facts, rules or agenda
	 * watch what  Toggles the specified watches
	 * load  file  Loads the specified file
	 * run num     Performs the specified number of runs
	 * log         Unimplemented
	 * @param m The received message
	 */
	void parseMessage(const std::string& m);

	/**
	 * Handles commands received via topicIn
	 * @param c The received command message
	 */
	void handleCommand(const std::string& c);

	/**
	 * Unimplemented
	 * @param arg Unimplemented
	 */
	void handleLog(const std::string& arg);

	/**
	 * Handles path request commands received via topicIn
	 * @param path The path where CLP files are
	 */
	void handlePath(const std::string& path);

	/**
	 * Handles print request commands received via topicIn
	 * @param arg What to print. Accepted values are facts, rules
	 *            and agenda.
	 */
	void handlePrint(const std::string& arg);

	/**
	 * Handles run request commands received via topicIn.
	 * On a successful parsing of the argument performs clips::run(arg)
	 * @param arg A string representation of an integer specifying
	 *            the maximum number of run steps to perform
	 */
	void handleRun(const std::string& arg);

	/**
	 * Handles toggle-watch request commands received via topicIn.
	 * On a successful parsing of the argument toggles the watching
	 * of functions, globals, facts or rules
	 * @param arg A string specifying which watch shall be toggled
	 */
	void handleWatch(const std::string& arg);

	/**
	 * Parses command line arguments.
	 * Supported arguments are:
	 * -i   default input topic  (topicIn)
	 * -o   default output topic (topicOut)
	 * -s   default status_topic (topicStatus)
	 * -d   clp base path (where clips files are
	 * -e   File to load upon initialization
	 * -w   Indicates whether to watch facts upon initialization
	 * -r   Indicates whether to watch rules upon initialization
	 * @param  argc The main's argc
	 * @param  argv The main's argv
	 * @return      true if arguments were successfully parsed,
	 *              false otherwise
	 */
	bool parseArgs(int argc, char **argv);

	/**
	 * Prints the default arguments
	 * @param  pname The application name (receives argv[0])
	 */
	void printDefaultArgs(const std::string& pname);

	/**
	 * Prints the bridge's help message
	 * @param  pname The application name (receives argv[0])
	 */
	void printHelp(const std::string& pname);

	/**
	 * Publishes a message to the default output topic (topicOut)
	 * @param  message The message to be published
	 * @return         true if the message was successfully published,
	 *                 false otherwise
	 */
	bool publish(const std::string& message);

	/**
	 * Publishes a message to the specified topic
	 * @remark            This function is used to dynamically
	 *                    publish to any topic from CLIPS using
	 *                    the user-function (rospub topic message)
	 * @param  topicName  The name of the topic to publish to
	 * @param  message    The message to be published
	 * @return            true if the message was successfully published,
	 *                    false otherwise
	 */
	bool publish(const std::string& topicName, const std::string& message);

	/**
	 * Publishes the status of the bridge to topicStatus
	 * @return         true if the status was successfully published,
	 *                 false otherwise
	 */
	bool publishStatus();

	/**
	 * Subscribes to the given topic on the fly
	 * @param  topicName  The name of the tpic to subscribe to
	 * @param  factName   The fact name/tag to be asserted when a new message arrives
	 * @return            true if subscription succeeded, false otherwise
	 */
	bool subscribe(const std::string& topicName, const std::string& factName);

	/**
	 * Prints all facts by calling clips::printFacts()
	 */
	void printFacts();

	/**
	 * Prints all facts by calling clips::printRules()
	 */
	void printRules();

	/**
	 * Prints all facts by calling clips::printAgenda()
	 */
	void printAgenda();

	/**
	 * Returns the node's logger
	 */
	rclcpp::Logger getLogger();


	/**
	 */
	friend void send_message(ClipsBridge& br, const std::string& msg);
// CLIPS_rossub_wrapper)

	/**
	 * Friend function called by the homonymous registered CLIPS user-
	 * function when (rospub topic message) is invoked.
	 * @remark            The function shall return
	 *                    @c br.publish(topicName, message);
	 * @param  br         A reference to this bridge
	 * @param  topicName  The ropic to publish to.
	 * @param  message    The message to publish
	 * @return            1 if the mesage was published, 0 otherwise
	 */
	friend bool bridge_publish_invoker(ClipsBridge& br, const std::string& topicName, const std::string& message);

	/**
	 * Friend function called by the homonymous registered CLIPS user-
	 * function when (rossub topic) is invoked.
	 * @remark            The function shall return
	 *                    @c br.subscribe(topicName, factName);
	 * @param  br         A reference to this bridge
	 * @param  topicName  The topic to subscribe to.
	 * @param  factName   The fact name/tag to be asserted when a new message arrives
	 * @return            1 if the subscription succeeded, 0 otherwise
	 */
	friend bool bridge_subscribe_invoker(ClipsBridge& br, const std::string& topicName, const std::string& factName);
};

#endif // __CLIPS_BRIDGE_H__
