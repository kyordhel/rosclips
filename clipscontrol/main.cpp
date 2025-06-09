/* ** *****************************************************************
* main.cpp
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file clipscontrol/main.cpp
 * Anchor point (main function) for the clipscontrol node
 */

/** @cond */
#include <memory>
#include <regex>
#include <thread>
#include <iostream>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rosclips/srv/query_kdb.hpp"

/** @endcond */

#include "ncurseswin.h"


using namespace clipscontrol;
// using namespace ROSPACKAGE ;
using std::placeholders::_1;
using String = std_msgs::msg::String;
using Publisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
using Subscription = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
typedef std::shared_ptr<NCursesWin> NCursesWinPtr;

/* ** ********************************************************
* Global variables
* *** *******************************************************/

/**
 * A ros::Publisher object that publishes to the topic rosclips listens to
 */
Publisher pub;

/**
 * The topic rosclips listens to
 */
std::string topicClipsIn     = "clips_in";

/**
 * The topic to read from rosclips (or any topic to listen to)
 */
std::string topicClipsOut    = "clips_out";

/**
 * The topic where rosclips reports its status
 */
std::string topicClipsStatus = "clips_status";


/**
 * The name of the service for querying clips
 */
std::string serviceQuery     = "/clips/query";

/**
 * The service client used to perform queries
 */
// ros::ServiceClient cliQueryCLIPS; // Requires nodehandle



/* ** ********************************************************
* Prototypes
* *** *******************************************************/
void sendToCLIPS(const std::string& s);
void asyncNcwTask(NCursesWinPtr ncw);
void cloutSubsCallback(const String::SharedPtr msg, const std::string& topic, NCursesWinPtr ncw);
void clstatSubsCallback(const String::SharedPtr msg, NCursesWinPtr ncw);
void asyncCheckClipsOnlineTask(NCursesWinPtr ncw, const Subscription sub);
// bool queryHandler(const std::string& query, std::string& result);

NCursesWinPtr ncwPtr;
/* ** ********************************************************
* Main (program anchor)
* *** *******************************************************/
/**
 * Program anchor
 * @param  argc The number of arguments to the program
 * @param  argv The arguments passed to the program
 * @return      The program exit code
 */
int main(int argc, char** argv){

	// Creates and initializes gui
	ncwPtr = std::make_shared<NCursesWin>();
	rclcpp::init(argc, argv);

	// Setup ROS
	auto nodePtr = std::make_shared<rclcpp::Node>("clipscontrol");
	rclcpp::Rate rate(10);
	// The topic to write/send commands to
	pub = nodePtr->create_publisher<String>(topicClipsIn, 10);
	// The topic to read clps output from
	auto subclo = nodePtr->create_subscription<String>(topicClipsOut, 10,
		[](String::SharedPtr msg){ cloutSubsCallback(msg, topicClipsOut, ncwPtr); }
		// std::bind(cloutSubsCallback, _1, topicClipsOut, &ncw)
	);
	// The topic to read status from
	auto subcls = nodePtr->create_subscription<String>(topicClipsStatus, 1,
		[](String::SharedPtr msg){ clstatSubsCallback(msg, ncwPtr); }
		// std::bind(clstatSubsCallback, _1, &ncw)
	);
	// The service client used to perform queries
	// cliQueryCLIPS = nodePtr->serviceClient<QueryKDB>(serviceQuery);


	// boost::bind(&ClipsBridge::subscriberCallback, this, _1, topicName)
	pubfunc pf(sendToCLIPS);
	ncwPtr->addPublisher(pf);
	// queryfunc qf(queryHandler);
	// ncwPtr->addQueryHandler(qf);

	std::thread ncwThread = std::thread(asyncNcwTask, ncwPtr);
	std::thread ccoThread = std::thread(asyncCheckClipsOnlineTask, ncwPtr, subcls);
	rclcpp::spin(nodePtr);
	ncwPtr->exitPoll();
	ncwThread.join();
	// ccoThread.join();
	return 0;
}


/* ** ********************************************************
* Function definitions
* *** *******************************************************/
/**
 * Callback for the rosclips clipsout topic subscription.
 * It will print the message in the main window of the GUI.
 * @param msg    The received message
 * @param topic  The topic from where the message comes from
 * @param ncw    The GUI main window to update
 */
void cloutSubsCallback(const String::SharedPtr msg,
	                   const std::string& topic, NCursesWinPtr ncw){
	// ncw->print("["+topic+"]: " + msg->data);
	if(msg->data.back() != '\n')
		ncw->print(msg->data + "\n");
	else
		ncw->print(msg->data);
}


/**
 * Callback for the rosclips status topic subscription.
 * It will update the watch flags in the main window of the GUI
 * @param msg The received message
 * @param ncw The GUI main window to update
 */
void clstatSubsCallback(const String::SharedPtr msg, NCursesWinPtr ncw){
	static std::regex rxWatch("watching:(\\d+)");

	int flags;
	std::smatch match;

	if (!regex_search(msg->data, match, rxWatch))
		return;
	flags = std::stoi(match.str(1));
	ncw->setWatchFlags(flags);
}


/**
 * Sends the given strng to rosclips
 * @param s The string to send
 */
void sendToCLIPS(const std::string& s){
	String msg;
	msg.data = s;
	pub->publish(msg);
}


/**
 * Polls the GUI main window. Executed in a separated thread.
 * @param ncw The GUI main window
 */
void asyncNcwTask(NCursesWinPtr ncw){
	ncw->poll();
	std::cout << "GUI terminated. Awaiting for ROS..." << std::endl;
	rclcpp::shutdown();
	std::cout << "Done." << std::endl;
}


/**
 * Prdiodically checks whether rosclips is online
 * @param ncw The GUI main window
 * @param sub The ros::Subscriber object listening to the rosclips status topic
 */
void asyncCheckClipsOnlineTask(NCursesWinPtr ncw, const Subscription sub){
	do{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// ncw->setCLIPSStatus(sub->getNumPublishers() > 0 ?
		ncw->setCLIPSStatus(pub->get_subscription_count() > 0 ?
			NCursesWin::CLIPSStatus::Online : NCursesWin::CLIPSStatus::Offline);
	}
	while( rclcpp::ok() );
}

/*
bool queryHandler(const std::string& query, std::string& result){
	if(!ros::service::waitForService(serviceQuery, 500))
		return false;
	QueryKDB rpc;
	rpc.request.query = query;
	if (!cliQueryCLIPS.call(rpc)) return false;
	result = rpc.response.result;
	return true;
}
*/
