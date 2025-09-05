/* ** *****************************************************************
* main.cpp
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file main.cpp
 * Anchor point (main function) for the clipscontrol node
 */

/** @cond */
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
/** @endcond */

#include "bridge.h"
#include "clipswrapper/udf/udf.h"
#include "clipswrapper/clipswrapper.h"

/* ** ********************************************************
* Global variables
* *** *******************************************************/
/**
 * The bridge to interface with CLIPS
 */
Bridge bridge;

/* ** ********************************************************
* Prototypes
* *** *******************************************************/
int main(int argc, char **argv);
void addUserFunctions();
inline bool bridge_publish_invoker(ClipsBridge& br, std::string const& topicName, std::string const& message);
inline bool bridge_subscribe_invoker(ClipsBridge& br, std::string const& topicName, std::string const& factName);

void CLIPS_rospub_wrapper(clips::udf::Context& ctx, clips::udf::RetVal& rv);
void CLIPS_rossub_wrapper(clips::udf::Context& ctx, clips::udf::RetVal& rv);


/* ** ********************************************************
* Main (program anchor)
* *** *******************************************************/
/**
 * Program anchor
 * @param  argc The number of arguments to the program
 * @param  argv The arguments passed to the program
 * @return      The program exit code
 */
int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	printf("\f\n");
	auto nodePtr = std::make_shared<rclcpp::Node>("rosclips");

	if( !bridge.init(argc, argv, nodePtr) )
		return -1;
	addUserFunctions();
	bridge.runAsync();
	// Give control to ROS
	rclcpp::spin(nodePtr);
	bridge.stop();
	rclcpp::shutdown();
	std::cout << std::endl;
	return 0;
}


/* ** ********************************************************
* Function definitions
* *** *******************************************************/

/**
 * Defines and sets up userfunctions to use within clips scripts
 */
void addUserFunctions(){
	// (rospub ?topic ?str)
	clips::udf::addFunction("rospub", clips::udf::Type::Boolean,
		std::vector<clips::udf::Type>({clips::udf::Type::String, clips::udf::Type::String}),
		&CLIPS_rospub_wrapper, "CLIPS_rospub_wrapper");

	// (rossub ?topic ?fact)
	clips::udf::addFunction("rossub", clips::udf::Type::Boolean,
		std::vector<clips::udf::Type>({clips::udf::Type::String, clips::udf::Type::String}),
		&CLIPS_rossub_wrapper, "CLIPS_rossub_wrapper");
}


/**
 * Publishes the given message (second paramenter) to the specified topic (first parameter).
 * The topic type must be std_msgs::String
 * Wrapper for the CLIPS' rospub function. It calls ClipsBrige::publish via friend-function bridge_publish_invoker.
 * @return Zero if unwrapping was successful, -1 otherwise.
 */
void CLIPS_rospub_wrapper(clips::udf::Context& ctx, clips::udf::RetVal& rv){
	// (rospub ?topic ?str)
	// rv.setValue(false);
	std::string topic, message;
	if(
		(clips::udf::argumentCount(ctx) < 2) ||
		!clips::udf::firstArgument(ctx, topic) ||
		!clips::udf::nextArgument(ctx, message)
	) return;

	boost::trim_right(message);

	/* It sends the data */
	bool success = bridge_publish_invoker(bridge, topic, message);
	// rv.setValue(success);
}

inline
bool bridge_publish_invoker(ClipsBridge& br, std::string const& topicName, std::string const& message){
	return br.publish(topicName, message);
}

/**
 * Subscribes to the specified topic (first parameter). Values are asserted to the specified fact (second parameter)
 * The topic type must be std_msgs::String
 * Wrapper for the CLIPS' rossub function. It calls ClipsBrige::subscribe via friend-function bridge_subscribe_invoker
 * @return Zero if unwrapping was successful, -1 otherwise.
 */
void CLIPS_rossub_wrapper(clips::udf::Context& ctx, clips::udf::RetVal& rv){
	// (rossub ?topic ?fact)
	// (assert (?fact ?str))
	// rv.setValue(false);
	std::string topic, factName;
	if(
		(clips::udf::argumentCount(ctx) < 2) ||
		!clips::udf::firstArgument(ctx, topic) ||
		!clips::udf::nextArgument(ctx, factName)
	) return;

	/* It sends the data */
	bool success = bridge_subscribe_invoker(bridge, topic, factName);
	// rv.setValue(success);
}

inline
bool bridge_subscribe_invoker(ClipsBridge& br, std::string const& topicName, std::string const& factName){
	return br.subscribe(topicName, factName);
}
