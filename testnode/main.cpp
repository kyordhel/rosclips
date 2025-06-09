/** @file main.cpp
* @author Mauricio Matamoros
*
* Anchor point (main function) for the test node
*
*/

/** @cond */
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
/** @endcond */

#define CMD_MARK 0x02

using String = std_msgs::msg::String;
using Publisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
using Subscription = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
typedef std::shared_ptr<rclcpp::Node> NodePtr;

/* ** ********************************************************
* Global variablesn
* *** *******************************************************/
/**
 * The node handle
 */
NodePtr np;

/**
 * A rclcpp::Publisher object that publishes to the topic rosclips listens to
 */
Publisher pub;

/**
 * A rclcpp::Subscriber object to listen rosclips messages
 */
Subscription sub;

/**
 * The topic rosclips listens to
 */
std::string topicClipsIn     = "clips_in";

/**
 * The topic to read from rosclips (or any topic to listen to)
 */
std::string topicClipsOut    = "clips_out";



/* ** ********************************************************
* Prototypes
* *** *******************************************************/
int main(int argc, char **argv);
void setupROS(int argc, char **argv);
void reqLoadFile(const std::string& file);
void reqReset();
void reqRun();
void reqAssert(const std::string& fact);
void reqRetract(const std::string& fact);
void publish(const std::string& s);
void subCallback(const String::SharedPtr msg);
static inline void sleep_ms(size_t ms);


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
	setupROS(argc, argv);

	// Spin ROS asynchronously
	std::thread rosspinThread = std::thread( [&]{rclcpp::spin(np);} );

	reqLoadFile("cubes.clp");
	reqReset();
	reqRun();

	printf("\n\nCool, right!? Let's retry adding another cube...\n");
	reqReset();
	reqAssert("(block G)");
	reqAssert("(on-top-of (upper nothing)(lower G))");
	reqAssert("(on-top-of (upper G)(lower D))");
	reqRun();

	printf("\n\nNow with yet another cube...\n");
	reqReset();
	printf("\nUpon reset, we lost G, so let's add it again...\n");
	reqAssert("(block G)");
	reqAssert("(on-top-of (upper nothing)(lower G))");
	reqAssert("(on-top-of (upper G)(lower D))");
	printf("\nNow with H...\n");
	reqAssert("(block H)");
	reqAssert("(on-top-of (upper nothing)(lower H))");
	reqAssert("(on-top-of (upper H)(lower A))");
	reqRun();

	printf("All done. Shutting down... ");
	rclcpp::shutdown();
	rosspinThread.join();
	printf("Bye!\n");
	return 0;
}


/* ** ********************************************************
* Function definitions
* *** *******************************************************/
void setupROS(int argc, char **argv){
	rclcpp::init(argc, argv);
	np = std::make_shared<rclcpp::Node>("test");

	pub = np->create_publisher<String>(topicClipsIn, 10);
	sub = np->create_subscription<String>(topicClipsOut, 10, subCallback);

	rclcpp::Rate loop_rate(10);
}


/**
 * Requests rosclips to load the specified file
 * @param file Path to file to load
 */
void reqLoadFile(const std::string& file){
	printf("Press enter to load %s", file.c_str());
	while (std::cin.get() != '\n');
	publish( "(clear)" );
	sleep_ms( 250 );
	publish( "(load cubes.clp)" );
}


/**
 * Requests rosclips to reset the KDB
 */
void reqReset(){
	printf("Press enter to reset CLIPS");
	while (std::cin.get() != '\n');
	publish( "(reset)" );
}


/**
 * Requests rosclips to run the KDB engine
 */
void reqRun(){
	printf("Press enter to run CLIPS");
	while (std::cin.get() != '\n');
	publish( "(run -1)" );
	sleep_ms(1000);
}


/**
 * Requests rosclips to assert a fact.
 */
void reqAssert(const std::string& fact){
	printf("Press enter to assert %s", fact.c_str());
	while (std::cin.get() != '\n');
	publish( "(assert " + fact + ")" );
}


/**
 * Requests rosclips to retract a fact.
 */
void reqRetract(const std::string& fact){
	printf("Press enter to retract %s", fact.c_str());
	while (std::cin.get() != '\n');
	publish( "(retract " + fact + ")" );
}


/**
 * Sends the given strng to rosclips
 * @param s The string to send
 */
void publish(const std::string& s){
	std::string prefix;
	prefix.push_back((char)CMD_MARK);
	prefix+= "raw ";

	String msg;
	msg.data = prefix + s;
	pub->publish(msg);
}

void subCallback(const String::SharedPtr msg){
	printf("CLIPS: %s\n", msg->data.c_str() );
}

/**
 * Sleeps the current execution thread for the specified amount of time
 * @param ms The amount of time in milliseconds
 */
static inline void sleep_ms(size_t ms){
	std::this_thread::sleep_for( std::chrono::milliseconds(ms) );
}
