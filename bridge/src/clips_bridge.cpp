#include "clips_bridge.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <functional>
#include <unistd.h>

#include "utils.h"
#include "clipswrapper/clipswrapper.h"

using String = std_msgs::msg::String;
using Publisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
using Subscription = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;

#define CMD_MARK 0x02

/* ** ********************************************************
* Macros
* *** *******************************************************/
#define contains(s1,s2) s1.find(s2) != std::string::npos


/* ** ********************************************************
* Local helpers
* *** *******************************************************/
static inline
bool ends_with(const std::string& s, const std::string& end){
	if (end.size() > s.size()) return false;
	return std::equal(end.rbegin(), end.rend(), s.rbegin());
}

static inline
void split_path(const std::string& fpath, std::string& dir, std::string& fname){
	size_t slashp = fpath.rfind("/");
	if(slashp == std::string::npos){
		dir = std::string();
		fname = fpath;
		return;
	}
	dir = fpath.substr(0, slashp);
	fname = fpath.substr(slashp+1);
}

static inline
std::string get_current_path(){
	char buff[FILENAME_MAX];
	getcwd(buff, sizeof(buff));
	return std::string(buff);
}




/* ** ********************************************************
* Constructor
* *** *******************************************************/
ClipsBridge::ClipsBridge():
	// clipsFile("cubes.dat"),
	defaultMsgInFact("network"),
	topicIn("clips_in"), topicOut("clips_out"), topicStatus("clips_status"),
	flgFacts(false), flgRules(false), nodePtr(NULL), clppath(get_current_path()){
}

ClipsBridge::~ClipsBridge(){
	stop();
}


/* ** ********************************************************
*
* Class methods
* Initialization
*
* *** *******************************************************/
bool ClipsBridge::init(int argc, char **argv, std::shared_ptr<rclcpp::Node> np, int delay){
	if( !parseArgs(argc, argv) ) return false;

	this->nodePtr = np;
	initSubscriptions();
	initPublishers();
	initServices();

	std::this_thread::sleep_for(std::chrono::milliseconds(delay));

	clips::initialize();
	clips::rerouteStdin(argc, argv);
	clips::clear();
	publishStatus();

	return true;
}


void ClipsBridge::initCLIPS(int argc, char **argv){
	clips::initialize();
	clips::rerouteStdin(argc, argv);
	clips::clear();
	std::cout << "Clips ready" << std::endl;

	// Load clp files specified in file
	loadFile(clipsFile);
	if(flgFacts) clips::toggleWatch(clips::WatchItem::Facts);
	if(flgRules) clips::toggleWatch(clips::WatchItem::Rules);
	// clips::reset();

	// clips::printFacts();
}


void ClipsBridge::initPublishers(){
	Publisher
	pub = nodePtr->create_publisher<String>(topicOut, 100);
	publishers[topicOut] = pub;

	pub = nodePtr->create_publisher<String>(topicStatus, 1);
	publishers[topicStatus] = pub;
}


void ClipsBridge::initServices(){
}


void ClipsBridge::initSubscriptions(){
	Subscription
	sub = nodePtr->create_subscription<String>(topicIn, 100,
		// boost::bind(&ClipsBridge::subscriptionCallback, this, _1, topicIn)
		// std::bind(&ClipsBridge::subscriptionCallback, this, std::placeholders::_1, topicIn)
		[this](const String::SharedPtr msg){this->subscriptionCallback(msg, topicIn);}
	);
	subscriptions[topicIn] = sub;
}



/* ** ********************************************************
*
* Class methods: Clips wrappers
*
* *** *******************************************************/
void ClipsBridge::assertFact(const std::string& s, const std::string& fact, bool resetFactListChanged) {
	std::string f = fact.empty() ? defaultMsgInFact : fact;
	std::string as = "(" + fact + " " + s + ")";
	clips::assertString( as );
	if(resetFactListChanged)
		clips::setFactListChanged(0);
	RCLCPP_INFO(getLogger(), "Asserted string %s", as.c_str());
}


void ClipsBridge::clearCLIPS(){
	clips::clear();
	RCLCPP_INFO(getLogger(), "KDB cleared (clear)");
}


void ClipsBridge::resetCLIPS(){
	clips::reset();
	RCLCPP_INFO(getLogger(), "KDB reset (reset)");
}


void ClipsBridge::sendCommand(const std::string& s){
	RCLCPP_INFO(getLogger(), "Executing command: %s", s.c_str());
	clips::sendCommand(s);
}


bool ClipsBridge::loadClp(const std::string& fpath){
	RCLCPP_INFO(getLogger(), "Loading file '%s'...", fpath.c_str() );
	if( !clips::load( fpath ) ){
		RCLCPP_ERROR(getLogger(), "Error in file '%s' or does not exist", fpath.c_str());
		return false;
	}
	RCLCPP_INFO(getLogger(), "File %s loaded successfully", fpath.c_str());
	return true;
}


bool ClipsBridge::loadDat(const std::string& fpath){
	if( fpath.empty() ) return false;
	std::ifstream fs;
	fs.open(fpath);

	if( fs.fail() || !fs.is_open() ){
		RCLCPP_ERROR(getLogger(), "File '%s' does not exists", fpath.c_str());
		return false;
	}

	bool err = false;
	std::string line, fdir, fname;
	std::string here = get_current_path();
	split_path(fpath, fdir, fname);
	if(!fdir.empty()) chdir(fdir.c_str());
	RCLCPP_INFO(getLogger(), "Loading '%s'...", fname.c_str());
	while(!err && std::getline(fs, line) ){
		if(line.empty()) continue;
		// size_t slashp = fpath.rfind("/");
		// if(slashp != std::string::npos) line = fdir + line;
		if (!loadClp(line)) err = true;
	}
	fs.close();
	chdir(here.c_str());
	RCLCPP_INFO(getLogger(), err? "Aborted." : "Done.");

	return !err;
}


bool ClipsBridge::loadFile(const std::string& fpath){
	RCLCPP_INFO(getLogger(), "Current path '%s'\n", get_current_path().c_str() );
	if(ends_with(fpath, ".dat"))
		return loadDat(fpath);
	else if(ends_with(fpath, ".clp"))
		return loadClp(fpath);
	return false;
}


/**
 * Parses messages from subscribed topics
 * Re-implements original parse_network_message by Jesús Savage
 * @param m String contained in the topic message
 */
void ClipsBridge::parseMessage(const std::string& m){
	if((m[0] == CMD_MARK) && (m.length()>1)){
		handleCommand(m.substr(1));
		return;
	}
	assertFact( m );
}


static inline
void splitCommand(const std::string& s, std::string& cmd, std::string& arg){
	std::string::size_type sp = s.find(" ");
	if(sp == std::string::npos){
		cmd = s;
		arg.clear();
	}
	else{
		cmd = s.substr(0, sp);
		arg = s.substr(sp+1);
	}
}


void ClipsBridge::handleCommand(const std::string& c){
	std::string cmd, arg;
	splitCommand(c, cmd, arg);

	// RCLCPP_INFO(getLogger(), "Received command %s", c.c_str());
	if(cmd == "assert") { clips::assertString(arg); }
	else if(cmd == "reset") { resetCLIPS(); }
	else if(cmd == "clear") { clearCLIPS(); }
	else if(cmd == "raw")   { sendCommand(arg); }
	else if(cmd == "path")  { handlePath(arg); }
	else if(cmd == "print") { handlePrint(arg); }
	else if(cmd == "watch") { handleWatch(arg); }
	else if(cmd == "load")  { loadFile(arg); }
	else if(cmd == "run")   { handleRun(arg); }
	else if(cmd == "log")   { handleLog(arg); }
	else return;

	// RCLCPP_INFO(getLogger(), "Handled command %s", c.c_str());
}


void ClipsBridge::handleLog(const std::string& arg){
}


void ClipsBridge::handlePath(const std::string& path){
	if(chdir(path.c_str()) != 0){
		RCLCPP_ERROR(getLogger(), "Can't access {%s}: %s\n", path.c_str(), strerror(errno));
		printf("Reset clppath  to {%s}\n", clppath.c_str() );
	}
	clppath = path;
}


void ClipsBridge::handlePrint(const std::string& arg){
	if(arg == "facts"){       clips::printFacts();  }
	else if(arg == "rules"){  clips::printRules();  }
	else if(arg == "agenda"){ clips::printAgenda(); }
}


void ClipsBridge::handleRun(const std::string& arg){
	int n = std::stoi(arg);
	clips::run(n);
}


void ClipsBridge::handleWatch(const std::string& arg){
	if(arg == "functions"){    clips::toggleWatch(clips::WatchItem::Deffunctions); }
	else if(arg == "globals"){ clips::toggleWatch(clips::WatchItem::Globals);      }
	else if(arg == "facts"){   clips::toggleWatch(clips::WatchItem::Facts);        }
	else if(arg == "rules"){   clips::toggleWatch(clips::WatchItem::Rules);        }
	publishStatus();
}



/* ** ********************************************************
*
* Class methods: ROS-related
*
* *** *******************************************************/
bool ClipsBridge::publish(const std::string& message){
	return publish(topicOut, message);
}


bool ClipsBridge::publish(const std::string& topicName, const std::string& message){
	if (publishers.find(topicName) == publishers.end()){
		// Topic not in publishers. Insert.
		Publisher pub = nodePtr->create_publisher<String>( std::string(topicName), 10);

		publishers[topicName] = pub;
		RCLCPP_INFO(getLogger(), "Added publisher for topic %s", topicName.c_str());
	}
	Publisher& pub = publishers[topicName];
	// if (pub.getNumSubscribers() < 1) return false;
	String msg;
	msg.data = message;
	pub->publish(msg);
	// RCLCPP_INFO(getLogger(), "Published <%s> on %s", message.c_str(), pub.getTopic().c_str());
	return true;
}


bool ClipsBridge::publishStatus(){
	std::string status("watching:" + std::to_string((int)clips::getWatches()));
	return publish(topicStatus, status);
}


bool ClipsBridge::subscribe(const std::string& topicName, const std::string& factName){
	if (subscriptions.find(topicName) == subscriptions.end()){
		// Topic not in subscriptions. Insert.
		Subscription sub = nodePtr->create_subscription<String>(topicName, 100,
			// boost::bind(&ClipsBridge::subscriptionCallback, this, _1, topicName)
			// std::bind(&ClipsBridge::subscriptionCallback, this, std::placeholders::_1, topicName)
			[this, topicName](const String::SharedPtr msg){this->subscriptionCallback(msg, topicName);}
		);
		RCLCPP_INFO(getLogger(), "Subscribed to topic %s", topicName.c_str());
		subscriptions[topicName] = sub;
		topicFacts[topicName] = factName;
	}
	return true;
}


/* ** ********************************************************
*
* Class methods: Multithreaded execution
*
* *** *******************************************************/
void ClipsBridge::stop(){
	running = false;
	if(asyncThread.joinable())
		asyncThread.join();
}

void ClipsBridge::runAsync(){
	asyncThread = std::thread(&ClipsBridge::run, this);
}


void ClipsBridge::run(){
	if(running) return;
	running = true;
	// Loop forever
	while(running && rclcpp::ok()){
		if( queue.empty() ){
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			continue;
		}
		parseMessage( queue.consume() );
	}
}



/* ** ********************************************************
*
* Class methods: Callbacks
*
* *** *******************************************************/
void ClipsBridge::subscriptionCallback(const String::SharedPtr msg, const std::string& topic) {
	// printf("[%s (%ldbytes)] %s\n", topic.c_str(), msg->data.length(), msg->data.c_str());
	if(msg->data.length() < 1) return;
	if (topic == topicIn){
		queue.produce( msg->data );
		return;
	}
	if(msg->data[0] == CMD_MARK) return;
	assertFact(msg->data, topicFacts[topic]);
}



/* ** ********************************************************
*
* Class methods: Misc
*
* *** *******************************************************/
rclcpp::Logger ClipsBridge::getLogger(){
	return nodePtr->get_logger();
}

bool ClipsBridge::parseArgs(int argc, char **argv){
	std::string pname(argv[0]);
	pname = pname.substr(pname.find_last_of("/") + 1);
	// Read input parameters
	if (argc <= 1) {
		printDefaultArgs(pname);
		return true;
	}

	for(int i = 1; i < argc; ++i){
		if (!strcmp(argv[i], "-h") || (i+1 >= argc) ){
			printHelp( pname );
			return false;
		}
		else if (!strcmp(argv[i],"-d")){
			clppath = std::string(argv[++i]);

			if(chdir(argv[i]) != 0){
				fprintf(stderr, "Can't access {%s}: %s\n", argv[i], strerror(errno));
				printf("Reset clppath  to {%s}\n", get_current_path().c_str() );
			}
		}
		else if (!strcmp(argv[i],"-e")){
			clipsFile = std::string(argv[++i]);
		}
		else if (!strcmp(argv[i],"-w")){
			flgFacts = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i],"-r")){
			flgRules = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i],"-i")){
			topicIn = std::string(argv[++i]);
		}
		else if (!strcmp(argv[i],"-o")){
			topicOut = std::string(argv[++i]);
		}
		else if (!strcmp(argv[i],"-s")){
			topicStatus = std::string(argv[++i]);
		}
	}
	return true;
}



void ClipsBridge::printDefaultArgs(const std::string& pname){
	std::cout << "Using default parameters:" << std::endl;
	std::cout << "    "   << pname;
	std::cout << " -i "   << topicIn;
	std::cout << " -o "   << topicOut;
	std::cout << " -s "   << topicStatus;
	std::cout << " -d "   << clppath;
	std::cout << " -e "   << ( (clipsFile.length() > 0) ? clipsFile : "''");
	std::cout << " -w "   << flgFacts;
	std::cout << " -r "   << flgRules;
	std::cout << std::endl << std::endl;
}



void ClipsBridge::printHelp(const std::string& pname){
	std::cout << "Usage:" << std::endl;
	std::cout << "    " << pname << " ";
	std::cout << "-i input_topic ";
	std::cout << "-o output_topic ";
	std::cout << "-s status_topic ";
	std::cout << "-d clp base path (where clips files are)";
	std::cout << "-e clipsFile ";
	std::cout << "-w watch_facts ";
	std::cout << "-r watch_rules ";
	std::cout << std::endl << std::endl;
	std::cout << "Example:" << std::endl;
	std::cout << "    " << pname << " -e virbot.dat -w 1 -r 1"  << std::endl;
}

