#include <cstring>
#include "queryrouter.h"

namespace clips{
/* ** ***************************************************************
*
* Static prototypes for interface with CLIPS
*
** ** **************************************************************/
static bool queryFunction(const std::string& logicalName);
static void writeFunction(const std::string& logicalName, const std::string& str);
static void exitFunction(int exitCode);


/* ** ***************************************************************
*
* QueryRouter class members
*
** ** **************************************************************/

QueryRouter& QueryRouter::getInstance(
			const std::string& routerName,
			clips::RouterPriority priority)
{
	// Guaranteed to be destroyed.
	// Instantiated on first use.
	static QueryRouter instance(routerName, priority);
	return instance;
}


QueryRouter::QueryRouter(const std::string& routerName, clips::RouterPriority priority):
	routerName(routerName), priority(priority),
	registered(false), enabled(false){}

QueryRouter::~QueryRouter(){
	unregisterR();
}


void QueryRouter::enable(){
	if(enabled) return;
	if(!registered) registerR();
	enabled = clips::activateRouter(routerName);
}


void QueryRouter::disable(){
	if(!enabled) return;
	clips::deactivateRouter(routerName);
	enabled = false;
}


bool QueryRouter::isEnabled(){
	return enabled;
}


bool QueryRouter::hasLogicalName(const std::string& ln){
	return logicalNames.count(ln) > 0;
}

void QueryRouter::addLogicalName(const std::string& ln){
	if( !hasLogicalName(ln) )
		logicalNames.insert(ln);
}

void QueryRouter::removeLogicalName(const std::string& ln){
	if( hasLogicalName(ln) )
		logicalNames.erase(ln);
}


std::string QueryRouter::getName(){
	return routerName;
}


clips::RouterPriority QueryRouter::getPriority(){
	return priority;
}


std::string QueryRouter::read(){
	std::string copy(buffer);
	buffer.clear();
	return copy;
}


void QueryRouter::write(const std::string& s){
	buffer+=s;
}


void QueryRouter::registerR(){
	if(registered) return;

	clips::addRouter(routerName,
		priority,       // Priority
		queryFunction,  // Query function
		writeFunction,  // Write function
		NULL,           // Getc function
		NULL,           // Ungetc function
		exitFunction    // Exit function
	);
}


void QueryRouter::unregisterR(){
	if(!registered) return;
	clips::deactivateRouter(routerName);
	clips::deleteRouter(routerName);
}



/* ** ***************************************************************
*
* Static function definitions
*
** ** **************************************************************/

/*
We want to recognize any output that is sent to the logical name
"wtrace" because all tracing information is sent to this logical
name. The recognizer function for our router is defined below.
*/
bool queryFunction(const std::string& logicalName){
	QueryRouter& qr = QueryRouter::getInstance();
	return qr.hasLogicalName(logicalName);
}

/*
We now need to define a function which will print the tracing in-
formation to our trace file. The print function for our router is
defined below.
*/
void writeFunction(const std::string& logicalName, const std::string& str){
	static std::set<std::string> clpln = { "stdin", "stdout", "wclips", "wdialog", "wdisplay", "werror", "wwarning", "wtrace" };

	QueryRouter& qr = QueryRouter::getInstance();
	if(!qr.hasLogicalName(logicalName)) return;
	if(!qr.isEnabled()){
		if(clpln.count(logicalName) > 0)
			clips::print(logicalName, str);
		return;
	}

	qr.write(str);
	if(clpln.count(logicalName) > 0){
		clips::deactivateRouter(qr.getName());
		clips::print(logicalName, str);
		clips::activateRouter(qr.getName());
	}
}

/*
When we exit CLIPS the trace file needs to be closed.
function for our router is defined below.
*/
void exitFunction(int exitCode){}

} // end namespace clips
