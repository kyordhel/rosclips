/* ** ***************************************************************
* clipswrapper.cpp
*
* Author: Mauricio Matamoros
*
* Wrapper for most used CLIPS functions required/used by the bridge
*
** ** **************************************************************/

#include <map>
#include <stack>
#include "clipsdefenv.h"
#include "clipswrapper.h"

extern "C"{
	#include "clips/clips.h"
	#include "clips/pprint.h"
	#include "clips/prcdrfun.h"
}

typedef WatchItem ClipsWatchItem;

/* ** ***************************************************************
*
* Helpers
*
** ** **************************************************************/
static inline
char* clipsstr(const std::string& s){
	return s.length() ? (char*)s.c_str() : NULL;
}


namespace clips{
Environment* defEnv = NULL;

std::map<clips::WatchItem,ClipsWatchItem> watchItems = {
	{clips::WatchItem::Facts,            ClipsWatchItem::FACTS},
	{clips::WatchItem::Rules,            ClipsWatchItem::RULES},
	{clips::WatchItem::Activations,      ClipsWatchItem::ACTIVATIONS},
	{clips::WatchItem::Focus,            ClipsWatchItem::FOCUS},
	{clips::WatchItem::Compilations,     ClipsWatchItem::COMPILATIONS},
	{clips::WatchItem::Statistics,       ClipsWatchItem::STATISTICS},
	{clips::WatchItem::Globals,          ClipsWatchItem::GLOBALS},
	{clips::WatchItem::Deffunctions,     ClipsWatchItem::DEFFUNCTIONS},
	{clips::WatchItem::Instances,        ClipsWatchItem::INSTANCES},
	{clips::WatchItem::Slots,            ClipsWatchItem::SLOTS},
	{clips::WatchItem::Messages,         ClipsWatchItem::MESSAGES},
	{clips::WatchItem::MessageHandlers,  ClipsWatchItem::MESSAGE_HANDLERS},
	{clips::WatchItem::GenericFunctions, ClipsWatchItem::GENERIC_FUNCTIONS},
	{clips::WatchItem::Methods,          ClipsWatchItem::METHODS},
};


bool isValidClipsString(std::string const& str){
	if((str.length() < 1) || (str[0] != '('))
		return false;

	size_t i;
	std::stack<char> stack;
	for(i = 0; i < str.length(); ++i){
		if(str[i] == '\\'){ ++i; continue; }
		if(str[i] == '('){ stack.push(str[i]); continue; }
		if(str[i] == ')'){
			if(stack.empty()) return false;
			stack.pop();
		}
	}
	return stack.empty();
}


const std::string version(){
	return VERSION_STRING;
}


int run(int maxRules){
	return Run(defEnv, maxRules);
}

void initialize(){
	if(!defEnv)	defEnv = CreateEnvironment();
}

void rerouteStdin(int argc, char** argv){
	RerouteStdin(defEnv, argc, argv);
}

void clear(){
	Clear(defEnv);
}

void reset(){
	Reset(defEnv);
}

bool getFactListChanged(){
	return GetFactListChanged(defEnv);
}

void setFactListChanged(const bool changed){
	SetFactListChanged(defEnv, changed);
}

void assertString(const std::string& s){
	AssertString( defEnv, clipsstr(s) );
}


void printAgenda(
	const std::string& logicalName,
	const std::string& module){
	Defmodule* m = module.empty() ? GetCurrentModule(defEnv) : FindDefmodule(defEnv, module.c_str());
	Agenda( defEnv, clipsstr(logicalName), m );
}


void printFacts(
	const std::string& logicalName,
	const std::string& module,
	size_t start, size_t end, size_t max){
	Defmodule* m = module.empty() ? GetCurrentModule(defEnv) : FindDefmodule(defEnv, module.c_str());
	Facts( defEnv, clipsstr(logicalName), m, start, end, max);
}


void printDefrules(
	const std::string& logicalName,
	const std::string& module){
	Defmodule* m = module.empty() ? NULL : FindDefmodule(defEnv, module.c_str());
	ListDefrules( defEnv, clipsstr(logicalName), m );
}


bool print(const std::string& ln, const std::string& str){
	return print( clipsstr(ln), clipsstr(str) );
}


std::vector<std::string> getDefruleList(std::string const& module){
	CLIPSValue out;
	struct multifield* theList;
	std::vector<std::string> rules;

	Defmodule* m = module.empty() ? NULL : FindDefmodule(defEnv, module.c_str());

	GetDefruleList(defEnv, &out, m);
	if( !(theList = (struct multifield *)out.multifieldValue) )
		return rules;
	if(theList->header.type != MULTIFIELD_TYPE) return rules;

	for(int i = 0; theList && i <= theList->length; ++i) {
		CLIPSLexeme* item = theList->contents[i].lexemeValue;
		if(item->header.type != SYMBOL_TYPE) continue; // Check STRING_TYPE
		rules.push_back((char*)(item->contents));
	}
	return rules;
}


bool load(std::string const& fpath){
	return Load( defEnv, clipsstr(fpath) ) == LE_NO_ERROR;
}


void sendCommandRaw(std::string const& s, bool verbose){
	// Resets the pretty print save buffer.
	FlushPPBuffer(defEnv);
	// Sets PPBufferStatus flag to boolean
	// value of ON or OFF
	SetPPBufferStatus(defEnv, false);
	// Processes a completed command
	RouteCommand( defEnv, clipsstr(s), verbose );
	// Returns the EvaluationError flag
	int res = GetEvaluationError(defEnv);
	// Resets the pretty print save buffer.
	FlushPPBuffer(defEnv);
	// Sets the HaltExecution flag
	SetHaltExecution(defEnv, false);
	// Sets the EvaluationError flag
	SetEvaluationError(defEnv, false);
	// Removes all variables from the list
	// of currently bound local variables.
	FlushBindList(defEnv, NULL);
}

bool sendCommand(std::string const& s, bool verbose){
	if(!isValidClipsString(s)) return false;
	sendCommandRaw(s, verbose);
	return true;
}


bool query(const std::string& query, std::string& result){
	int steps;
	return clips::query(query, result, steps) && steps > 0;
}


bool query(const std::string& query, std::string& result, int& steps){
	static QueryRouter& qr = QueryRouter::getInstance();
	qr.enable();
	if( !clips::sendCommand(query, true) ) return false;
	steps = clips::run();
	result = qr.read();
	qr.disable();
	return true;
}


bool watch(const WatchItem& item){
	if((int)(item & WatchItem::All)){
		Watch(defEnv, ClipsWatchItem::ALL);
		return true;
	}
	if((int)(item & WatchItem::Facts))            Watch(defEnv, ClipsWatchItem::FACTS);
	if((int)(item & WatchItem::Rules))            Watch(defEnv, ClipsWatchItem::RULES);
	if((int)(item & WatchItem::Activations))      Watch(defEnv, ClipsWatchItem::ACTIVATIONS);
	if((int)(item & WatchItem::Focus))            Watch(defEnv, ClipsWatchItem::FOCUS);
	if((int)(item & WatchItem::Compilations))     Watch(defEnv, ClipsWatchItem::COMPILATIONS);
	if((int)(item & WatchItem::Statistics))       Watch(defEnv, ClipsWatchItem::STATISTICS);
	if((int)(item & WatchItem::Globals))          Watch(defEnv, ClipsWatchItem::GLOBALS);
	if((int)(item & WatchItem::Deffunctions))     Watch(defEnv, ClipsWatchItem::DEFFUNCTIONS);
	if((int)(item & WatchItem::Instances))        Watch(defEnv, ClipsWatchItem::INSTANCES);
	if((int)(item & WatchItem::Slots))            Watch(defEnv, ClipsWatchItem::SLOTS);
	if((int)(item & WatchItem::Messages))         Watch(defEnv, ClipsWatchItem::MESSAGES);
	if((int)(item & WatchItem::MessageHandlers))  Watch(defEnv, ClipsWatchItem::MESSAGE_HANDLERS);
	if((int)(item & WatchItem::GenericFunctions)) Watch(defEnv, ClipsWatchItem::GENERIC_FUNCTIONS);
	if((int)(item & WatchItem::Methods))          Watch(defEnv, ClipsWatchItem::METHODS);
	return true;
}


bool unwatch(const WatchItem& item){
	if((int)(item & WatchItem::All)){
		Unwatch(defEnv, ClipsWatchItem::ALL);
		return true;
	}
	if((int)(item & WatchItem::Facts))            Unwatch(defEnv, ClipsWatchItem::FACTS);
	if((int)(item & WatchItem::Rules))            Unwatch(defEnv, ClipsWatchItem::RULES);
	if((int)(item & WatchItem::Activations))      Unwatch(defEnv, ClipsWatchItem::ACTIVATIONS);
	if((int)(item & WatchItem::Focus))            Unwatch(defEnv, ClipsWatchItem::FOCUS);
	if((int)(item & WatchItem::Compilations))     Unwatch(defEnv, ClipsWatchItem::COMPILATIONS);
	if((int)(item & WatchItem::Statistics))       Unwatch(defEnv, ClipsWatchItem::STATISTICS);
	if((int)(item & WatchItem::Globals))          Unwatch(defEnv, ClipsWatchItem::GLOBALS);
	if((int)(item & WatchItem::Deffunctions))     Unwatch(defEnv, ClipsWatchItem::DEFFUNCTIONS);
	if((int)(item & WatchItem::Instances))        Unwatch(defEnv, ClipsWatchItem::INSTANCES);
	if((int)(item & WatchItem::Slots))            Unwatch(defEnv, ClipsWatchItem::SLOTS);
	if((int)(item & WatchItem::Messages))         Unwatch(defEnv, ClipsWatchItem::MESSAGES);
	if((int)(item & WatchItem::MessageHandlers))  Unwatch(defEnv, ClipsWatchItem::MESSAGE_HANDLERS);
	if((int)(item & WatchItem::GenericFunctions)) Unwatch(defEnv, ClipsWatchItem::GENERIC_FUNCTIONS);
	if((int)(item & WatchItem::Methods))          Unwatch(defEnv, ClipsWatchItem::METHODS);
	return true;
}


WatchItem toggleWatch(const WatchItem& item){
	int result = 0;
	for(const auto& kv : watchItems){
		if(!(int)(item & kv.first)) continue;
		ClipsWatchItem wi = kv.second;
		if( GetWatchState(defEnv, wi) )
			Unwatch(defEnv, kv.second);
		else
			Watch(defEnv, kv.second); break;
		if( GetWatchState(defEnv, wi) )
			result|= (int)kv.first;
	}
	return (WatchItem)result;
}


WatchItem getWatches(){
	int result = 0;
	for(const auto& kv : watchItems){
		ClipsWatchItem wi = kv.second;
		if( GetWatchState(defEnv, wi) == 1 )
			result|= (int)kv.first;
	}
	return (WatchItem)result;
}

inline
bool watching(const WatchItem& item){
	return (int)getWatches() & (int)item;
}



/*
bool defineFunction_impl(const std::string& functionName,
	const char& returnType,
	int (*functionPointer)(),
	const std::string& actualFunctionName,
	const std::string& restrictions
){
	if(restrictions.length())
		return defineFunction(defEnv, clipsstr(functionName), returnType, functionPointer,
			clipsstr(actualFunctionName), clipsstr(restrictions));
	return defineFunction(defEnv, clipsstr(functionName), returnType, functionPointer, clipsstr(actualFunctionName));
}
*/

[[deprecated]]
bool defineFunction_impl(const std::string& functionName,
	const char& returnType,
	int (*functionPointer)(),
	const std::string& actualFunctionName,
	const std::string& restrictions
){
	return false;
}

} // end namespace
