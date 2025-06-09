#include "clipswrapperrouter.h"
#include "clipsdefenv.h"

extern "C"{
	#include "clips/clips.h"
	#include "clips/router.h"
}

/* ** ***************************************************************
*
* Helpers
*
** ** **************************************************************/
typedef struct{
	Environment* env;
	void* ctx;
	bool (*qf)(const std::string&);
	void (*wf)(const std::string&,const std::string&);
	int  (*rf)(const std::string&);
	int  (*uf)(const std::string&, int);
	void (*ef)(int);
} DefaultContext;

static inline
char* clipsstr(const std::string& s){
	return s.length() ? (char*)s.c_str() : NULL;
}

namespace clips{

static bool defaultQueryFunction(Environment*, const char*, void*);
static void defaultWriteFunction(Environment*, const char*,const char*, void*);
static int  defaultReadFunction(Environment*, const char*, void*);
static int  defaultUnreadFunction(Environment*, const char*, int, void*);
static void defaultExitFunction(Environment*, int, void*);



/* ** ***************************************************************
*
* Router functions
*
** ** **************************************************************/
bool addRouter(const std::string& routerName, RouterPriority priority,
	bool (*queryFunction)(Environment*, const char*, void*),
	void (*writeFunction)(Environment*, const char*,const char*, void*),
	int  (*readFunction)(Environment*, const char*, void*),
	int  (*unreadFunction)(Environment*, const char*, int, void*),
	void (*exitFunction)(Environment*, int, void*)
){
	// auto qf = [queryFunction](Environment*, char* ln, void*)-> int{ return queryFunction(ln); };
	return AddRouter(defEnv, routerName.c_str(), (int)priority,
		queryFunction,
		writeFunction,
		readFunction,
		unreadFunction,
		exitFunction,
		NULL);
}


bool activateRouter(const std::string& routerName){
	return ActivateRouter( defEnv, clipsstr(routerName) );
}

bool deactivateRouter(const std::string& routerName){
	return DeactivateRouter( defEnv, clipsstr(routerName) );
}

bool deleteRouter(const std::string& routerName){
	return DeleteRouter( defEnv, clipsstr(routerName) );
}

bool addRouter(const std::string& routerName, RouterPriority priority,
	bool (*queryFunction)(const std::string&),
	void (*writeFunction)(const std::string&, const std::string&),
	int  (*readFunction)(const std::string&),
	int  (*unreadFunction)(const std::string&, int),
	void (*exitFunction)(int)
){
	DefaultContext *dc = new DefaultContext();
		dc->env=defEnv;      dc->ctx=NULL;          dc->qf=queryFunction; dc->wf=writeFunction;
		dc->rf=readFunction; dc->uf=unreadFunction; dc->ef=exitFunction;
		return AddRouter(defEnv, routerName.c_str(), (int)priority,
		&defaultQueryFunction,
		&defaultWriteFunction,
		&defaultReadFunction,
		&defaultUnreadFunction,
		&defaultExitFunction,
		dc);
}

/* ** ***************************************************************
*
* Parameter discarding functions
*
** ** **************************************************************/
bool defaultQueryFunction(Environment* env, const char* ln, void* ctx){
	return ((DefaultContext*)ctx)->qf(ln);
}

void defaultWriteFunction(Environment* env, const char* ln, const char* str, void* ctx){
	((DefaultContext*)ctx)->wf(ln, str);
}

int  defaultReadFunction(Environment* env, const char* ln, void* ctx){
	return ((DefaultContext*)ctx)->rf(ln);
}

int  defaultUnreadFunction(Environment* env, const char* ln, int ch, void* ctx){
	return ((DefaultContext*)ctx)->uf(ln, ch);
}

void defaultExitFunction(Environment* env, int ec, void* ctx){
	return ((DefaultContext*)ctx)->ef(ec);
}

} // end namespace