#include <stdexcept>

#include "clipsdefenv.h"
#include "contextimpl.h"
#include "udf/udf.h"
#include "udf/retval.h"

extern "C"{
	#include "clips/clips.h"
}

namespace clips{ namespace udf{

/* ** *** *************************************************************
*
* Prototypes
*
** ** *** ************************************************************/
static inline
std::string type2str(const Type& t);

static inline
std::string type2str(const std::vector<Type>& types);

static
void rv2udfv(const RetVal& rv, UDFValue*);

static
void udfWrapper(Environment* env, UDFContext* udfc, UDFValue* out);

/* ** *** *************************************************************
*
* Helpers
*
** ** *** ************************************************************/
static
std::string type2str(const Type& t){
	std::string s;
	if( t == Type::Any) return "*";
	if( (uint16_t)(t & Type::Boolean        ) ) s+= "b";
	if( (uint16_t)(t & Type::Double         ) ) s+= "d";
	if( (uint16_t)(t & Type::ExternalAddress) ) s+= "e";
	if( (uint16_t)(t & Type::FactAddress    ) ) s+= "f";
	if( (uint16_t)(t & Type::InstanceAddress) ) s+= "i";
	if( (uint16_t)(t & Type::Integer        ) ) s+= "l";
	if( (uint16_t)(t & Type::Multifield     ) ) s+= "m";
	if( (uint16_t)(t & Type::InstanceName   ) ) s+= "n";
	if( (uint16_t)(t & Type::String         ) ) s+= "s";
	if( (uint16_t)(t & Type::Symbol         ) ) s+= "y";
	if( (uint16_t)(t & Type::Void           ) ) s+= "v";
	return s;
}

static
std::string type2str(const std::vector<Type>& types){
	std::string s;
	s.reserve(32);
	for(const Type& t : types)
		s+= ";" + type2str(t);
	return s;
}

static
void rv2udfv(const RetVal& rv, UDFValue* udfv){
	if(!udfv) return;
	switch(rv.getType()){
		case Type::Void:    return;
		case Type::Boolean: udfv->lexemeValue  = CreateBoolean(defEnv, rv.getValue().b); return;
		case Type::Double:  udfv->floatValue   = CreateFloat(defEnv, rv.getValue().d);   return;
		case Type::Integer: udfv->integerValue = CreateInteger(defEnv, rv.getValue().l); return;
		case Type::String:  udfv->lexemeValue  = CreateString(defEnv, rv.getValue().s);  return;
	}
}


/* ** *** *************************************************************
*
* Public functions
*
** ** *** ************************************************************/
void addFunction(const std::string& clipsName, const Type& returnTypes, const std::vector<Type>& argTypes,
	std::function<void(Context&, RetVal&)> udf, const std::string& cName, void* context){
	addFunction(
		clipsName, type2str(returnTypes), argTypes.size(), argTypes.size(), type2str(argTypes),
		udf, cName, context
	);
}

void addFunction(const std::string& clipsName, const std::string& returnTypes,
	uint16_t minArgs, uint16_t maxArgs, const std::string& argTypes,
	std::function<void(Context&, RetVal&)> udf, const std::string& cName, void* context){

	if(!defEnv) std::invalid_argument("Clips uninitialized!");
	if(!udf)    std::invalid_argument("No user function provided");
	Context* ctx = new ContextImpl(udf, context);
	std::string ex;
	AddUDFError e = AddUDF(
		defEnv, clipsName.c_str(), returnTypes.c_str(), minArgs, maxArgs, argTypes.c_str(),
		&udfWrapper, cName.empty() ? clipsName.c_str() : cName.c_str(), ctx
	);

	switch(e){
		case AUE_NO_ERROR: return;
		case AUE_FUNCTION_NAME_IN_USE_ERROR:
			ex = "The function name is already in use.";
		case AUE_INVALID_ARGUMENT_TYPE_ERROR:
			ex = "An invalid argument type was specified.";
		case AUE_INVALID_RETURN_TYPE_ERROR:
			ex = "An invalid return type was specified.";
		case AUE_MIN_EXCEEDS_MAX_ERROR:
			ex = "The minimum number of arguments is greater than the maximum number of arguments.";
	}
	delete ctx;
	throw std::invalid_argument(ex);
}


void udfWrapper(Environment* env, UDFContext* udfc, UDFValue* out){
	if(!udfc || !udfc->context) return;
	RetVal rv;
	ContextImpl ctx(udfc);
	auto f = ctx.getFunction();
	f(ctx, rv);
	rv2udfv(rv, out);
}


uint32_t argumentCount(Context& ctx){
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	return UDFArgumentCount( ci.getClpCtx() );
}


bool hasNextArgument(Context& ctx){
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	return UDFHasNextArgument( ci.getClpCtx() );
}


/* ** *****************************************************************
*
* firstArgument overloads
*
** ** *****************************************************************/
bool firstArgument(Context& ctx, double& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFFirstArgument(ci, NUMBER_BITS, &uvout))
		return false;
	out = uvout.floatValue->contents;
	return true;
}


bool firstArgument(Context& ctx, float& out){
	return firstArgument(ctx, (double&)out);
}


bool firstArgument(Context& ctx, int64_t& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFFirstArgument(ci, INTEGER_BIT, &uvout))
		return false;
	out = uvout.integerValue->contents;
	return true;
}


bool firstArgument(Context& ctx, int32_t& out){
	return firstArgument(ctx, (int64_t&)out);
}


bool firstArgument(Context& ctx, int16_t& out){
	return firstArgument(ctx, (int64_t&)out);
}


bool firstArgument(Context& ctx, int8_t& out){
	return firstArgument(ctx, (int64_t&)out);
}


bool firstArgument(Context& ctx, std::string& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFFirstArgument(ci, LEXEME_BITS, &uvout))
		return false;
	out = uvout.lexemeValue->contents;
	return true;
}


/* ** *****************************************************************
*
* nextArgument overloads
*
** ** *****************************************************************/
bool nextArgument(Context& ctx, double& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFNextArgument(ci, NUMBER_BITS, &uvout))
		return false;
	out = uvout.floatValue->contents;
	return true;
}


bool nextArgument(Context& ctx, float& out){
	return nextArgument(ctx, (double&)out);
}


bool nextArgument(Context& ctx, int64_t& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFNextArgument(ci, INTEGER_BIT, &uvout))
		return false;
	out = uvout.integerValue->contents;
	return true;
}


bool nextArgument(Context& ctx, int32_t& out){
	return nextArgument(ctx, (int64_t&)out);
}


bool nextArgument(Context& ctx, int16_t& out){
	return nextArgument(ctx, (int64_t&)out);
}


bool nextArgument(Context& ctx, int8_t& out){
	return nextArgument(ctx, (int64_t&)out);
}


bool nextArgument(Context& ctx, std::string& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFNextArgument(ci, LEXEME_BITS, &uvout))
		return false;
	out = uvout.lexemeValue->contents;
	return true;
}

/* ** *****************************************************************
*
* nthArgument overloads
*
** ** *****************************************************************/
bool nthArgument(Context& ctx, const size_t& index, double& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFNthArgument(ci, index, NUMBER_BITS, &uvout))
		return false;
	out = uvout.floatValue->contents;
	return true;
}


bool nthArgument(Context& ctx, const size_t& index, float& out){
	return nthArgument(ctx, index, (double&)out);
}


bool nthArgument(Context& ctx, const size_t& index, int64_t& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFNthArgument(ci, index, INTEGER_BIT, &uvout))
		return false;
	out = uvout.integerValue->contents;
	return true;
}


bool nthArgument(Context& ctx, const size_t& index, int32_t& out){
	return nthArgument(ctx, index, (int64_t&)out);
}


bool nthArgument(Context& ctx, const size_t& index, int16_t& out){
	return nthArgument(ctx, index, (int64_t&)out);
}


bool nthArgument(Context& ctx, const size_t& index, int8_t& out){
	return nthArgument(ctx, index, (int64_t&)out);
}


bool nthArgument(Context& ctx, const size_t& index, std::string& out){
	UDFValue uvout;
	auto ci = dynamic_cast<ContextImpl&>(ctx);
	if(!UDFNthArgument(ci, index, LEXEME_BITS, &uvout))
		return false;
	out = uvout.lexemeValue->contents;
	return true;
}



}}