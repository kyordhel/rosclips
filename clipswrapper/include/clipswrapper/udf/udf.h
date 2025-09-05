/* ** ***************************************************************
* clipswrapperudf.h
*
* Author: Mauricio Matamoros
*
* Wrapper for CLIPS' 6.4 User Defined Function helpers
*
** ** **************************************************************/

#ifndef __CLIPSWRAPPERUDF_H__
#define __CLIPSWRAPPERUDF_H__
#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <functional>

#include "type.h"

namespace clips{ namespace udf{

class Context;
class RetVal;

/**
 * Registers a function to be used from CLIPS
 *
 * A User Defined Function (UDF) must be registered with CLIPS using addFunction before it can be referenced from CLIPS.
 * A call should be made for every function which is to be integrated with CLIPS. The user’s source code then can be compiled and linked
 * with CLIPS. The only restrictions is that it must be called after CLIPS has been initialized and before the UDF is referenced.
 * @param  clipsName   The name associated with the UDF when it is called from within CLIPS
 * @param  returnTypes The types returned by the UDF
 * @param  argTypes    A vector containing the allowed of the for arguments
 * @param  cfp         The function to be invoked by CLIPS
 * @param  cName       The name of the UDF as specified in the C source code
 * @param  context     A user supplied pointer to data that is passed to the UDF when it is invoked through the UDFContext paramter,
 *                     if any. Defaults to NULL.
 */
void addFunction(
	const std::string& clipsName, const Type& returnTypes, const std::vector<Type>& argTypes,
	std::function<void(Context&, RetVal&)> udf, const std::string& cName, void* context=NULL);

/**
 * Registers a function to be used from CLIPS
 *
 * A User Defined Function (UDF) must be registered with CLIPS using addFunction before it can be referenced from CLIPS.
 * A call should be made for every function which is to be integrated with CLIPS. The user’s source code then can be compiled and linked
 * with CLIPS. The only restrictions is that it must be called after CLIPS has been initialized and before the UDF is referenced.
 * @param  clipsName   The name associated with the UDF when it is called from within CLIPS
 * @param  returnTypes A string containing character codes indicating the CLIPS types returned by the UDF
 * @param  minArgs     The minimum number of arguments that must be passed to the UDF
 * @param  maxArgs     The maximum number of arguments that may be passed to the UDF
 * @param  argTypes    A string containing one or more groups of character codes specifying the allowed types for arguments
 * @param  cfp         The function to be invoked by CLIPS
 * @param  cName       The name of the UDF as specified in the C source code
 * @param  context     A user supplied pointer to data that is passed to the UDF when it is invoked through the UDFContext paramter,
 *                     if any. Defaults to NULL.
 */
void addFunction(
	const std::string& clipsName, const std::string& returnTypes,
	uint16_t minArgs, uint16_t maxArgs, const std::string& argTypes,
	std::function<void(Context&, RetVal&)> udf, const std::string& cName, void* context=NULL);


/**
 * Returns the number of arguments passed to the UDF.
 * At the point the UDF is invoked, the argument count has been verified
 * to fall within the range specified by the minimum and maximum number of
 * arguments specified in the call to AddUDF. Thus a UDF should only need
 * to check the argument count if the minimum and maximum number of arguments
 * are not the same.
 *
 * @param  c   The execution context provided by clips that contains the given arguments
 * @return     The number of arguments passed to the UDF
 */
uint32_t argumentCount(Context& ctx);

/**
 * Retrieves the first argument passed to the User Defined Function
 * @param  c            The execution context provided by clips that contains the given arguments
 * @param  expectedType The expcted types of the argument. Can be OR-ed
 * @param  out          A pointer to a UDFValue in which the retrieved argument value is stored
 *                      if the argument was successfully retrieved and the expected type matches.
 * @return              true if the argument was successfully retrieved and is the expected type; false otherwise
 */
// bool firstArgument(Context *c, Type expectedType, Value* out);

/**
 * Retrieves the first argument passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with double.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool firstArgument(Context&, double& out);
/**
 * Retrieves the first argument passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with float.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool firstArgument(Context& ctx, float& out);
/**
 * Retrieves the first argument passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int64_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool firstArgument(Context& ctx, int64_t& out);
/**
 * Retrieves the first argument passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int32_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool firstArgument(Context& ctx, int32_t& out);
/**
 * Retrieves the first argument passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int16_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool firstArgument(Context& ctx, int16_t& out);
/**
 * Retrieves the first argument passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int8_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool firstArgument(Context& ctx, int8_t& out);
/**
 * Retrieves the first argument passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with std::string.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool firstArgument(Context& ctx, std::string& out);

/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param  c            The execution context provided by clips that contains the given arguments
 * @param  expectedType The expcted types of the argument. Can be OR-ed
 * @param  out          A pointer to a UDFValue in which the retrieved argument value is stored
 *                      if the argument was successfully retrieved and the expected type matches.
 * @return              true if the argument was successfully retrieved and is the expected type; false otherwise
 */
// bool nextArgument(Context *c, Type expectedType, Value* out);

/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with double.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nextArgument(Context& ctx, double& out);
/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with float.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nextArgument(Context& ctx, float& out);
/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int64_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nextArgument(Context& ctx, int64_t& out);
/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int32_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nextArgument(Context& ctx, int32_t& out);
/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int16_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nextArgument(Context& ctx, int16_t& out);
/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int8_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nextArgument(Context& ctx, int8_t& out);
/**
 * Retrieves the argument following the previously retrieved argument (either from firstArgument,
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with std::string.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nextArgument(Context& ctx, std::string& out);

/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * @param c             The execution context provided by clips that contains the given arguments
 * @param index         The ONE-base index of the argument to be retrieved (indices start with one)
 * @param expectedType  The expcted types of the argument. Can be OR-ed
 * @param out           A pointer to a UDFValue in which the retrieved argument value is stored
 *                      if the argument was successfully retrieved and the expected type matches.
 * @return              true if the argument was successfully retrieved and is the expected type; false otherwise
 */
// bool nthArgument(Context *c, size_t index, Type expectedType, Value* out);

/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param index    The zero-based index of the argument to be retrieved
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with double.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nthArgument(Context& ctx, const size_t& index, double& out);
/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param index    The zero-based index of the argument to be retrieved
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with float.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nthArgument(Context& ctx, const size_t& index, float& out);
/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param index    The zero-based index of the argument to be retrieved
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int64_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nthArgument(Context& ctx, const size_t& index, int64_t& out);
/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param index    The zero-based index of the argument to be retrieved
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int32_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nthArgument(Context& ctx, const size_t& index, int32_t& out);
/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param index    The zero-based index of the argument to be retrieved
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int16_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nthArgument(Context& ctx, const size_t& index, int16_t& out);
/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param index    The zero-based index of the argument to be retrieved
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with int8_t.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nthArgument(Context& ctx, const size_t& index, int8_t& out);
/**
 * Retrieves a specific, index-based argument passed to the User Defined Function
 * nextArgument, or nthArgument) passed to the User Defined Function
 * @param ctx      The execution context provided by CLIPS that contains the arguments
 * @param index    The zero-based index of the argument to be retrieved
 * @param out      When this function returns contains the retrieved value iif this
 *                 was successfully retrieved and its type is compatible with std::string.
 * @return         true if the argument was successfully retrieved and is the expected type; false otherwise
 */
bool nthArgument(Context& ctx, const size_t& index, std::string& out);

/**
 * Checks whether there is an argument available to be retrieved otherwise, it returns false.
 *
 * The “next” argument is considered to be the first argument if no previous call to firstArgument,
 * nextArgument, or nthArgument has been made; otherwise it is the next argument following the
 * most recent call to one of those functions.
 * @param  c   The execution context provided by clips that contains the given arguments
 * @return     true if there is an argument is available to be retrieved; false otherwise
 */
bool hasNextArgument(Context& ctx);
// void UDFThrowError(Context *c);

/**
 * Creates a primitive value with a type field value of SYMBOL_TYPE.
 * @param  s   The string that contents will be assigned to the contents field of the created CLIPS symbol.
 * @param  env A pointer to a previously created environment. If NULL the default environment is used.
 * @return     A pointer to a CLIPSLexeme type
 */
// clips::core::CLIPSLexeme* createSymbol(const std::string& s, clips::core::Environment *env=NULL);

/**
 * Creates a primitive value with a type field value of STRING_TYPE.
 * @param  s   The string that contents will be assigned to the contents field of the created CLIPS string.
 * @param  env A pointer to a previously created environment. If NULL the default environment is used.
 * @return     A pointer to a CLIPSLexeme type
 */
// clips::core::CLIPSLexeme* createString(const std::string& s, clips::core::Environment *env=NULL);

/**
 * Creates a primitive value with a type field value of INSTANCE_TYPE.
 * @param  s   The string that will be assigned to the contents field of the created CLIPS instance.
 * @param  env A pointer to a previously created environment. If NULL the default environment is used.
 * @return     A pointer to a CLIPSLexeme type
 */
// clips::core::CLIPSLexeme* createInstanceName(const std::string& s, clips::core::Environment *env=NULL);

/**
 * Creates a primitive value with a type field value of SYMBOL_TYPE.
 * @param  b   When true a CLIPSLexeme with the symbol TRUE is created, otherwise a CLIPSLexeme with the symbol FALSE is created.
 * @param  env A pointer to a previously created environment. If NULL the default environment is used.
 * @return     A pointer to a CLIPSLexeme type
 */
// clips::core::CLIPSLexeme* createBoolean(bool b, clips::core::Environment *env=NULL);

/**
 * Creates a primitive value with a type field value of INTEGER_TYPE.
 * @param  i   The C int64_t that will be assigned to the contents field of the created CLIPS float.
 * @param  env A pointer to a previously created environment. If NULL the default environment is used.
 * @return     A pointer to a CLIPSInteger type
 */
// clips::core::CLIPSInteger* CreateInteger(int64_t i, clips::core::Environment *env=NULL);


/**
 * Creates a primitive value with a type field value of FLOAT_TYPE.
 * @param  d   The C double that will be assigned to the contents field of the created CLIPS float.
 * @param  env A pointer to a previously created environment. If NULL the default environment is used.
 * @return     A pointer to a CLIPSFloat type
 */
// clips::core::CLIPSFloat* CreateFloat(double d, clips::core::Environment *env=NULL);

/**
 * Returns a bitwise AND operation over the values of two Type flags
 */
inline constexpr Type operator&(Type x, Type y) {
	return static_cast<Type>(static_cast<uint16_t>(x) & static_cast<uint16_t>(y));
}

/**
 * Returns a bitwise OR operation over the values of two Type flags
 */
inline constexpr Type operator|(Type x, Type y) {
	return static_cast<Type>(static_cast<uint16_t>(x) | static_cast<uint16_t>(y));
}

/**
 * Returns a bitwise eXclussive-OR operation over the values of two Type flags
 */
inline constexpr Type operator^(Type x, Type y) {
	return static_cast<Type>(static_cast<uint16_t>(x) ^ static_cast<uint16_t>(y));
}

}}

#endif // __CLIPSWRAPPERUDF_H__