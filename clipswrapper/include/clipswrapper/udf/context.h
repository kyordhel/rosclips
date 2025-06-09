/* ** *****************************************************************
* clipswrapper/udf/context.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file clipswrapper/udf/context.h
 * A wrapper class to pass a context to user defined functions
 */
#ifndef __CLIPS_UDF_CONTEXT_H__
#define __CLIPS_UDF_CONTEXT_H__
#pragma once

namespace clips{ namespace udf{

class Context{
public:
	virtual void* getContext() const = 0;
};

}}

#endif // __CLIPS_UDF_CONTEXT_H__
