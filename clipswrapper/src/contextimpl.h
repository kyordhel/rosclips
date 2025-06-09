/* ** *****************************************************************
* clipswrapper/src/contextimpl.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file clipswrapper/src/context.h
 * A wrapper class to pass a context to user defined functions
 */
#ifndef __CLIPS_UDF_CONTEXTIMPL_H__
#define __CLIPS_UDF_CONTEXTIMPL_H__
#pragma once

#include <functional>
#include "udf/retval.h"
#include "udf/context.h"

extern "C"{
	#include "clips/clips.h"
	// #include "clips/extnfunc.h"
}

namespace clips{ namespace udf{

class ContextImpl : public clips::udf::Context{
public:
	ContextImpl(UDFContext* cctx);
	ContextImpl(const std::function<void(Context&, RetVal&)>& udf, void* udfc);
	~ContextImpl();

public:
	void*       getContext() const;
	UDFContext* getClpCtx() const;
	std::function<void(Context&, RetVal&)> getFunction() const;

public:
	operator UDFContext*() const;

private:
	void*       uctx;
	UDFContext* cctx;
	std::function<void(Context&, RetVal&)> udf;
};

}}

#endif // __CLIPS_UDF_CONTEXTIMPL_H__