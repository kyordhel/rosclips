#include "contextimpl.h"

using namespace clips::udf;

ContextImpl::ContextImpl(UDFContext* cctx):
	cctx(cctx), uctx(NULL), udf(NULL){
		if(!cctx->context) return;
		ContextImpl* original = (ContextImpl*)(cctx->context);
		udf  = original->getFunction();
		uctx = original->getContext();
}

ContextImpl::ContextImpl(const std::function<void(Context&, RetVal&)>& udf, void* udfc):
	cctx(NULL), uctx(udfc), udf(udf){}

ContextImpl::~ContextImpl(){
}

void* ContextImpl::getContext() const{
	return uctx;
}

UDFContext* ContextImpl::getClpCtx() const{
	return cctx;
}

std::function<void(Context&, RetVal&)> ContextImpl::getFunction() const{
	return udf;
}


ContextImpl::operator UDFContext*() const{
	return cctx;
}
