/* ** ***************************************************************
* clipswrapperudf.h
*
* Author: Mauricio Matamoros
*
* Wrapper for CLIPS' 6.4 User Defined Function helpers
*
** ** **************************************************************/

#ifndef __CLIPSWRAPPERUDF_TYPE_H__
#define __CLIPSWRAPPERUDF_TYPE_H__
#pragma once

namespace clips{ namespace udf{

enum class Type : uint16_t{
	Boolean         = 0x0001,  // b
	Double          = 0x0002,  // d
	ExternalAddress = 0x0004,  // e
	FactAddress     = 0x0008,  // f
	InstanceAddress = 0x0010,  // i
	Integer         = 0x0020,  // l
	Multifield      = 0x0040,  // m
	InstanceName    = 0x0080,  // n
	String          = 0x0100,  // s
	Symbol          = 0x0200,  // y
	Void            = 0x0000,  // v
	Any             = 0x03ff   // *
};

}}


#endif // __CLIPSWRAPPERUDF_TYPE_H__