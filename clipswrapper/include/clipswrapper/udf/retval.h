#ifndef __CLIPS_UDF_RETVAL_H__
#define __CLIPS_UDF_RETVAL_H__
#pragma once

#include <string>
#include "type.h"

namespace clips{ namespace udf{

class RetVal{
public:
	RetVal();
	~RetVal();

private:
	union _RetValValue{
		_RetValValue();
		bool     b;
		int64_t  l;
		double   d;
		char*    s;
		void*    ptr;
	} value;

public:
	/**
	 * Sets a boolean value as result of the function operation
	 * @param v Boolean value to pass to CLIPS
	 */
	void setValue(bool v);

	/**
	 * Sets an integer value as result of the function operation
	 * @param v Integer value to pass to CLIPS
	 */
	void setValue(int64_t v);

	/**
	 * Sets a double value as result of the function operation
	 * @param v Double value to pass to CLIPS
	 */
	void setValue(double v);

	/**
	 * Sets a string value as result of the function operation
	 * that can represent either an Instance, String, or Symbol
	 * @param v String value to pass to CLIPS
	 * @param t Optional. Specifies how CLIPS should interpret the contents of the string.
	 *          Default is String.
	 */
	void setValue(const std::string& v, Type t=Type::String);

	Type getType() const;
	_RetValValue getValue() const;

private:
	Type type;
	bool deleteOnDestruct;
};

}}

#endif // __CLIPS_UDF_RETVAL_H__