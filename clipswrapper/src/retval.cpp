#include <stdexcept>
#include "udf/retval.h"

using namespace clips::udf;

RetVal::_RetValValue::_RetValValue(): l(0) {}

RetVal::RetVal(): deleteOnDestruct(false), type(Type::Void){}


RetVal::~RetVal(){
	if(deleteOnDestruct)
		delete value.s;
}


void RetVal::setValue(bool v){
	type = Type::Boolean;
	value.b = v;
}


void RetVal::setValue(int64_t v){
	type = Type::Integer;
	value.l = v;
}


void RetVal::setValue(double v){
	type = Type::Double;
	value.d = v;
}


void RetVal::setValue(const std::string& v, Type t){
	if((t != Type::String) && (t != Type::Symbol) && (t != Type::InstanceName))
		throw std::invalid_argument("t must be InstanceName, String, or Symbol.");
	type = t;
	value.s = new char[v.length()+1];
	v.copy(value.s, v.length());
	value.s[v.length()] = 0;
	deleteOnDestruct = true;
}

Type RetVal::getType() const{
	return type;
}

RetVal::_RetValValue RetVal::getValue() const{
	return value;
}
