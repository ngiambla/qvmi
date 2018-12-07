#include "parameter.h"

Parameter::Parameter(std::string parameter____name, std::string parameter_operand) {

	this->parameter____name 	= parameter____name;
	this->parameter_operand 	= parameter_operand;
}

std::string Parameter::getName() {
	return this->parameter____name;
}

std::string Parameter::getOperand() {
	return this->parameter_operand;
}
