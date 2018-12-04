#include "port.h"

Port::Port(std::string port_name, std::string width, int typeId) {

	this->port_name 	= port_name;
	this->width 		= width;
	this->typeId 		= typeId;

	switch(typeId) {
		case Port::IN:
			this->type 		= "input";
			break;
		case Port::OUT:
			this->type 		= "output";
			break;
		case Port::INOUT:
			this->type 		= "inout";
			break;
		default:
			exit(-1);
	}
}

std::string Port::getName() {
	return this->port_name;
}

std::string Port::getWidth() {
	return this->width;
}

std::string Port::getType() {
	return this->type;
}

int Port::getTypeId() {
	return this->typeId;
}
