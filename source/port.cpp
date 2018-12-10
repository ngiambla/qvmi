#include "port.h"

Port::Port(std::string port_name, std::string width, int typeId) {

	this->width 		= width;
	this->typeId 		= typeId;

	std::smatch portmatch;


	std::regex regx_rm_space("([^\\s]+)");

	if(std::regex_search (port_name, portmatch, regx_rm_space)) {
		std::cout << "--> " << portmatch[0].str() << "\n";
		port_name = portmatch[0].str();
	}	

	this->port_name 	= port_name;

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
