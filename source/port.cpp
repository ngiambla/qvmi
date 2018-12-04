#include "port.h"

Port::Port(std::string port_name, std::string in_width="", std::string out_width="", std::string inout_width="") {
	this->port_name 	= port_name;
	this->in_width 		= in_width;
	this->out_width 	= out_width;
	this->inout_width 	= inout_width;
}


Port::getInWidth() {
	return this->in_width;
}

Port::getOutWidth() {
	return this->out_width;
}

Port::getInoutWidth() {
	return this->inout_width;
}