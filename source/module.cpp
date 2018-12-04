#include "module.h"

Module::Module(std::string name) {
	this->name = name;
}


void Module::addPort(Port * port) {
	this->ports.push_back(port);
} 