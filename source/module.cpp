#include "module.h"

Module::Module(std::string name, std::vector<Port *> ports) {
	this->name 		= name;
	this->ports 	= ports;
	for(int i = 0; i < ports.size(); ++i) {
		if(ports[i]->getTypeId() == Port::IN) {
			this->input_ports.push_back(ports[i]);
		}
		if(ports[i]->getTypeId() == Port::OUT) {
			this->output_ports.push_back(ports[i]);
		}
		if(ports[i]->getTypeId() == Port::INOUT) {
			this->inout_ports.push_back(ports[i]);
		}				
	}
}

std::string Module::getName() {
	return this->name;
}

std::vector<Port *> Module::getPorts() {
	return this->ports;
}

std::vector<Port *> Module::getInputPorts() {
	return this->input_ports;
}

std::vector<Port *> Module::getOutputPorts() {
	return this->output_ports;
}

std::vector<Port *> Module::getInoutPorts() {
	return this->inout_ports;
}