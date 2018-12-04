#ifndef __MODULE_H__
#define __MODULE_H__

#include "port.h"

#include <string>
#include <vector>
#include <algorithm>

class Module {
	private:
		std::string name;
		std::vector<Port * > ports;
		std::vector<Port * > input_ports;
		std::vector<Port * > output_ports;
		std::vector<Port * > inout_ports;

	public:
		Module(std::string name, std::vector<Port *> ports);
		std::string getName();
		std::vector<Port *> getPorts();
		std::vector<Port *> getInputPorts();
		std::vector<Port *> getOutputPorts();
		std::vector<Port *> getInoutPorts();
};

#endif