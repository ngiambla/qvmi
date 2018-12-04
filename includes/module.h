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
	public:
		Module(std::string name);
		void addPort(Port * port);


};

#endif