#ifndef __PORT_H__
#define __PORT_H__

// C & C++ Stdlib & STL - Defines
#include <string>
#include <regex>
#include <algorithm>
#include <unordered_map>
#include <iostream> 
#include <assert.h>     /* assert */

class Port {
	private:
		std::string in_width;
		std::string out_width;
		std::string inout_width;
		std::string port_name;

	public:
		Port(std::string port_name, std::string in_width, std::string out_width);
		std::string getInWidth();
		std::string getOutWidth();
		std::string getInoutWidth();

};

#endif