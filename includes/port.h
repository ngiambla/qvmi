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
		std::string port_name;
		std::string width;
		std::string type;
		int typeId;
	public:
		Port(std::string port_name, std::string width, int type);
		std::string getName();
		std::string getWidth();
		std::string getType();
		int getTypeId();
   		enum TYPES { 
						IN,
						OUT,
						INOUT
					};

};

#endif