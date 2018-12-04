#ifndef __PARSER_H__
#define __PARSER_H__
// QVMI - Defines
#include "module.h"

// C & C++ Stdlib & STL - Defines
#include <fstream>
#include <string>
#include <regex>
#include <algorithm>
#include <unordered_map>
#include <iostream> 
#include <assert.h>     /* assert */

class Parser {
	private:
		std::string filename;
		std::unordered_map<std::string, Module *> modules_inputs;
		std::vector<std::string> vfile;

		std::string extractModNameFromDef(std::string moddef);
		std::vector<std::string> extractPortsFromDef(std::string moddef);
		std::vector<Port *> generatePortsFromDeclaration(std::vector<std::string> port_names);
		std::string getPortWidth(std::string port_type, std::string port_name);
		Port * generatePortFromModule(std::string port_name, std::string mod);

	public:
		Parser();
		bool parse(std::string filename);
		Module * getModule(std::string mod_name);

};

#endif