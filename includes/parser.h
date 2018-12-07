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
#include <fstream>
#include <assert.h>     /* assert */

class Parser {
	private:
		std::string filename;
		std::unordered_map<std::string, Module *> modules;
		std::vector<std::string> vfile;

		// Utility Methods
		std::string isolatePortNameFromVerilog2001Declaration(std::string port_name);
		std::string getPortWidth(std::string port_name);
		std::string ReplaceAll(std::string &str, const std::string &from, const std::string &to);
		std::string extractModNameFromDef(std::string moddef);
		std::vector<Parameter *> extractParametersFromDef(std::string moddef);
		std::vector<Parameter *> extractParametersFromDefInternal(std::string moddef);
		std::vector<std::string> extractPortsFromDef(std::string moddef);
		std::vector<Port *> generatePortsFromDeclaration(std::vector<std::string> port_names);
		Port * generatePortFromModule(std::string port_name, std::string mod);

	public:
		Parser();
		bool parse(std::string filename);
		Module * getModule(std::string mod_name);

};

#endif