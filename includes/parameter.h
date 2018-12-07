#ifndef __PARAMETER_H__
#define __PARAMETER_H__

// C & C++ Stdlib & STL - Defines
#include <string>
#include <regex>
#include <algorithm>
#include <unordered_map>
#include <iostream> 
#include <assert.h>     /* assert */

class Parameter {
	private:
		std::string parameter____name;
		std::string parameter_operand;

	public:
		Parameter(std::string parameter____name, std::string parameter_operand);
		std::string getName();
		std::string getOperand();

};

#endif