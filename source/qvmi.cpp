#include "qvmi.h"
#include "parser.h"


std::string usage_string 	= " Usage "; 
std::string usage_operands 	= " [file] [module]\n";
enum STATUS{SUCCESS, FAIL, ERROR};

QVMI::QVMI() {

}



int main(int argc, char * argv[]) {
	std::string filename;
	std::string module;

	for(int i = 1; i < argc; ++i) {
		std::string arg = argv[i];		
		if(arg == "-h" || arg == "-help" || arg == "--h" || arg == "--help") {
			std::cout << usage_string+argv[0]+usage_operands;
			exit(SUCCESS);
		}
	}

	if(argc < 3 || argc > 3) {
		std::cout << usage_string+argv[0]+usage_operands;
		exit(FAIL);
	}



	filename 	= argv[1];
	module 		= argv[2];

	Parser * P = new Parser();

	if(P->parse(filename)) {
		// Generate verilog DUT for Module.

	} else {
		std::cout << "File does not exist. Exiting.\n";
		return ERROR;
	}

	return SUCCESS;
}