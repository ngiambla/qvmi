#include "qvmi.h"


std::string usage_string 	= 	" Usage "; 
std::string usage_operands 	= 	" [file] [module1,module2,...]\n";
std::string QVMI_MESSAGE 	= 	"//=-------------------------------------------------------------------\n"\
								"//   ______    __   __   ___ __ __    ________     \n"\
								"//  /_____/\\  /_/\\ /_/\\ /__//_//_/\\  /_______/\\    \n"\
								"//  \\:::_ \\ \\ \\:\\ \\\\ \\ \\\\::\\| \\| \\ \\ \\__.::._\\/    \n"\
								"//   \\:\\ \\ \\ \\_\\:\\ \\\\ \\ \\\\:.      \\ \\   \\::\\ \\     \n"\
								"//    \\:\\ \\ /_ \\\\:\\_/.:\\ \\\\:.\\-/\\  \\ \\  _\\::\\ \\__  \n"\
								"//     \\:\\_-  \\ \\\\ ..::/ / \\. \\  \\  \\ \\/__\\::\\__/\\ \n"\
								"//      \\___|\\_\\_/\\___/_(   \\__\\/ \\__\\/\\________\\/ \n"\
								"//=-------------------------------------------------------------------\n"\
								"// Quick Verilog Module Isolator - Nicholas V. Giamblanco, 2018\n"\
								"//=-------------------------------------------------------------------\n\n\n";

enum STATUS{SUCCESS, FAIL, ERROR};

std::string QVMI::generateDUTWrapper(Module * M) {
	
	std::unordered_map<std::string, int> reserved_clk_map = { {"clk", 1}, {"clock",  1} };
	std::unordered_map<std::string, int> reserved_all_map = { {"clk", 1}, {"clk2x", 1}, {"clock",  1}, {"rst", 1} , {"reset", 1} };

	std::string dut_out = ""; 

	std::cout 	<< "Generating Wrapper for:\n"
				<< "  Module: " << M->getName() << "\n";

	dut_out += QVMI_MESSAGE;

	if(M->getParameters().size() > 0)
		dut_out += "//Parameters for Module\n\n";
	for(int i = 0; i < M->getParameters().size(); ++i) {
		dut_out += "`define "+M->getParameters()[i]->getName() + " " + M->getParameters()[i]->getOperand() + "\n";
	}
	if(M->getParameters().size() > 0)
		dut_out += "\n";

	dut_out += "// Wrapper Definition\n";
	dut_out += "module ";

	dut_out += M->getName() + "_ut (\n";
	for(int i = 0; i < M->getPorts().size(); ++i) {
		Port * P = M->getPorts()[i];
		if(i < M->getPorts().size()-1){
			dut_out += "\t"+ P->getName() +",\n";	
		}
		else {
			dut_out += "\t"+ P->getName() + "\n";
		}
	}	

	dut_out += ");\n\n";
	Port * clkPort = NULL;
	
	dut_out += "// Inputs and Registers\n";
	for(int i = 0; i < M->getInputPorts().size(); ++i) {
		Port * P = M->getInputPorts()[i];
		if(reserved_clk_map.find(P->getName()) != reserved_clk_map.end()) {
			clkPort = P;
		}
		if(reserved_all_map.find(P->getName()) == reserved_all_map.end()) {
			dut_out += P->getType() + " " + P->getWidth() + " " + P->getName() + ";\n";
			dut_out += "reg" + P->getWidth() + " " + P->getName() + "_reg;\n";			
		} else {
			dut_out += P->getType() + " " + P->getName() + ";\n";
		}
	}

	dut_out += "\n// Outputs, Registers & Wires\n";
	for(int i = 0; i < M->getOutputPorts().size(); ++i) {
		Port * P = M->getOutputPorts()[i];
		dut_out += P->getType() + " " + P->getWidth() + " " + P->getName() + ";\n";
		dut_out += "reg" + P->getWidth() + " " + P->getName() + "_reg;\n";
		dut_out += "wire" + P->getWidth() + " " + P->getName() + "_wire;\n";		
	}

	dut_out += "\n// Assigning outputs.\n";
	for(int i = 0; i < M->getOutputPorts().size(); ++i) {
		Port * P = M->getOutputPorts()[i];
		dut_out += "assign " + P->getName() + " = " +P->getName() +"_reg;\n";
	}

	if(clkPort){
		dut_out += "\nalways @(posedge "+clkPort->getName() +")\n\tbegin\n";
	} else {
		dut_out += "\nalways @(*)\n\tbegin\n";		
	}
		for(int i = 0; i < M->getInputPorts().size(); ++i) {
			Port * P = M->getInputPorts()[i];
			if(reserved_all_map.find(P->getName()) == reserved_all_map.end()) {
				dut_out += "\t\t"+P->getName() + "_reg <= "+P->getName() +";\n";
			}			
		}
		for(int i = 0; i < M->getOutputPorts().size(); ++i) {
			Port * P = M->getOutputPorts()[i];
			dut_out += "\t\t"+P->getName() + "_reg <= "+P->getName() +"_wire;\n";
		}		
		dut_out += "\tend\n\n";		
	

	int sigcount = 0;
	dut_out += "// Module Instantiation\n";
	dut_out += M->getName() + " inst1(\n";
	for(int i = 0; i < M->getInputPorts().size(); ++i) {
		Port * P = M->getInputPorts()[i];
		if(reserved_all_map.find(P->getName()) == reserved_all_map.end()) {
			dut_out += "\t ." + P->getName()+ "("+P->getName()+"_reg)";
		}
		else {
			dut_out += "\t ." + P->getName()+ "("+P->getName()+")";
		}

		if(sigcount < M->getPorts().size()-1) {
			dut_out += ",\n";
		} else {
			dut_out += "\n";
		}
		++sigcount;
	}	

	for(int i = 0; i < M->getOutputPorts().size(); ++i) {
		Port * P = M->getOutputPorts()[i];
		dut_out += "\t ." + P->getName()+ "("+P->getName()+"_wire)";
		if(sigcount < M->getPorts().size()-1) {
			dut_out += ",\n";
		} else {
			dut_out += "\n";
		}
		++sigcount;
	}
	dut_out += ");\n\n";

	dut_out += "endmodule\n";
	
	return dut_out;
}


int main(int argc, char * argv[]) {
	size_t list_idx = 0, pos = 0;
	std::string filename;
	std::string dir 			= "";
	std::string module 			= "";
	std::string modules_list 	= "";
	std::string list_a_delim 	= ",";

	std::vector<std::string> modules;

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

	std::cout << QVMI_MESSAGE;

	filename 		= argv[1];
	modules_list 	= argv[2];

	while((list_idx = modules_list.find(list_a_delim)) != std::string::npos) {
		module = modules_list.substr(0, list_idx);
		modules.push_back(module);
		modules_list.erase(0, list_idx+list_a_delim.length());
	}
	modules.push_back(modules_list);	

	Parser * P = new Parser();
	QVMI * qvmi = new QVMI();

	if(P->parse(filename)) {
		// Generate verilog DUT for Module List.
		for(std::string module : modules) {
			Module * M = P->getModule(module);
			if(M == NULL) {
				std::cout << "Module does not exist within " << filename << ". Exiting.\n";
				return ERROR;
			}
			std::string dut_file_cont = qvmi->generateDUTWrapper(M);
			std::ofstream dut_file;
			if((pos =filename.rfind("/")) != std::string::npos) {
				dir = filename.substr(0, pos);
			}
			dut_file.open (dir+"/"+M->getName()+"_dut.v");
			dut_file << dut_file_cont;
			dut_file.close();
		}		
		
	} else {
		std::cout << "File does not exist. Exiting.\n";
		return ERROR;
	}

	return SUCCESS;
}