#include "qvmi.h"


std::string usage_string 	= " Usage "; 
std::string usage_operands 	= " [file] [module]\n";
enum STATUS{SUCCESS, FAIL, ERROR};

std::string QVMI::generateDUTWrapper(Module * M) {
	
	std::string dut_out = "module ";

	std::cout 	<< "Generating Wrapper for:\n"
				<< "  Module: " << M->getName() << "\n";

	dut_out += M->getName() + "_ut (\n";
	for(int i = 0; i < M->getPorts().size(); ++i) {
		Port * P = M->getPorts()[i];
		if(i < M->getPorts().size()-1)
			dut_out += "\t"+ P->getName() +",\n";	
		else 
			dut_out += "\t"+ P->getName() + "\n";
	}	

	dut_out += ");\n\n";
	Port * clkPort = NULL;
	
	dut_out += "// Inputs and Registers\n";
	for(int i = 0; i < M->getInputPorts().size(); ++i) {
		Port * P = M->getInputPorts()[i];
		if(P->getName() != "clk" && P->getName() != "clock") {
			dut_out += P->getType() + " " + P->getWidth() + " " + P->getName() + ";\n";
			dut_out += "reg" + P->getWidth() + " " + P->getName() + "_reg;\n";
		} else {
			clkPort = P;
			dut_out += P->getType() + " " + P->getName() + ";\n";			
		}
	}

	dut_out += "\n// Outputs, Registers & Wires\n";
	for(int i = 0; i < M->getOutputPorts().size(); ++i) {
		Port * P = M->getOutputPorts()[i];
		dut_out += P->getType() + " " + P->getWidth() + " " + P->getName() + ";\n";
		if(P->getName() != "clk" && P->getName() != "clock")
			dut_out += "reg" + P->getWidth() + " " + P->getName() + "_reg;\n";
			dut_out += "wire" + P->getWidth() + " " + P->getName() + "_wire;\n";
	}

	dut_out += "\n// Assigning outputs.\n";
	for(int i = 0; i < M->getOutputPorts().size(); ++i) {
		Port * P = M->getOutputPorts()[i];
		if(P->getName() != "clk" && P->getName() != "clock")
			dut_out += "assign " + P->getName() + " = " +P->getName() +"_reg;\n";
	}

	if(clkPort){
		dut_out += "\nalways @(posedge "+clkPort->getName() +")\n\tbegin\n";
		for(int i = 0; i < M->getInputPorts().size(); ++i) {
			Port * P = M->getInputPorts()[i];
			if(P->getName() != "clk" && P->getName() != "clock")
				dut_out += "\t\t"+P->getName() + "_reg <= "+P->getName() +";\n";
		}
		for(int i = 0; i < M->getOutputPorts().size(); ++i) {
			Port * P = M->getOutputPorts()[i];
			if(P->getName() != "clk" && P->getName() != "clock")
				dut_out += "\t\t"+P->getName() + "_reg <= "+P->getName() +"_wire;\n";
		}		
		dut_out += "\tend\n\n";		
	}

	int sigcount = 0;

	dut_out += M->getName() + " inst1(\n";
	for(int i = 0; i < M->getInputPorts().size(); ++i) {
		Port * P = M->getInputPorts()[i];
		if(P->getName() != "clk" && P->getName() != "clock")
			dut_out += "\t ." + P->getName()+ "("+P->getName()+"_reg)";
		else
			dut_out += "\t ." + P->getName()+ "("+P->getName()+")";
		if(sigcount < M->getPorts().size()-1) {
			dut_out += ",\n";
		} else {
			dut_out += "\n";
		}

		++sigcount;
	}	

	for(int i = 0; i < M->getOutputPorts().size(); ++i) {
		Port * P = M->getOutputPorts()[i];
		if(P->getName() != "clk" && P->getName() != "clock")
			dut_out += "\t ." + P->getName()+ "("+P->getName()+"_wire)";
		else
			dut_out += "\t ." + P->getName()+ "("+P->getName()+")";
		if(sigcount < M->getPorts().size()-1) {
			dut_out += ",\n";
		} else {
			dut_out += "\n";
		}
		++sigcount;
	}
	dut_out += ");\n\n";

	dut_out += "endmodule\n";
	//std::cout << dut_out; 
	
	return dut_out;
}


int main(int argc, char * argv[]) {
	size_t pos = 0;
	std::string filename;
	std::string dir ="";
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
		Module * M = P->getModule(module);
		if(M == NULL) {
			std::cout << "Module does not exist within " << filename << ". Exiting.\n";
			return ERROR;
		}
		QVMI * qvmi = new QVMI();
		std::string dut_file_cont = qvmi->generateDUTWrapper(M);
		std::ofstream dut_file;
		if((pos =filename.rfind("/")) != std::string::npos) {
			dir = filename.substr(0, pos);
		}
		dut_file.open (dir+"/"+M->getName()+"_dut.v");
		dut_file << dut_file_cont;
		dut_file.close();		
		
	} else {
		std::cout << "File does not exist. Exiting.\n";
		return ERROR;
	}

	return SUCCESS;
}