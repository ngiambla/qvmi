#include "parser.h"


Parser::Parser() {
	this->filename = "";
	this->modules_inputs.clear();
	this->vfile.reserve(100*sizeof(std::string));
}


Module * Parser::getModule(std::string mod_name) {
	Module * M = modules_inputs[mod_name];
	assert(M);
	return M;
}

std::string Parser::extractModNameFromDef(std::string moddef) {
	std::string brac_delim = "(";
	std::string mod__delim = "module";

	size_t pos = 0;	
	std::string token;
	std::string modname;

	while ((pos = moddef.find(brac_delim)) != std::string::npos) {
	    token = moddef.substr(0, pos);
	    break;
	}

	if( (pos = token.find(mod__delim)) !=std::string::npos) {
		modname = token.erase(0, pos+mod__delim.length());
		std::string::iterator end_pos = std::remove(modname.begin(), modname.end(), ' ');
		modname.erase(end_pos, modname.end());		
		end_pos = std::remove(modname.begin(), modname.end(), '\n');
		modname.erase(end_pos, modname.end());			
		return modname;
	}

	return "";
}

std::vector<std::string> Parser::extractPortsFromDef(std::string moddef) {
	std::string brac_s_delim = "(";
	std::string brac_e_delim = ")";
	std::string list_a_delim = ",";

	std::vector<std::string> port_names;

	size_t list_begin = 0, list_end =0, list_idx =0;	
	std::string token;
	std::string modname;
	std::string port_name;

	std::string::iterator end_pos = std::remove(moddef.begin(), moddef.end(), '\t');
	moddef.erase(end_pos, moddef.end());		
	end_pos = std::remove(moddef.begin(), moddef.end(), '\n');
	moddef.erase(end_pos, moddef.end());

	if( (list_begin = moddef.find(brac_s_delim)) != std::string::npos ) {
		list_begin += brac_s_delim.length();		
	    token = moddef.substr(list_begin, moddef.length());
	}

	if( (list_end = token.find(brac_e_delim)) != std::string::npos) {
	    token = token.substr(0, list_end);
	}

	if(list_begin == std::string::npos && list_end == std::string::npos) {
		return port_names;
	}

	while((list_idx = token.find(list_a_delim)) != std::string::npos) {
		port_name = token.substr(0, list_idx);
		port_names.push_back(port_name);
		token.erase(0, list_idx+list_a_delim.length());
	}
	port_names.push_back(token);

	return port_names;
	
}


std::string Parser::getPortWidth(std::string port_type, std::string port_name) {
	size_t pos = 0, l1=0, l2=0;

	std::string sbdl = "[";
	std::string sbdr = "]";	
	std::string token;

	if ( (l1 = port_name.find(sbdl)) !=std::string::npos ) {
		if( (l2 = port_name.find(sbdr)) !=std::string::npos ) {
			return port_name.substr(l1, l2);
		} else {
			return "[0:0]";
		}
	} else {
		return "[0:0]";
	}
}

std::vector<Port *> Parser::generatePortsFromDeclaration(std::vector<std::string> port_names) {
	std::vector<std::string> port_types = {"input", "output", "inout"};
	std::vector<Port *> ports;
	std::string port_name;
	size_t list_idx = 0;
	int current_port = 0;

	// while(current_port < port_names.size()) {
	// 	port_name = port_names[current_port];
	// 	for(auto port_type : port_types) {
	// 		if( (list_idx = port_name.find(port_type)) != std::string::npos) {
	// 				std::string width = getPortWidth(port_type, port_name);
	// 				//std::cout << "Width is "<< width << "\n";
	// 		}
	// 	port_name = port_names[current_port];			
	// 		while((port_name.find(port_type)) != std::string::npos && current_port < port_names.size())

	// 	}
	// 	break;

	// }
	return ports;

}

Port * Parser::generatePortFromModule(std::string port_name, std::string mod) {

	std::smatch portmatch;

	std::regex regx____in("input([^;\\)]*)"  +port_name+";");
	std::regex regx___out("output([^;\\)]*)"  +port_name+";");
	std::regex regx_inout("inout([^;\\)]*)"  +port_name+";");
	
	std::string port_found="";
	
	while (std::regex_search (mod, portmatch, regx____in)) {
		for(auto x : portmatch)
			std::cout << x << "\n";
		break;
		//mod = portmatch.suffix().str();		
	}	
	return NULL;
}

bool Parser::parse(std::string filename) {

	bool is_recording 		= false;
	std::string moduledef 	= "";

	std::ifstream file(filename);
	if (file.is_open()) {
	    std::string line;
	    while (getline(file, line)) {
	    	if(line.length() > 0) {
	    		this->vfile.push_back(line);
	    	}
	    }
	    file.close();
	} else {
		return false;
	}

	// Parse Syntax;
	for(int i = 0; i < vfile.size(); ++i) {
		if(vfile[i].find("module") !=std::string::npos) {
			is_recording = !is_recording;
			if(is_recording) {
			} else {
				moduledef += "endmodule";
				std::cout << "  [PARSER] - Module Found.\n";
				std::smatch modulematch;
				std::regex regx("module([^)]*)\\);");
				std::string module_found="";
				
				while (std::regex_search (moduledef,modulematch,regx)) {
					module_found = modulematch.str(0) + "\n";
					break;
				}
				
				if(module_found != "") {
					std::string modulename = extractModNameFromDef(module_found);
					std::cout << "Module: " << modulename << "\n";
					std::vector<std::string> port_names = extractPortsFromDef(module_found);
					std::vector<Port *> ports = generatePortsFromDeclaration(port_names);

					for(int j = 0; j < port_names.size(); ++j) {
						ports.push_back(generatePortFromModule(port_names[j], moduledef));
					}

				}

				moduledef="";
			}
		}
		if(is_recording) {
			//::cout << vfile[i] << "\n";
			moduledef += vfile[i] + "\n";
		}
	}
	return true;
}

