#include "parser.h"

Parser::Parser() {
	this->filename = "";
	this->modules.clear();
	this->vfile.reserve(100*sizeof(std::string));
}


Module * Parser::getModule(std::string mod_name) {
	Module * M = modules[mod_name];
	if(!M) {
		std::cout << "Module [" << mod_name << "] was not found.\n";
		assert(M);
	}
	return M;
}

std::string Parser::extractModNameFromDef(std::string moddef) {
	std::string brac__delim = "(";
	std::string mod___delim = "module";
	std::string pound_delim = "#";
	size_t pos = 0;	
	std::string token = "";
	std::string modname;

	if ((pos = moddef.find(pound_delim)) != std::string::npos) {
	    token = moddef.substr(0, pos);
	    goto FETCH_MODNAME;
	}

	if ((pos = moddef.find(brac__delim)) != std::string::npos) {
	    token = moddef.substr(0, pos);
	    goto FETCH_MODNAME;
	}

	FETCH_MODNAME:
		if( (pos = token.find(mod___delim)) !=std::string::npos) {
			modname = token.erase(0, pos+mod___delim.length());
			std::string::iterator end_pos = std::remove(modname.begin(), modname.end(), ' ');
			modname.erase(end_pos, modname.end());		
			end_pos = std::remove(modname.begin(), modname.end(), '\n');
			modname.erase(end_pos, modname.end());			
			return modname;
		}

	return "";
}

std::string Parser::ReplaceAll(std::string &str, const std::string &from, const std::string &to) {
    size_t start_pos = 0;
    
    if(str == "") {
    	return str;
    }

    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
    }
    return str;
}

std::vector<Parameter *> Parser::extractParametersFromDefInternal(std::string moddef) {
	std::smatch parametermatch;
	std::regex regx_param("parameter(\\s+)([^\\(;]*)(\\s*);");	
	size_t list_idx = 0, param_idx = 0;

	std::string parameters_found;
	std::string list_a_delim = ",";	
	std::string equl_a_delim = "=";
	std::string param = "";
	std::string moddef_r=moddef;

	std::vector<Parameter *> parameters;


	while (std::regex_search (moddef_r, parametermatch, regx_param)) {
		std::vector<std::string> params_unformed;	
		for(auto x : parametermatch) {
			std::string pline = x.str();
			ReplaceAll(pline, "parameter", "");
			ReplaceAll(pline, ";", "");

			while((list_idx = pline.find(list_a_delim)) != std::string::npos) {
				param = pline.substr(0, list_idx);
				params_unformed.push_back(param);
				pline.erase(0, list_idx+list_a_delim.length());
			}
			params_unformed.push_back(pline);

			for(int i = 0; i < params_unformed.size(); ++i) {
				if( (param_idx = params_unformed[i].find(equl_a_delim)) != std::string::npos ) {
					std::string pname = params_unformed[i].substr(0, param_idx);
					params_unformed[i].erase(0, param_idx+equl_a_delim.length());
					parameters.push_back(new Parameter(pname, params_unformed[i]));
				}
			}
			break;
		}
    	moddef_r = parametermatch.suffix().str();

		//break;
	}
	return parameters;	
}


std::vector<Parameter *> Parser::extractParametersFromDef(std::string moddef) {
	std::string brac_s_delim = "#(";
	std::string brac_e_delim = ")";
	std::string list_a_delim = ",";
	std::string equl_a_delim = "=";

	std::string moddef_real = moddef;

	size_t list_begin = 0, list_end =0, list_idx =0, param_idx = 0;	

	std::smatch parametermatch;

	std::regex regx("#(\\s*)(\\([^\\)]*\\))");
	std::string parameters_found="", token="", param = "";

	std::vector<std::string> params_unformed;
	std::vector<Parameter *> parameters;

	
	while (std::regex_search (moddef,parametermatch,regx)) {
		parameters_found = parametermatch.str(0) + "\n";
		goto EXTRACT;
	}

	return parameters;

	EXTRACT:
	std::string::iterator end_pos = std::remove(parameters_found.begin(), parameters_found.end(), '\t');
	parameters_found.erase(end_pos, parameters_found.end());		
	end_pos = std::remove(parameters_found.begin(), parameters_found.end(), '\n');
	parameters_found.erase(end_pos, parameters_found.end());	

	ReplaceAll(parameters_found, "parameter", "");


	if( (list_begin = parameters_found.find(brac_s_delim)) != std::string::npos ) {
		list_begin += brac_s_delim.length();		
	    token = parameters_found.substr(list_begin, parameters_found.length());
	}

	if( (list_end = token.find(brac_e_delim)) != std::string::npos) {
	    token = token.substr(0, list_end);
	}

	while((list_idx = token.find(list_a_delim)) != std::string::npos) {
		param = token.substr(0, list_idx);
		params_unformed.push_back(param);
		token.erase(0, list_idx+list_a_delim.length());
	}
	params_unformed.push_back(token);

	for(int i = 0; i < params_unformed.size(); ++i) {
		if( (param_idx = params_unformed[i].find(equl_a_delim)) != std::string::npos ) {
			std::string pname = params_unformed[i].substr(0, param_idx);
			params_unformed[i].erase(0, param_idx+equl_a_delim.length());
			parameters.push_back(new Parameter(pname, params_unformed[i]));
		}
	}

	return parameters;
}

std::vector<std::string > Parser::extractPortsFromDef(std::string moddef) {
	std::string brac_s_delim = "(";
	std::string brac_e_delim = ")";
	std::string list_a_delim = ",";

	std::vector<std::string> port_names;

	size_t list_begin = 0, list_end =0, list_idx =0;	
	std::string token;
	std::string modname;
	std::string port_name;
	std::string ports = "";
	std::smatch modmatch, portmatch;
	std::regex regx("module([^\\)#;]+)(#\\s*\\([^;]*\\))?\\)?;");
	std::regex regx_moddef_ports("[^#](\\([^\\)]*\\));");

	if(std::regex_search(moddef, modmatch, regx)) {
		std::string mod_def_match = modmatch.str(0) + "\n";
		if(std::regex_search(mod_def_match, portmatch, regx_moddef_ports)) {
			port_name = portmatch.str(0) + "\n";
		}
	}


	std::string::iterator end_pos = std::remove(port_name.begin(), port_name.end(), '\t');
	port_name.erase(end_pos, port_name.end());		
	end_pos = std::remove(port_name.begin(), port_name.end(), '\n');
	port_name.erase(end_pos, port_name.end());

	if( (list_begin = port_name.find(brac_s_delim)) != std::string::npos ) {
		list_begin += brac_s_delim.length();		
	    token = port_name.substr(list_begin, moddef.length());
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


std::string Parser::getPortWidth(std::string port_name) {
	size_t l1=0, l2=0;

	std::string sbdl = "[";
	std::string sbdr = "]";	
	std::string token;

	if ( (l1 = port_name.find(sbdl)) !=std::string::npos ) {
		token = port_name.substr(l1, std::string::npos);
	} else {
		return "";
	} 
	if( (l2 = token.find(sbdr)) !=std::string::npos ) {
		return token.substr(0, l2+1);
	} else {
		return "";
	}
}

std::string Parser::isolatePortNameFromVerilog2001Declaration(std::string port_name) {


	size_t l2=0;

	if( (l2 = port_name.find("]")) !=std::string::npos ) {
		return port_name.substr(l2+1, std::string::npos);
	}
	if( (l2 = port_name.find("reg")) !=std::string::npos ) {
		return port_name.substr(l2+3, std::string::npos);
	}
	if( (l2 = port_name.find("input")) !=std::string::npos ) {
		return port_name.substr(l2+5, std::string::npos);
	}
	if( (l2 = port_name.find("output")) !=std::string::npos ) {
		return port_name.substr(l2+6, std::string::npos);
	}		
	if( (l2 = port_name.find("inout")) !=std::string::npos ) {
		return port_name.substr(l2+5, std::string::npos);
	}	

	std::string::iterator end_pos = std::remove(port_name.begin(), port_name.end(), '\t');
	port_name.erase(end_pos, port_name.end());		
	end_pos = std::remove(port_name.begin(), port_name.end(), '\n');
	port_name.erase(end_pos, port_name.end());
	end_pos = std::remove(port_name.begin(), port_name.end(), ' ');
	port_name.erase(end_pos, port_name.end());

	return port_name;

}

std::vector<Port *> Parser::generatePortsFromDeclaration(std::vector<std::string> &port_names) {

	std::vector<std::string> port_types = {"input", "output", "inout"};
	std::vector<int> port_type_idx;

	std::unordered_map<std::string, int> port_type_map = { 	{"input", Port::IN},
															{"output", Port::OUT},
															{"inout", Port::INOUT}
														};
	std::vector<Port *> ports;
	std::string port_name;
	size_t list_idx = 0;

	int current_port, inferred_port=0;

	for(int i = 0; i < port_names.size(); ++i) {

		if(port_names[i].find("input") != std::string::npos) {
			port_type_idx.push_back(Port::IN);
		} else if(port_names[i].find("output") != std::string::npos) {
			port_type_idx.push_back(Port::OUT);
		} else if(port_names[i].find("inout") != std::string::npos) {
			port_type_idx.push_back(Port::INOUT);
		} else {
			port_type_idx.push_back(-1);
		}

	}

	int hold = -1;
	std::vector<int> idx_to_erase;

	for(int i = 0; i < port_names.size(); ++i) {
		if(port_type_idx[i] != hold && port_type_idx[i] != -1) {
			hold = port_type_idx[i];
		}		
		if(hold != -1) {
			std::string width = getPortWidth(port_names[i]);
			ports.push_back(new Port(isolatePortNameFromVerilog2001Declaration(port_names[i]), width, hold));
			idx_to_erase.push_back(i);
		}
	}

	for(int i = idx_to_erase.size()-1; i >=0 ; --i) {
		port_names.erase(port_names.begin() + idx_to_erase[i]);
	}


	return ports;

}

Port * Parser::generatePortFromModule(std::string port_name, std::string mod) {

	std::smatch portmatch;


	std::string::iterator end_pos = std::remove(port_name.begin(), port_name.end(), '\t');
	port_name.erase(end_pos, port_name.end());		
	end_pos = std::remove(port_name.begin(), port_name.end(), '\n');
	port_name.erase(end_pos, port_name.end());
	end_pos = std::remove(port_name.begin(), port_name.end(), ' ');
	port_name.erase(end_pos, port_name.end());

	std::regex regx____in("input(\\s+)(reg)?(wire)?(\\s*)(\\[[^;]*\\])?(\\s*)"+port_name+";");
	std::regex regx___out("output(\\s+)(reg)?(wire)?(\\s*)(\\[[^;]*\\])?(\\s*)"+port_name+";");
	std::regex regx_inout("inout(\\s+)(reg)?(wire)?(\\s*)(\\[[^;]*\\])?(\\s*)"+port_name+";");
		
	if(port_name == "") {
		return NULL;
	}

	while (std::regex_search (mod, portmatch, regx____in)) {
		for(auto x : portmatch) {
			return new Port(port_name, getPortWidth(x.str()), Port::IN);
		}
	}
	while (std::regex_search (mod, portmatch, regx___out)) {
		for(auto x : portmatch) {
			return new Port(port_name, getPortWidth(x.str()), Port::OUT);			
		}
	}
	while (std::regex_search (mod, portmatch, regx_inout)) {
		for(auto x : portmatch) {
			return new Port(port_name, getPortWidth(x.str()), Port::INOUT);			
		}	
	}

	std::cout << "  [WARNING] Couldn't find: "<<port_name << "\n";
	std::cout << mod << "\n";

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
				moduledef += "endmodule\n";
				std::smatch modulematch;
				std::regex regx("module([^\\)#;]+)(#\\s*\\([^;]*\\))?\\)?;");
				std::string module_found="";
				
				while (std::regex_search (moduledef,modulematch,regx)) {
					module_found = modulematch.str(0) + "\n";
					break;
				}
				
				if(module_found != "") {
					std::string modulename = extractModNameFromDef(module_found);
					std::cout << "[INFO] Module Found - "<< modulename << "\n";

					std::vector<Parameter *> parameters = extractParametersFromDef(module_found);				
					std::vector<std::string> port_names = extractPortsFromDef(module_found);
					std::vector<Port *> ports = generatePortsFromDeclaration(port_names);

					for(int j = 0; j < port_names.size(); ++j) {
						Port * P = generatePortFromModule(port_names[j], moduledef);
						if(P != NULL) {
							ports.push_back(P);
						} 
					}

					modules[modulename] = new Module(modulename, ports, parameters);

				}

				moduledef="";
			}
		}
		if(is_recording) {
			moduledef += vfile[i] + "\n";
		}
	}
	std::cout << "\n";
	return true;
}

