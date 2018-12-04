

# QVMI

For hardware designers, it is sometimes useful to inspect the power, area and performance of isolated modules. 
However, isolating modules for testing purposes is often tedious and repetitive. 

A typical way to test an individual modules power, area and performance by wrapping the modules with registers feeding into the modules and the module outputs feeding out into output registers.

An example is scripted below:
```python


	''' Inputs of Wrapper'''
	####[31:0]#####[2:0]######
	#     ||         ||      #
	#     ---------------    #
	#     [  registers  ]    #
	#     ---------------    #
	#     ||         ||      #
	#    #||#########||##    #
	#    #              #    #
	#    #   MyModule   #    #
	#    #              #    #
	#    #######||#######    #
	#           ||           #
	#     ---------------    #
	#     [  registers  ]    #
	#     ---------------    #
	#           ||           #
	##########[31:0]##########
	''' Ouputs of Wrapper '''


```

If we have many inputs and outputs for the module we wish to isolate, then this process is long and unproductive. 
It is even more grueling when the modules are outputs from HLS designs, where ports may have arbitrary names, taken from LLVM-level naming, and/or many port declarations.

Hence, I created a Quick Verilog Module Isolator (QVMI) for this purpose. (How else can I get my thesis done on time ;) )

## Compiling

Literally, just head to the top level directory of this repository and type:

```bash
make;
```

Should be good to go! If there is an issue, just issue a bug~ (I will fix it... when I have free time!)

## Usage

To use QVMI just issue the following command:

```
./qvmi /path/to/a/verilogfile.v theModuleToIsolate
```

This will:

1.  Locate `verilogfile.v`, interpret all modules (as long as the syntax is good)
2.  From all modules in `verilogfile.v`, QVMI attempts to fetch `theModuleToIsolate`
3.  If Step 2 was successful, an wrapper is generated and the `theModuleToIsolate` is isolated and saved in the same directory as `verilogfile.v` and is saved as `theModuleToIsolate_dut.v`
4.  Else, you'll just error out (...)

## Features

* Compiles All Modules
* Should be Compliant with Verilog 2001 Syntax
* Fast & Efficient

## Known Issues

* This currently has the mechanics for `inout` ports, but I don't regularly see this in design... so I didn't include it.
* The module to be tested is saved in the same directory as the original verilog file (useful I think...)
* No unit tests have been made (I'll be adding support for this soon)


## Author

Nicholas V. Giamblanco, M.A.Sc @ University of Toronto
