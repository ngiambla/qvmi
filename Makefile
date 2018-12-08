# Makefile for QVMI;
# =-----------------=

EXE = qvmi
CXX = g++
BIN = bin
SRC = source
INC = -Iincludes
CFLAGS += -std=c++11 -O3

_OBJS = module.o parser.o qvmi.o port.o parameter.o
OBJS = $(patsubst %,$(BIN)/%,$(_OBJS))

$(shell mkdir -p $(BIN))


all: $(EXE)

$(BIN)/%.o: $(SRC)/%.cpp 
	$(CXX) -c $(INC) -o $@ $< $(CFLAGS) 

$(EXE): $(OBJS)
	$(CXX) -o $(EXE) $^ $(INC) $(CFLAGS)



.PHONY: clean

clean:
	rm -f $(EXE)
	rm -rf $(BIN)

cleanut:
	rm -f unittests/*_dut.v