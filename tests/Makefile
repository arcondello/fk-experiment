ROOT := ../
CATCH2_SRC := $(ROOT)/Catch2/single_include/catch2/
EIGEN_SRC := $(ROOT)/eigen/

tests:
	g++ -std=c++11 -Wall test_chain.cpp -o test_chain -I $(CATCH2_SRC) -I $(EIGEN_SRC)
	./test_chain

catch2:
	git submodule init
	git submodule update
