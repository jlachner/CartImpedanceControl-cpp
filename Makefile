# This makefile is to run local .cpp files to test the library
# The local .cpp files are in the main directory.
# Simply run "make" in the terminal


.PHONY: all

all:

	mkdir -p bin 
	mkdir -p build

	g++ -Iinclude -I./Eigen -MMD -MP -Wall -g -O3 -std=c++17 -c ./src/cart_imp_ctrl.cpp -o ./build/cart_imp_ctrl.o
