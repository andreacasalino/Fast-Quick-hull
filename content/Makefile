all: clean create_bin
	g++ -o ./bin/Sample_01 ./Sample_01/Main_01.cpp -O3 -fopenmp #remove openmp flag to compile QuickHull in the standard non parallel version
	g++ -o ./bin/Sample_02 ./Sample_02/Main_02.cpp -O3 -fopenmp #remove openmp flag to compile QuickHull in the standard non parallel version
	
clean: 
	rm bin -r -rf

create_bin:	
	mkdir bin -p