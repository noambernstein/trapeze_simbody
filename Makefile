.cpp.o:
	g++ -O3 -std=c++11 -stdlib=libc++ -c -I${HOME}/simbody/include/simbody $< -o $@

trapeze: trapeze.o
	g++ -O3 -std=c++11 -stdlib=libc++ trapeze.o -o trapeze -L${HOME}/simbody/lib -l SimTKsimbody -lSimTKmath -lSimTKcommon -lm

simplest_pendulum: simplest_pendulum.o
	g++ -O3 -std=c++11 -stdlib=libc++ simplest_pendulum.o -o simplest_pendulum -L${HOME}/simbody/lib -l SimTKsimbody -lSimTKmath -lSimTKcommon -lm
