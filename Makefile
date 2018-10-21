CPPFLAGS = -O3 -g

.cpp.o:
	g++ $(CPPFLAGS) -std=c++11 -stdlib=libc++ -c -I${HOME}/simbody/include/simbody $< -o $@

default: trapeze

trapeze: trapeze.o
	g++ $(CPPFLAGS) -std=c++11 -stdlib=libc++ trapeze.o -o trapeze -L${HOME}/simbody/lib -l SimTKsimbody -lSimTKmath -lSimTKcommon -lm

simplest_pendulum: simplest_pendulum.o
	g++ $(CPPFLAGS) -std=c++11 -stdlib=libc++ simplest_pendulum.o -o simplest_pendulum -L${HOME}/simbody/lib -l SimTKsimbody -lSimTKmath -lSimTKcommon -lm

clean:
	rm -f *.o
