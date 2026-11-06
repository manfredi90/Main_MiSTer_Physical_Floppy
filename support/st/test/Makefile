ST = ..
CXXFLAGS = -O2 -g -Wall -Wextra -std=gnu++17 -I$(ST)
LIB = $(ST)/st_stx.cpp $(ST)/st_trackbuild.cpp

all: stxtool stxgen

stxtool: stxtool.cpp wdsim.cpp wdsim.h $(LIB) $(ST)/st_stx.h $(ST)/st_trackbuild.h $(ST)/st_mfmt.h
	$(CXX) $(CXXFLAGS) -o $@ stxtool.cpp wdsim.cpp $(LIB)

stxgen: stxgen.cpp $(ST)/st_trackbuild.cpp
	$(CXX) $(CXXFLAGS) -o $@ stxgen.cpp $(ST)/st_trackbuild.cpp

test: all
	./stxgen synth.stx
	./stxtool -v synth.stx

clean:
	rm -f stxtool stxgen synth.stx
