# Motion capture viewer Makefile 
# Jernej Barbic, Yili Zhao, USC

include Makefile.FLTK

FLTK_PATH=../fltk-1.3.8
PLAYER_OBJECT_FILES = displaySkeleton.o interface.o motion.o posture.o skeleton.o transform.o vector.o mocapPlayer.o ppm.o pic.o performanceCounter.o
INTERPOLATE_OBJECT_FILES = motion.o posture.o skeleton.o transform.o vector.o interpolator.o quaternion.o interpolate.o
COMPILER = g++
COMPILEMODE= -O2
COMPILERFLAGS = $(COMPILEMODE) -I$(FLTK_PATH) $(CXXFLAGS)
LINKERFLAGS = $(COMPILEMODE) $(LINKFLTK_ALL)
CXXFLAGS += -std=c++11

all: mocapPlayer interpolate

mocapPlayer: $(PLAYER_OBJECT_FILES)
	$(COMPILER) $^ $(LINKERFLAGS) -o $@

interpolate: $(INTERPOLATE_OBJECT_FILES)
	$(COMPILER) $^ $(LINKERFLAGS) -o $@

debug_interpolate: CXXFLAGS += -g -Wall -DDEBUG
debug_interpolate: $(INTERPOLATE_OBJECT_FILES)
	$(COMPILER) $^ $(LINKERFLAGS) -g -Wall -DDEBUG -o $@


%.o: %.cpp 
	$(COMPILER) -c $(COMPILERFLAGS) $^

clean:
	-rm -rf core *.o *~ "#"*"#" test

debug_clean: clean
	-rm -rf debug_*
