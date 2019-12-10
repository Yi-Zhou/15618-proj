OUTPUTDIR := build

#-std=c++14
CFLAGS := -std=c++11 -fvisibility=hidden -lpthread

ifeq (,$(CONFIGURATION))
	CONFIGURATION := async
endif

ifeq (debug,$(CONFIGURATION))
CFLAGS += -g
else
CFLAGS += -O3
endif

# SOURCES := src/*.cpp
# TARGETBIN := bp-$(CONFIGURATION)

CXX = mpic++

.SUFFIXES:
.PHONY: 

SYNCBIN := $(OUTPUTDIR)/synchronous
DBSBIN := $(OUTPUTDIR)/dpsplash
PARTBIN := $(OUTPUTDIR)/partition

all: $(SYNCBIN) $(DBSBIN)

SOURCES := src/dbsplash.cpp src/fg.cpp src/common.cpp
HEADERS := src/*.h

SYNCSOURCES := src/synchronous.cpp src/fg.cpp src/common.cpp

sync: $(SYNCBIN)
$(SYNCBIN) : $(SYNCSOURCES) $(HEADERS)
	$(CXX) -o $@ $(CFLAGS) $(SYNCSOURCES)


dbs: $(DBSBIN)
$(DBSBIN) : $(SOURCES) $(HEADERS)
	$(CXX) -o $@ $(CFLAGS) $(SOURCES)

PARTSOURCES := src/partition.cpp


part: $(PARTBIN)
$(PARTBIN) : $(PARTSOURCES)
	$(CXX) -o $@ $(CFLAGS) $(PARTSOURCES) -lmetis

$(TARGETBIN): $(SOURCES) $(HEADERS)
	$(CXX) -o $@ $(CFLAGS) $(SOURCES)

clean:
	rm -rf $(OUTPUTDIR)/*

# FILES = src/*.cpp \
# 		src/*.h

# handin.tar: $(FILES)
# 	tar cvf handin.tar $(FILES)
