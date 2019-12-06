OUTPUTDIR := bin/

#-std=c++14
CFLAGS := -std=c++11 -fvisibility=hidden -lpthread

ifeq (,$(CONFIGURATION))
	CONFIGURATION := sync
endif

ifeq (debug,$(CONFIGURATION))
CFLAGS += -g
else
CFLAGS += -O3
endif

SOURCES := src/*.cpp
HEADERS := src/*.h

TARGETBIN := bp-$(CONFIGURATION)

CXX = mpic++

.SUFFIXES:
.PHONY: all clean

all: $(TARGETBIN)

$(TARGETBIN): $(SOURCES) $(HEADERS)
	$(CXX) -o $@ $(CFLAGS) $(SOURCES)

clean:
	rm -rf ./bp-$(CONFIGURATION)

FILES = src/*.cpp \
		src/*.h

handin.tar: $(FILES)
	tar cvf handin.tar $(FILES)
