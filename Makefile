CC_FILES  := $(wildcard src/*.cc)
EXE_FILES := $(patsubst src/%.cc,bin/%,$(CC_FILES))

CXX = g++
IFLAGS = -Iinc


all: $(EXE_FILES)

bin/%: LFLAGS = `grep -m1 "^// GCC-FLAGS:" src/$*.cc | cut -c 14-`
bin/%:  FLAGS = $(shell grep -m1 "^// GCC-FLAGS:" src/$*.cc | cut -c 14-)

bin/%: src/%.cc
	@echo "[$*] FLAGS: $(LFLAGS)"
	@$(CXX) -o $@ $< -Wall -Wextra -pedantic $(FLAGS) $(IFLAGS)

clean:
	rm -f $(EXE_FILES)
