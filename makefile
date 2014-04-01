.SUFFIXES: .cpp .hpp

# Project Name (generate executable with this name)
EXETAR = cs296_06_exe

# Project Paths
PROJECT_ROOT=./
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/myobjs
BINDIR = $(PROJECT_ROOT)/mybins
LIBDIR = $(PROJECT_ROOT)/mylibs
DOCDIR = $(PROJECT_ROOT)/doc
RM 	= rm
DOXYGEN = doxygen

# Library Paths
BOX2D_ROOT=$(PROJECT_ROOT)/external
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib 

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
NOMAINSRCS := $(SRCS:$(SRCDIR)/main.cpp= )
NOMAINOBJS := $(OBJS:$(OBJDIR)/main.o= )
SHARED_LIB = TRUE
TO_DELETE = cs296_report_06.aux cs296_report_06.bbl cs296_report_06.blg cs296_report_06.dvi cs296_report_06.log cs296_report_06.pdf html

.PHONY: all setup exe staticL dynamicL clean distclean report doc

all:	setup exe exelib
setup:
	@echo "Setting up compilation..."
	@mkdir -p myobjs;
	@mkdir -p mybins;
	@mkdir -p mylibs;
	@if [ ! -d Box2D ]; \
	then cd $(BOX2D_ROOT)/src; tar xzf ./Box2D.tgz;\
	fi;
	@cd $(BOX2D_ROOT)/src/Box2D; ls; mkdir -p build296; \
	cd build296; ls; cmake ../; make; make install;

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@g++ -fPIC $(CPPFLAGS) -c $< -o $@ -MD

-include $(OBJS:.o=.d)

exe: setup $(OBJS)
	@g++ -o $(BINDIR)/$(EXETAR) $(CPPFLAGS) $(LDFLAGS) $(OBJS) $(LIBS)

staticL: $(OBJS)
	@ar rvs ./mylibs/libCS296test.a $(NOMAINOBJS);

dynamicL: $(OBJS)
	@g++ -shared -o ./mylibs/libCS296test.so $(NOMAINOBJS);

exelib: setup
	@if [ $(SHARED_LIB) = TRUE ]; \
	then make dynamicL; \
	g++ -o $(BINDIR)/cs296_06_exelib $(LDFLAGS) -L ./mylibs/ $(CPPFLAGS) $(OBJDIR)/main.o -l:libCS296test.so $(LIBS); \
	else \
	make staticL; \
	g++ -o $(BINDIR)/cs296_06_exelib $(CPPFLAGS) $(LDFLAGS) -L ./mylibs/ $(OBJDIR)/main.o -l:libCS296test.a $(LIBS); \
	fi;

doc:
	@echo -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@echo "Done"

clean:
	@echo "Cleaning code files";
	@rm -r -f ./mylibs ./mybins ./myobjs
	@cd doc;\
	rm -f -r $(TO_DELETE);
	@echo "Done...";
	

distclean: clean
	@echo "Cleaning Box2D files";
	@rm -r -f $(BOX2D_ROOT)/src/Box2D $(BOX2D_ROOT)/include/* $(BOX2D_ROOT)/lib/*; 

report:
	@echo "Genrating Latex..."
	@cd doc;\
	latex cs296_report_06.tex;\
	bibtex cs296_report_06.aux;\
	latex cs296_report_06.tex;\
	latex cs296_report_06.tex;\
	dvipdf cs296_report_06.dvi cs296_report_06.pdf;
	@echo "Done....."
