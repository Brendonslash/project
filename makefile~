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
DEBUGFLAGS := $(CPPFLAGS:-O3=) 

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJR := $(OBJS:$(OBJDIR)/%.o=%.o)
ODR := $(OBJS:$(OBJDIR)/%.o=%.d)
NOMAINSRCS := $(SRCS:$(SRCDIR)/main.cpp= )
NOMAINOBJS := $(OBJS:$(OBJDIR)/main.o= )
SHARED_LIB = TRUE
INSTALL_FOLD = $(PROJECT_ROOT)/install
TO_DELETE = cs296_report_06.aux cs296_report_06.bbl cs296_report_06.blg cs296_report_06.dvi cs296_report_06.log cs296_report_06.pdf html g06_prof_report.dvi g06_prof_report.pdf g06_prof_report.log g06_prof_report.aux

.PHONY: all setup_common setup_release setup_debug setup exe staticL dynamicL clean distclean report doc data plots release_prof debug_prof debug_objects report exe_debug exe_release obj_debug dist install

all:	setup exe exelib doc

setup_common: 
	@echo "Setting up compilation..."
	@mkdir -p myobjs;
	@mkdir -p mybins;
	@mkdir -p mylibs;
	@mkdir -p data
	@if [ ! -d Box2D ]; \
	then cd $(BOX2D_ROOT)/src; tar xzf ./Box2D.tgz;\
	fi;
	@patch -p0 < b2Timer_cpp.patch; patch -p0 < b2Timer_h.patch;
	
setup: setup_common
	@cd $(BOX2D_ROOT)/src/Box2D; ls; mkdir -p build296; \
	cd build296; ls; cmake ../; make; make install;

setup_release: setup_common
	@cd $(BOX2D_ROOT)/src/Box2D; ls; mkdir -p build296; \
	cd build296; ls; cmake -DCMAKE_BUILD_TYPE=Release ../; make; make install;

setup_debug: setup_common
	@cd $(BOX2D_ROOT)/src/Box2D; ls; mkdir -p build296; \
	cd build296; ls; cmake -DCMAKE_BUILD_TYPE=Debug ../; make; make install;



dist:	
	@cd ..; tar cvzf cs296_g06_project.tar.gz cs296_base_code; 

install:
	@mkdir -p $(INSTALL_FOLD);
	@cp -a ./mybins $(INSTALL_FOLD);
	@cp -a ./doc $(INSTALL_FOLD);
	@cp -a ./external $(INSTALL_FOLD);
	@ cd $(INSTALL_FOLD); cd ./external rm -rf src;
 


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

obj_debug:
	@g++ -fPIC $(DEBUGFLAGS) -pg -c $(SRCS) -MD;\
	mv $(OBJR) $(ODR) $(OBJDIR);		

exe_debug: obj_debug
	@g++ -o $(BINDIR)/$(EXETAR) $(DEBUGFLAGS) -pg $(LDFLAGS) $(OBJS) $(LIBS)

exe_release: 
	@g++ -fPIC $(CPPFLAGS) -pg -c $(SRCS) -MD;
	@mv $(OBJR) $(ODR) $(OBJDIR);
	@g++ -o $(BINDIR)/$(EXETAR) $(CPPFLAGS) -pg $(LDFLAGS) $(OBJS) $(LIBS);

doc:
	@echo -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@echo "Done"

clean:
	@echo "Cleaning code files";
	@rm -r -f ./mylibs ./mybins ./myobjs ./plots ./data
	@cd doc;\
	rm -f -r $(TO_DELETE);
	@echo "Done...";
	

distclean: clean
	@echo "Cleaning Box2D files";
	@rm -r -f $(BOX2D_ROOT)/src/Box2D $(BOX2D_ROOT)/include/* $(BOX2D_ROOT)/lib/*; 

report:
	@echo "Genrating Latex..."
	@cd doc;\
	pdflatex cs296_report_06.tex;\
	bibtex cs296_report_06.aux;\
	pdflatex cs296_report_06.tex;\
	pdflatex cs296_report_06.tex;
	@cd scripts;\
	chmod 755 g06_gen_html.py;\
	./g06_gen_html.py;
	@echo "Done....."

data:
	@cd scripts;\
	./g06_gen_data.sh;\
	./g06_gen_csv.sh;\
	./g06_gen_data_csv.sh; ./g06_gen_data_random.sh;

plots:
	@mkdir -p plots;
	@cd scripts;\
	./avg_01.sh;\
	./ebars_01.sh;\
	./ext_04.sh;\
	./line_fit.sh;\
	gnuplot g06_plot01.gpt;\
	gnuplot g06_plot02.gpt;\
	gnuplot g06_plot03.gpt;\
	gnuplot g06_plot04.gpt;\
	gnuplot g06_plot05.gpt;
	@rm -f ./data/temp_*;
	@rm -f ./scripts/*.log;

debug_prof: setup_debug exe_debug
	@cd mybins; ./cs296_06_exe 1000;\
	gprof cs296_06_exe gmon.out > g06_debug_prof.dat;\
	rm gmon.out;\
	mv g06_debug_prof.dat ../data/g06_debug_prof.dat;
	

release_prof: setup_release exe_release
	@cd mybins; ./cs296_06_exe 1000;\
	gprof cs296_06_exe gmon.out > g06_release_prof.dat;\
	rm gmon.out;\
	mv g06_release_prof.dat ../data/g06_release_prof.dat;

	
