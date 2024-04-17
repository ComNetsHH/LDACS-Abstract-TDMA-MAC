###
# This file is responsible for providing the full toolchain of creating result graphs.
# Each target is responsible for one scenario, and should
# - start the simulation(s)
# - parse the resulting .sca and .vec files
# - and plot the parsed results into .pdfs
# 
# Before each target, some brief comments should describe what the scenario is about.
# Also, variables can be created that specify parameters of the respective target. These can be shared among several targets (where e.g. each target runs some sub-scenario).
# Naming these variables should follow a SCENARIO_VARNAME syntax.
###
build-debug:
	cd ../../; \
	echo -e "\nLDACSAbstractRadio"; \
	cd ldacs-abstract-radio/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET_dbg; make -j8 MODE=debug; cd ../..; \
	echo -e "\nLDACSAbstractTdma"; \
	cd ldacs-abstract-tdma/src; opp_makemake -f --deep -O out -KINET4_PROJ=../../inet4 -DINET_IMPORT -I../../inet4 -I../../ldacs-abstract-radio/src -I. -I../../inet4/src -L../../inet4/src -L../../ldacs-abstract-radio/out/gcc-debug/src/ -lINET_dbg -lldacs-abstract-radio_dbg; make -j$(NUM_CPUS) MODE=debug; \

build-release:	
	cd ../../; \
	echo -e "\nLDACSAbstractRadio"; \
	cd ldacs-abstract-radio/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET; make -j8 MODE=release; cd ../..; \
	echo -e "\nLDACSAbstractTdma"; \
	cd ldacs-abstract-tdma/src; opp_makemake -f --deep -O out -KINET4_PROJ=../../inet4 -DINET_IMPORT -I../../ldacs-abstract-radio/src -I. -I../../inet4/src -L../../inet4/src -L../../ldacs-abstract-radio/out/gcc-release/src/ -lINET -lldacs-abstract-radio; make -j$(NUM_CPUS) MODE=release
