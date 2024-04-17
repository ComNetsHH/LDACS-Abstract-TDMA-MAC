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
	echo -e "\nUdpBroadcastApp"; \
	cd udpbroadcastapp/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET; make -j8 MODE=release; cd ../..; \
	echo -e "\nLDACSAbstractRadio"; \
	cd ldacs-abstract-radio/src; opp_makemake --make-so -f --deep -KINET_PROJ=../../inet4 -DINET_IMPORT -I../../inet4/src -L../../inet4/src -lINET; make -j8 MODE=release; cd ../..; \
	echo -e "\nLDACSAbstractTdma"; \
	cd ldacs-abstract-tdma/src; opp_makemake -f --deep -O out -KINET4_PROJ=../../inet4 -DINET_IMPORT -I../../inet4 -I../../udpbroadcastapp/src -I../../ldacs-abstract-radio/src -I. -I../../inet4/src -L../../inet4/src -L../../ldacs-abstract-radio/out/gcc-release/src/ -L../../udpbroadcastapp/out/gcc-release/src/ -lINET  -ludpbroadcastapp -lldacs-abstract-radio; make -j$(NUM_CPUS) MODE=release


# GLOBAL VARIABLES  can be shared among all targets
# default binary in RELEASE mode
SIM_BINARY = ../out/gcc-release/src/ldacs-abstract-tdma
SIM_BINARY_DBG = ../out/gcc-debug/src/ldacs-abstract-tdma_dbg
# include .ned files
INCL = -n ../src:./:../../inet4/src/:../../udpbroadcastapp/src:../../ldacs-abstract-radio/src

# END-to-END SIMUlATIONS

LDACS_ABSTRACT_TDMA_CONFIGNAME = General
LDACS_ABSTRACT_TDMA_FULL_BOTH_CONFIGNAME = full-both
LDACS_ABSTRACT_TDMA_FULL_UNICAST_CONFIGNAME = full-unicast
LDACS_ABSTRACT_TDMA_FULL_BROADCAST_CONFIGNAME = full-broadcast
LDACS_ABSTRACT_TDMA_EMPTY_BOTH_CONFIGNAME = empty-both

# run from ldacs-abstract-tdma/simulations
ldacsAbstractTdma-qtenv:
	opp_runall -j1 $(SIM_BINARY) omnetpp.ini -c $(LDACS_ABSTRACT_TDMA_CONFIGNAME) $(INCL) -u Qtenv

ldacsAbstractTdma-debug:
	gdbgui --args $(SIM_BINARY_DBG) omnetpp.ini -u Cmdenv -c $(LDACS_ABSTRACT_TDMA_CONFIGNAME) $(INCL)

ldacsAbstractTdma_fullBuffersBoth-qtenv:
	opp_runall -j1 $(SIM_BINARY) omnetpp_full_buffers.ini -c $(LDACS_ABSTRACT_TDMA_FULL_BOTH_CONFIGNAME) $(INCL) -u Qtenv

ldacsAbstractTdma_fullBuffersBoth-debug:
	gdbgui --args $(SIM_BINARY_DBG) omnetpp_full_buffers.ini -u Cmdenv -c $(LDACS_ABSTRACT_TDMA_FULL_BOTH_CONFIGNAME) $(INCL)

ldacsAbstractTdma_fullBuffersUnicast-qtenv:
	opp_runall -j1 $(SIM_BINARY) omnetpp_full_buffers.ini -c $(LDACS_ABSTRACT_TDMA_FULL_UNICAST_CONFIGNAME) $(INCL) -u Qtenv

ldacsAbstractTdma_fullBuffersUnicast-debug:
	gdbgui --args $(SIM_BINARY_DBG) omnetpp_full_buffers.ini -u Cmdenv -c $(LDACS_ABSTRACT_TDMA_FULL_UNICAST_CONFIGNAME) $(INCL)

ldacsAbstractTdma_fullBuffersBroadcast-qtenv:
	opp_runall -j1 $(SIM_BINARY) omnetpp_full_buffers.ini -c $(LDACS_ABSTRACT_TDMA_FULL_BROADCAST_CONFIGNAME) $(INCL) -u Qtenv

ldacsAbstractTdma_fullBuffersBroadcast-debug:
	gdbgui --args $(SIM_BINARY_DBG) omnetpp_full_buffers.ini -u Cmdenv -c $(LDACS_ABSTRACT_TDMA_FULL_BROADCAST_CONFIGNAME) $(INCL)

ldacsAbstractTdma_emptyBuffersBoth-qtenv:
	opp_runall -j1 $(SIM_BINARY) omnetpp_full_buffers.ini -c $(LDACS_ABSTRACT_TDMA_EMPTY_BOTH_CONFIGNAME) $(INCL) -u Qtenv

ldacsAbstractTdma_emptyBuffersBoth-debug:
	gdbgui --args $(SIM_BINARY_DBG) omnetpp_full_buffers.ini -u Cmdenv -c $(LDACS_ABSTRACT_TDMA_EMPTY_BOTH_CONFIGNAME) $(INCL)

ldacsAbstractTdmaWithoutRouting-qtenv:
	opp_runall -j1 $(SIM_BINARY) omnetpp_without_routing.ini -c $(LDACS_ABSTRACT_TDMA_CONFIGNAME) $(INCL) -u Qtenv

ldacsAbstractTdmaWithoutRouting-debug:
	gdbgui --args $(SIM_BINARY_DBG) omnetpp_without_routing.ini -u Cmdenv -c $(LDACS_ABSTRACT_TDMA_CONFIGNAME) $(INCL)

ldacsAbstractTdmaWithoutRouting-memory-valgrind:
	valgrind --leak-check=full --show-leak-kinds=all $(SIM_BINARY) omnetpp_without_routing.ini -u Cmdenv -c $(LDACS_ABSTRACT_TDMA_CONFIGNAME) $(INCL)