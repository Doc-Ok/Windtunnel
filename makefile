########################################################################
# Makefile for "windtunnel" (particle simulation of hard spheres)
# project.
# Copyright (c) 2001-2020 Oliver Kreylos
#
# This file is part of the Virtual Wind Tunnel package.
# 
# The Virtual Wind Tunnel package is free software; you can redistribute
# it and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
# 
# The Virtual Wind Tunnel package is distributed in the hope that it
# will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
# the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the Virtual Wind Tunnel package; if not, write to the Free
# Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307 USA
########################################################################

# Set directory containing Vrui's build system:
VRUI_MAKEDIR := $(HOME)/Vrui-5.3/share/make
ifdef DEBUG
  VRUI_MAKEDIR = $(VRUI_MAKEDIR)/debug
endif

# Base installation directory for the Windtunnel package. If this is set
# to the default of $(PWD), the Windtunnel package does not have to be
# installed to be run. The Windtunnel package's executable will be
# installed in the bin directory underneath the given base directory.
INSTALLDIR := $(shell pwd)

########################################################################
# Everything below here should not have to be changed
########################################################################

# Version number for installation subdirectories. This is used to keep
# subsequent release versions of the Windtunnel package from clobbering
# each other. The value should be identical to the major.minor version
# number found in VERSION in the root package directory.
VERSION = 1.0

# Include definitions for the system environment and system-provided
# packages
include $(VRUI_MAKEDIR)/SystemDefinitions
include $(VRUI_MAKEDIR)/Packages.System
include $(VRUI_MAKEDIR)/Configuration.Vrui
include $(VRUI_MAKEDIR)/Packages.Vrui

# Set installation directory structure:
EXECUTABLEINSTALLDIR = $(INSTALLDIR)/$(EXEDIR)

########################################################################
# List common packages used by all components of this project
# (Supported packages can be found in $(VRUI_MAKEDIR)/Packages.*)
########################################################################

PACKAGES = MYVRUI

########################################################################
# Specify all final targets
########################################################################

ALL = $(EXEDIR)/CollisionBoxTest
.PHONY: all
all: $(ALL)

########################################################################
# Specify other actions to be performed on a `make clean'
########################################################################

.PHONY: extraclean
extraclean:

.PHONY: extrasqueakyclean
extrasqueakyclean:

# Include basic makefile
include $(VRUI_MAKEDIR)/BasicMakefile

########################################################################
# Specify build rules for executables
########################################################################

COLLISIONBOXTEST_SOURCES = CollisionBox.cpp \
                           CollisionBoxTest.cpp

$(EXEDIR)/CollisionBoxTest: $(COLLISIONBOXTEST_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: CollisionBoxTest
CollisionBoxTest: $(EXEDIR)/CollisionBoxTest

install: $(ALL)
	@echo Installing the Windtunnel package in $(INSTALLDIR)...
	@install -d $(INSTALLDIR)
	@install -d $(EXECUTABLEINSTALLDIR)
	@install $(ALL) $(EXECUTABLEINSTALLDIR)
