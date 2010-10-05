all: installed

# We build from a tarball of an svn export of gmapping at revision 39.  To
# get the original code (e.g., to make a new patch):
#  svn co -r 39 https://openslam.informatik.uni-freiburg.de/data/svn/gmapping/trunk gmapping-svn
SVN_DIR = ../rviz
SVN_URL = https://code.ros.org/svn/ros-pkg/stacks/visualization/trunk/rviz
SVN_REVISION = -r 30593
SVN_PATCH = rviz-r30593.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched Makefile
#	cd $(SVN_DIR) && patch -p0 < ../../$(PATCH)
	cd $(SVN_DIR) && rosmake
	touch installed

wipe: clean
	rm -rf $(SVN_DIR)

wiped: Makefile.rviz $(PATCH)
	make -f Makefile.rviz wipe
	touch wiped

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf $(SVN_DIR) patched installed

