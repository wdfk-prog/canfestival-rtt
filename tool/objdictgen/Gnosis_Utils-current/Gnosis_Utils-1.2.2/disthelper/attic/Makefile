#
# OBSOLETE - used to be the toplevel in Gnosis-master
#
# frankm@hiwaay.net
#

#
# This Makefile is unixy, sorry.
#

TOPDIR = .

all:

#
# I run "make edit" on the tree to make sure all
# files are in unix text format.
#
# You might want to edit this to suit your local machine.
#
edit: clean
	@echo "*** Removing ^M chars ***"
	python crlf.py `find $(TOPDIR) -name "*.py"`

formatout:
	@echo "*** Removing tabs ***"
	python untabify.py -t 4 `find $(TOPDIR) -name "*.py"`

	# put sources in unix textfile format, so auto-conversion
	# to platform format works (in setup.py)
	@echo "*** Converting to lf format **"
	python crlf.py `find $(TOPDIR) -name "*.py"`

	chown -R frankm *

#
# Run "make dist" to create the source distribution.
# (Use this instead of distutils directly since we
# need to prep some things.)
#
dist: formatout clean

	# create empty MANIFEST
	rm -f MANIFEST
	touch MANIFEST

	# create real MANIFEST, including MANIFEST in listing
	python setup.py sdist --manifest-only

	# create dist (which *includes* MANIFEST just created)
	# (yes, its gross, but required as far as I can tell)
	rm -rf dist
	python setup.py master
	python setup.py sdist
	chown -R frankm *

clean:
	rm -f `find $(TOPDIR) -name "*.pyc"`
	rm -f `find $(TOPDIR) -name "*~"`
	rm -f `find $(TOPDIR) -name "core"`
	rm -f gnosis/xml/pickle/test/TESTS.OUT-* MANIFEST
	rm -f gnosis/xml/pickle/test/aaa.xml
	rm -rf build dist




