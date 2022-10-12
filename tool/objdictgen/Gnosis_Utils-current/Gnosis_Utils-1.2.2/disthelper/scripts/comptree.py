
# Sample app showing how to use TreeOps
#
# This byte-compiles all matched files.
#
# frankm@hiwaay.net
#

# make sure I can import disthelper
import sys
sys.path.insert(0,'.')
import grab_disthelper

# now the real code begins ...

from disthelper.treeops.treeops import *
import os
from stat import *
from time import strftime, localtime
import compiler

class TreeCompiler(TreeOps):
    """Example of a TreeOp that byte-compiles matched files."""
    
    def __init__(self):
        TreeOps.__init__(self)

    def run(self, argv):
        # parse standard tree options (-r, -R, -x, etc.)
        p = TreeOptParser('comptree.py','Byte-compile a tree of files.')		
        opts,args = p.parse_argv(argv)

        if len(args) == 0:
            print "** Must give a directory and/or file to byte-compile."
            p.show_usage()
            sys.exit(1)
            
        # remember which files/dirs we couldn't access
        self.nofile = []
        self.nodir = []

        # walk the tree with globbing, etc.
        self.runtree(opts,args)

        # tell user which files/dirs I couldn't access
        if len(self.nofile):
            print "I could not access these files:"
            for f in self.nofile:
                print "  %s" % f

        if len(self.nodir):
            print "I could not access these directories:"
            for d in self.nodir:
                print "  %s" % d
    
    # - internal API - called as the tree is walked -
    def process_one_file(self,fullname,opts):
        print "Byte-compiling %s ..." % fullname
        compiler.compileFile(fullname)
        
    def process_one_dir(self,fullname):
        pass

    def dir_noaccess(self,fullname):
        self.nodir.append(fullname)

t = TreeCompiler()

# special case, if user gave me no switches, but
# includes dirnames, add the standard options that
# make sense (-r -x py) for convenience.

alldirs = 1
for arg in sys.argv[1:]:
    if not os.path.isdir(arg):
        alldirs = 0

if len(sys.argv) > 1 and alldirs:
    t.run(['dummy','-r','-x','py'] + sys.argv[1:])	
else:
    t.run(sys.argv)


