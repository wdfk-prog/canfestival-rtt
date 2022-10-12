
# sample app showing how to use TreeOps
#
# this is a simple file lister, like 'ls -lR', demonstrating
# that it takes no special code to implement all the standard
# tree options (-r, -R, -x, -i, -h)
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

class TreeLister(TreeOps):
    """Example of a TreeOp that lists files recursively."""
    
    def __init__(self):
        TreeOps.__init__(self)
        self.nr_files = 0
        
    def run(self, argv):
        # parse standard tree options (-r, -R, -x, etc.)
        p = TreeOptParser('dirtree.py','Show file/directory listing.')		
        opts,args = p.parse_argv(argv)

        if len(args) == 0:
            print "** Must give a directory to list."
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
        try:
            st = os.stat(fullname)
            self.nr_files += 1
        except:
            self.nofile.append(fullname)
            return
        
        print "%-30s %8d %s" % (fullname,st[ST_SIZE],
                                strftime('%Y-%m-%d',localtime(st[ST_MTIME])))

    def process_one_dir(self,fullname):
        print "\nDIRECTORY %s" % fullname
        print "-------------------------------------------------"

    def dir_noaccess(self,fullname):
        self.nodir.append(fullname)
        
t = TreeLister()
t.run(sys.argv)
print "Listed %d files." % t.nr_files



