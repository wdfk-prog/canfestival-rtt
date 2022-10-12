
#
# Recursively remove specific files.
#
# This is a portable replacement for simple uses of rm `find ..`.
#
# Example:
#      POSIX:
#          rm `find src -name "*.pyc"`
#
#      rmfind:
#          python rmfind.py -R "*.pyc" src
#      -or-
#          python rmfind.py -x pyc src
#
# Use -h for help.
#
#
# frankm@hiwaay.net
#

# make sure I can import disthelper
import sys
sys.path.insert(0,'.')
import grab_disthelper

# now the real code begins ...

from disthelper.treeops.treeops import *
from disthelper.misc import unlink

class TreeOpRmFind(TreeOps):
    "TreeOps worker"

    # - internal API - called by TreeOps -
    def process_one_file(self,fullname,opts):
        "Called for each matched file."
        if opts.verbose:
            print "rm",fullname

        unlink(fullname)
        
    def process_one_dir(self,fullname):
        "Called for each directory along the way."
        pass

    def dir_noaccess(self,fullname):
        """Called when access is denied to a directory
        (strictly informational, there is no provision to
        retry the operation)."""
        pass
    

p = TreeOptParser('rmfind.py',
                  'Recursively remove files matching a pattern.')

opts,args = p.parse_argv(sys.argv)

#print opts
#print args

if len(args) == 0:
    # do NOT automatically add '.' as a target, since this
    # is such a destructive command
    p.show_usage()
    sys.exit(1)

if len(opts.regexlist) == 0:
    # safety ... running with no args means "rm *".
    print "***"
    print "*** No pattern given - I'm assuming you DON'T really mean `rm *` or `rm -rf *`"
    print "*** If you really want to do `rm -rf *`, then use -R \"*\""
    print "***"
    sys.exit(1)
    
# Run TreeOpRmFind on the tree
treeop = TreeOpRmFind()
treeop.runtree(opts,args)
