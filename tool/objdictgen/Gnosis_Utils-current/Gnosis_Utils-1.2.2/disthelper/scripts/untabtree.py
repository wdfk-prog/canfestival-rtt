
#
# Untabify an entire tree, with filename matching.
#
# frankm@hiwaay.net
#

# make sure I can import disthelper
import sys
sys.path.insert(0,'.')
import grab_disthelper

# now the real code begins ...

from disthelper.treeops.treeops import *
from disthelper.treeops.fileops import *
from disthelper.treeops.lineops import untabify_line, copy_line
from indentcheck import *

# define the command-line arg parser
class argv_parser(TreeOptParser):

    def __init__(self):
        TreeOptParser.__init__(self,'untabtree.py',
                               'Untabify a directory tree, with filename matching.')

        # add my specific options
        self.add_intopt( 'w', 'tabwidth', 'tabwidth',
                         "Set the tab width (REQUIRED)" )

p = argv_parser()
opts,args = p.parse_argv(sys.argv)

if opts.tabwidth is None:
    print "** ERROR: You must specify a tab-width with -w"
    p.show_usage()
    sys.exit(1)

if len(args) == 0:
    if len(opts.regexlist) > 0:
        # user gave a glob but no targets - add cwd
        args.append('.')
    else:
        # don't know what user wants
        p.show_usage()
        sys.exit(1)

class UntabifyFileTransform(FileTransformFromLineOp):
    def __init__(self,tabwidth):
        FileTransformFromLineOp.__init__(self, self.my_untabify)
        self.tabwidth = tabwidth
        
    def my_untabify(self,line):
        return untabify_line(line, self.tabwidth)

    def process(self, file_out, file_in):
        if has_tab_space_mixing( file_in ):
            # too dangerous to try untabifying with mixed line beginnings			
            print "ERROR: Skipping file '%s' - has mixed tabs & spaces" % file_in.name
            self.set_lineop( copy_line )
        else:
            self.set_lineop( self.my_untabify )
        
        FileTransformFromLineOp.process(self, file_out, file_in)

# make a file operation
fileop = UntabifyFileTransform(opts.tabwidth)

# ... into a tree operation
treeop = TreeOpFromFileTransform( fileop )

# ... and run the tree
treeop.runtree(opts, args)


