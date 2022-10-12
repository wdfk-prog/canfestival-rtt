
#
# Tabify an entire directory tree, with filename matching.
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
from disthelper.treeops.lineops import tabify_line, copy_line
from indentcheck import *

# define the command-line arg parser
class argv_parser(TreeOptParser):

    def __init__(self):
        TreeOptParser.__init__(self,'tabtree.py',
                               'Tabify a directory tree, with filename matching.')

        # add my specific options
        self.add_intopt( 'w', 'tabwidth', 'tabwidth',
                         "Set the tab width. It is recommended you DON'T use this if you're\n\ttabifying Python code. However, for non-Python code, it is\n\tbetter to use this option.")

p = argv_parser()
opts,args = p.parse_argv(sys.argv)

if len(args) == 0:
    if len(opts.regexlist) > 0:
        # user gave a glob but no targets - add cwd
        args.append('.')
    else:
        # don't know what user wants
        p.show_usage()
        sys.exit(1)
    
# need a null transform
def copy_line(line):
    return line

class TabifyFileTransform(FileTransformFromLineOp):
    def __init__(self,tabwidth=None):
        FileTransformFromLineOp.__init__(self, self.my_tabify)
        self.TABWIDTH = tabwidth
        
    def my_tabify(self,line):
        return tabify_line(line, self.TABWIDTH)
    
    def process(self, file_out, file_in):

        # save original value to restore at end
        tw = self.TABWIDTH

        # set lineop, might change it below
        self.set_lineop( self.my_tabify )

        if has_tab_space_mixing( file_in ):
            # too dangerous to try tabifying with mixed line beginnings
            print "ERROR: Skipping file '%s' - has mixed tabs & spaces" % file_in.name
            self.set_lineop( copy_line )
                
        elif self.TABWIDTH is None:
            # -w not given, try and guess tabwidth
            (uses_tabs, tabwidth) = guess_indentation(file_in)
            #print "GUESSED ",uses_tabs,tabwidth
            if uses_tabs == 1:
                print "** NOTE ** %s is already tabified - not changing." % \
                      file_in.name
                self.set_lineop( copy_line )
            elif uses_tabs < 0 and tabwidth < 0 and self.TABWIDTH < 0:
                print "** NOTE ** Can't determine tab settings for %s - not changing." % \
                      file_in.name
                self.set_lineop( copy_line )
            elif uses_tabs == 0:
                self.TABWIDTH = tabwidth

        #print "TABWIDTH NOW ",self.TABWIDTH
        FileTransformFromLineOp.process(self, file_out, file_in)

        self.TABWIDTH = tw

# make a file operation
fileop = TabifyFileTransform(opts.tabwidth)

# ... into a tree operation
treeop = TreeOpFromFileTransform( fileop )

# run the tree
treeop.runtree(opts, args)
