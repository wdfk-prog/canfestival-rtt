
#
# Convert a tree of files into portable (\n) text format,
# with filename matching.
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
from disthelper.treeops.fileops import *
from disthelper.treeops.lineops import to_portable_text

p = TreeOptParser('porttext.py',
                  'Convert a tree to portable (\\n) text format, with filename matching.')

opts,args = p.parse_argv(sys.argv)

#print opts
#print args

if len(args) == 0:
    if len(opts.regexlist) > 0:
        # user gave a glob but no targets - add cwd
        args.append('.')
    else:
        # don't know what user wants
        p.show_usage()
        sys.exit(1)

# make a file transform from the lineop
fileop = FileTransformFromLineOp( to_portable_text )

# now make into a tree operation
treeop = TreeOpFromFileTransform( fileop )

# and run the tree
treeop.runtree(opts, args)
