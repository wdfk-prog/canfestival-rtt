
# Obsoleted by scripts in disthelper/scripts

# ADD
#  auto-guess indentation on .py files
#  other files require -w arg to 'tabify'
#
# make it work under python 2.0+
#======================================================================
#
# textify.py - Perform various text transformations on a
#              set of files.
#
# Can do the following:
#
#       tabify file
#       untabify file
#       convert file to portable text format (\n)
#       convert file to platform-specific text format
#
# Examples:
#    Recursively tabify all .py, .pl, and .c files under /home/frankm:
#        textify.py -t -r -x py,pl,c /home/frankm
#
#    Recursively untabify all .py, .pl, and .c files under /home/frankm
#    and convert to portable text format:
#        textify.py -t --to-portable -r -x py,pl,c /home/frankm
#
#    Recursively convert all README* files to platform-specific
#    text format:
#        textify.py --to-platform -R "README*"
#
# Requires Python 2.3+
# (Would only be run by developers, so this seems OK to me.)
#
# [The core tab/untabification logic was adapted from IDLE.]
#
#======================================================================
# There is nothing here that couldn't be done with simple shell
# commands and tools under a POSIXy operating system. However, this
# program is meant for cross-platform use where those common tools
# may not exist.
#
# written by Frank McIngvale <frankm@hiwaay.net>
#======================================================================

from optparse import OptionParser
import os, re
from tempfile import mkstemp
from disthelper.treeops.lineops import *

# default tab width, settable with -w
TABWIDTH = 4

def transform_thing( thing, opts, regexlist ):
    "thing can be a file or directory"

    # create the list of transforms
    lineops = []

    if opts.tabify:
        lineops.append( tabify_line )

    if opts.untabify:
        lineops.append( untabify_line )

    if opts.to_platform:
        lineops.append( to_platform_text )

    if opts.to_portable:
        lineops.append( to_portable_text )

    # transform the thing
    
    if os.path.isfile( thing ):
        transform_file( lineops, thing )

    elif os.path.isdir( thing ) and \
         (opts.glob or opts.recursive ):
        transform_tree( lineops, root, regexlist )

    else:
        raise "Unable to transform '%s'" % thing
            
def transform_tree( lineops, root, regexlist=[] ):
    """Run list of transforms (lineops) on a directory
    tree at 'root', on files matching the regexlist."""

    for path,dnames,fnames in os.walk( root ):
        for name in fnames:
            for r in regexlist:
                if r.match( name ):
                    full = os.path.join(root, name)
                    transform_file(lineops, full)
                    break
                
def transform_file( lineops, filename ):
    """Run list of transforms (lineops) on a single file."""
    
    hout, tname = mkstemp()

    #print "CONVERT FILE ",filename,tname,lineop

    for line in open( filename, 'rb' ):
        # do transforms
        buf = line
        for op in lineops:
            buf = op(line)
            
        while len(buf):
            nr = os.write(hout, buf)
            buf = buf[nr:]

    os.close(hout)
    os.unlink(filename)
    os.rename(tname, filename)

# parse cmdline
o = OptionParser(usage="%prog [opts] filename ...\n\nPerform text transformations on a set of files.")
o.add_option("-t","--tabify",
             action='store_true', dest='tabify', default=False,
             help='Perform tabification (replace spaces with tabs).')
o.add_option("-u","--untabify",
             action='store_true', dest='untabify', default=False,
             help='Perform untabification (replace tabs with spaces).')
o.add_option('','--to-platform',
             action='store_true', dest='to_platform', default=False,
             help='Convert to platform-specific text format')
o.add_option('','--to-portable',
             action='store_true', dest='to_portable', default=False,
             help='Convert to portable text format (\\n line endings)')
o.add_option('-w','--tab-width', dest='tabwidth', default=TABWIDTH,
             type='int', help='Set TAB width (default=%d)' % TABWIDTH)
o.add_option("-r","--recursive",
             action='store_true', dest='recursive', default=False,
             help='Include subdirectories.')
o.add_option("-R","--recursive-glob",
             action='store', dest='glob', type='string', default=None,
             help='Recurse (like -r) but use GLOB recursively to match files to transform')
o.add_option("-x","--extension",type="string",
             action="append", dest="extlist", default=[],
             help="Transform only files with the given file extension(s) [seperate multiple extensions with commas, or use -x multiple times]")
o.add_option('-i','--ignore-case',
             action='store_true', dest='nocase', default=False,
             help='Ignore case when matching filenames with -x or -R')

opts, args = o.parse_args()

#print opts
#print args

# note that "-x py,pl -x c" will result in: ['py,pl', 'c'], so
# normalize to one extension per string. assume that, if the
# user forces extra spaces into the string by using quotes,
# that they really intended it, so don't worry about stripping them
extlist = []
if opts.extlist is not None:
    for ext in opts.extlist:
        extlist += ext.lower().split(',')

globlist = []

# First, make a regex out of each extension match
for ext in extlist:
    globlist.append( r'^.+\.%s$' % ext )

# Now add any glob pattern the user specified with -R
if opts.glob:
    # turn the shell-style glob into a regex
    g = opts.glob.replace('.',r'\.').replace('*','.*')
    g = '^' + g + '$'
    globlist.append(g)

# compile them all
reflags = 0
if opts.nocase:
    reflags = re.I
    
globlist = map(lambda x: re.compile(x,reflags), globlist)

# set user-selected tabwidth
TABWIDTH = opts.tabwidth

for arg in args:
    tab_or_untab_thing( arg, opts, globlist )


    
