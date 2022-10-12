#
# Detect indentation style of Python code
#
# frankm@hiwaay.net
#

__all__ = ['guess_indentation','has_tab_space_mixing']

# make sure I can import disthelper
import sys
sys.path.insert(0,'.')
import grab_disthelper

from tokenize import tokenize, INDENT, ERRORTOKEN

class IndentDetector:
    """
    Detect the indentation style of a Python source file.
    Sometimes works on other file types, but no guarantee.

    This class hides the ugly details of using the Python 1.5.2-compatible
    tokenize() function.
    """
    
    def __init__(self, fileobj):
        self.fileobj = fileobj
        self.found_indent = 0
        
    def readline(self):
        # at first, I would return EOF here after finding the INDENT.
        # however, that leads to an exception if you EOF in the middle
        # of a multi-line statement, so skip the optimization.
        return self.fileobj.readline()

    def token_eater(self, ttype, tokstr, start, end, line):
        if ttype == INDENT:
            if self.found_indent:
                return # only count the first one!
            
            self.found_indent = 1
            
            if line[0] == '\t':
                # file uses tabs
                self.uses_tabs = 1
                self.tabwidth = 0
            else:
                # file uses spaces
                self.uses_tabs = 0
                self.tabwidth = end[1] - start[1]

        elif ttype == ERRORTOKEN:
            self.uses_tabs = -1
            self.tabwidth = -1

class MixedIndentDetector:
    """
    Detect if a Python file has mixed tabs & spaces for indentation.
    Sometimes works on other file types, but no guarantee.

    This class hides the ugly details of using the Python 1.5.2-compatible
    tokenize() function.
    """
    
    def __init__(self, fileobj):
        self.fileobj = fileobj
        self.found_tab = -1
        self.errlist = []
        
    def readline(self):
        return self.fileobj.readline()

    def token_eater(self, ttype, tokstr, start, end, line):
        if ttype == INDENT:
            if line[0] == '\t':
                if self.found_tab == 0:
                    # already found indent w/space - error
                    self.errlist.append('<TAB> ' + line)
                else:
                    self.found_tab = 1
            elif line[0] == '\x20':
                if self.found_tab == 1:
                    # already found indent w/tab - error
                    self.errlist.append('<SPACE> ' + line)
                else:
                    self.found_tab = 0

        elif ttype == ERRORTOKEN:
            self.errlist.append('<COMPILATION ERROR> ' + line)

def get_tab_space_mixing( fileobj ):
    
    # need to restore ofs when finished
    ofs = fileobj.tell()

    o = MixedIndentDetector(fileobj)

    tokenize( o.readline, o.token_eater )

    # restore position
    fileobj.seek(ofs)

    return o.errlist

def has_tab_space_mixing( fileobj ):
    """
    Check if fileobj has mixed tabs & spaces.
    If so, you should NOT run guess_indentation, or do any sort
    of tab/untab on the file as you'd likely screw it up.
    """
    return (len( get_tab_space_mixing(fileobj) ) > 0)		
            
def guess_indentation( fileobj ):
    """
    Given a file-like object, guess its indentation style.
    This is intended for use on Python source code, but
    works (somewhat) on random text files as well.
    
    Returns:
       (uses_tabs, tabwidth)

    If uses_tabs == 1, the file uses tab characters for
    indentation, and tabwidth isn't used.

    If uses_tabs == 0, the file uses spaces for indentation
    and tabwidth is the number of spaces per indent.

    The file offset of fileobj is preserved. It is recommended
    that you pass fileobj with its offset set to zero, but
    it is not required.

    If the file contains no indentations, the return value is (-1,-1).
    """

    # need to restore ofs when finished
    ofs = fileobj.tell()

    o = IndentDetector(fileobj)

    tokenize( o.readline, o.token_eater )

    # restore position
    fileobj.seek(ofs)

    if not hasattr(o,'uses_tabs'):
        # file had no indentations, so it doesn't matter
        # what values I use
        return (0,4)
    else:
        return (o.uses_tabs, o.tabwidth)
    
#
# Can also run a standalone script to show problems over
# a tree.
#
# now the real code begins ...

from disthelper.treeops.treeops import *
import os
from stat import *

class TreeChecker(TreeOps):
    """Example of a TreeOp that checks indentation."""
    
    def __init__(self):
        TreeOps.__init__(self)

    def run(self, argv):
        # parse standard tree options (-r, -R, -x, etc.)
        p = TreeOptParser('indentcheck.py','Check indentation of a directory tree.')
        opts,args = p.parse_argv(argv)

        if len(args) == 0:
            print "** Must give a directory and/or file to check."
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

        fileobj = open(fullname,'r')
        errs = get_tab_space_mixing(fileobj)
        if len(errs):
            s = "ERROR: '%s' has mixed tab/space line beginnings." % fullname
            if not opts.verbose:
                s += ' (use -v for details)'

            print s

        if opts.verbose and len(errs):
            print "Bad lines are shown below:"
            for line in errs:
                print '    '+line		
        
    def process_one_dir(self,fullname):
        pass

    def dir_noaccess(self,fullname):
        self.nodir.append(fullname)
        
if __name__ == '__main__':

    t = TreeChecker()

    # special case, if user gave me no switches (except
    # possibly -v), but includes dirnames, add the standard
    # options that make sense (-r -x py) for convenience.

    alldirs = 1
    for arg in sys.argv[1:]:
        if arg != '-v' and not os.path.isdir(arg):
            alldirs = 0

    if len(sys.argv) > 1 and alldirs:
        t.run(['dummy','-r','-x','py'] + sys.argv[1:])	
    else:
        t.run(sys.argv)

            
