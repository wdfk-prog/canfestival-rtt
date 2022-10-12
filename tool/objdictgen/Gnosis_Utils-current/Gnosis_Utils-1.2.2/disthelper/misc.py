
#
# More handy functions I found myself reinventing every
# time I made a setup.py
#
# This should be compatible back to Python 1.5.2.
#
# frankm@hiwaay.net
#

import os, string
from stat import *
import shutil

# public interface
__all__ = ['mtime','find_in_path','find_exe_in_path','unlink','make_tempdir',
           'samepath']

def mtime(file): # convenience
    return os.stat(file)[ST_MTIME]

def __normpath_for_comp(path):
    # normalize path for textual comparison
    #
    # this does an implicit os.path.normpath() as well
    return os.path.normcase(os.path.abspath(path))

def samepath(path1,path2):
    "Test for equality of path names. Neither path has to exist."
    
    # device:inode comparison can only be done on posix & mac (OS X).
    # however, using inodes requires that the file exists.
    # I want this function to work on non-existant paths as well.
    # (I may add a future 'samefile'/'samedir' to check for existing
    # files/dirs)
    #
    # so, do the best I can without inodes ...
    return __normpath_for_comp(path1) == __normpath_for_comp(path2)

def find_in_path( filename ):
    """Search PATH for the named file.
    Returns full pathname if found, None if not."""

    pathlist = string.split( os.environ['PATH'], os.pathsep )

    for path in filter( os.path.isdir, pathlist ):
        name = os.path.join( path,filename )
        if os.path.isfile(name):
            return name

    return None

def find_exe_in_path( basename ):
    """Find executable in PATH (after adding extension to
    basename, as appropriate).

    Returns full pathname, or None if not found."""

    if os.name == 'posix' and sys.platform[:6] == 'cygwin':
        return find_in_path( basename+'.exe')
    elif os.name == 'posix':
        return find_in_path( basename )
    elif os.name in ['nt','os2']:
        name = find_in_path( basename+'.exe' )
        if name is None:
            name = find_in_path( basename )
            
        return name
    else:
        return find_in_path( basename )

def unlink(filename):
    """An unlink() wrapper to work around win32 problems
    in some Python versions. This is like 'rm -f' - the file
    doesn't have to exist."""
    if not os.path.isfile(filename):
        return
    
    try:
        os.unlink(filename)
    except:
        os.remove(filename)

def rmtree(path):
    "Like 'rm -rf path'"
    if os.path.isdir(path):
        shutil.rmtree(path)
        
try:
    # use secure mkdtemp if available
    from tempfile import mkdtemp
    HAVE_MKDTEMP = 1
except:
    from tempfile import mktemp
    HAVE_MKDTEMP = 0
    
def make_tempdir():
    """Make a temporary directory, securely if possible.
    Creates and returns the full pathname. Caller is
    responsible for deleting dir when finished."""
    
    if HAVE_MKDTEMP:
        name = mkdtemp()
    else:
        name = mktemp()
        os.mkdir(name)

    return name

