#---------------------------------------------------------------
# find_python.py
#
# Routines for enumerating the Pythons available on a
# machine, and finding a Python matching certain criteria.
#
# Can be used as a module or standalone script.
#
# frankm@hiwaay.net
#---------------------------------------------------------------
#
# This works under all Pythons since 1.5.2, so some of the
# syntax looks a bit outdated :-)
#
# Note that Python 2.0+ is recommended under win32 so that
# the registry can be accessed.
#
#---------------------------------------------------------------

# TODO: match os.name and/or sys.platform in matching routines

import sys

# make disthelper accessible
sys.path.insert(0,'.')
import scripts.grab_disthelper

# now the real code begins ...

import os, re, string
from stat import *  
from disthelper.misc import make_tempdir, unlink

# this can be used as a module or script. here are the
# public functions ...
__all__ = ['get_python_verlist','find_py_atleast','find_py_between',
           'find_py_minor']

try:
    # using _winreg requires Python 2.0+, however we can detect
    # Python version back to (at least) 1.5.2 since they all
    # follow the same registration scheme.
    from _winreg import OpenKey, HKEY_LOCAL_MACHINE, EnumKey, \
         QueryInfoKey, QueryValueEx
    HAVE_WIN32_REGISTRY = 1
except:
    HAVE_WIN32_REGISTRY = 0

# to make version comparisons easy, I make the 'bold' assumption that
# no digit of the version # will be greater than 255, and calculate
# a linear value here.
def canon_ver( ver ):
    return ver[0]*256*256 + ver[1]*256 + ver[2]

def get_pythons_from_registry():
    """Search the win32 registry for installed Pythons.
    Returns a list of Python executables for all the Pythons
    installed on the host machine."""

    if HAVE_WIN32_REGISTRY == 0:
        return []
    
    # get the toplevel key
    topkey = OpenKey(HKEY_LOCAL_MACHINE,"SOFTWARE\\Python\\PythonCore")
    
    # under PythonCore will be subkeys like '2.0', '2.1', etc.
    nr_vers = QueryInfoKey(topkey)[0]
    namelist = []

    # for each of those keys, get the InstallPath
    for i in range(nr_vers):
        verkey = OpenKey(topkey, "%s\\InstallPath" % EnumKey(topkey,i))
        path,typ = QueryValueEx(verkey,None)
        name = os.path.join(path,'python.exe')
        if os.path.isfile(name):
            namelist.append(name)

    return namelist

def find_pythons_in_dir( dirname ):
    """
    Find everything that looks like a Python executable
    in the given directory. Returns a list containing
    the fullpath to each executable.
    """

    # I could be more strict here, and change the pattern
    # based on OS, but that gets messy, and there is no way
    # around the underlying flaw that a malicious file could
    # sneak in here. But that's a generic flaw, and the reason
    # why you aren't supposed to put '.' in your PATH on POSIXy
    # systems.
    patt = re.compile('^python[0-9\.]*(\.exe)?$',re.I)

    found = []
    for name in os.listdir(dirname):
        if patt.match( name ):
            found.append( os.path.join(dirname, name) )

    return found

def find_all_pythons():
    """Search system for all Python executables. The returned
    list may contain duplicates."""
    
    allpys = []
    
    # split PATH according to platform rules
    pathlist = string.split( os.environ['PATH'], os.pathsep )

    # search PATH, excluding nonexistant dirs
    for path in filter( os.path.isdir, pathlist ):
        allpys.extend( find_pythons_in_dir( path ) )

    # check the win32 registry, as appropriate
    allpys.extend( get_pythons_from_registry() )

    # and of course I'm running under a Python, in case
    # no others were found
    allpys.append( os.path.abspath(sys.executable) )
    
    return allpys

def get_pyver_from_exe( exename ):
    """
    Given a python executable, find out its version.
    Returns version as 3-item list:

        (os.name, sys.platform, version)

    Where version is a 3-element list, e.g. [2,1,3]

    Returns None if can't get version.
    """

    # hack: when running a win32-native Python from a cygwin shell,
    # with cygwin Python installed, you'll see cygwin symlinks to the
    # real cygwin Python, but NTVDM.exe will crash when trying to run
    # the symlinks. Of course, to win32, they aren't links, so I can't
    # just filter them with islink(). Instead, I check if the binary
    # looks too small to be real.
    if os.stat(exename)[ST_SIZE] < 1000:
        return None
        
    # this is required to work on Python 1.5.2
    # note that splitting sys.version doesn't work on .0 releases, so
    # I try sys.version_info first, but it isn't available on 1.5.2.

    # Don't insert any lefthand spaces!
    pycmd = """
import sys, string, os
try: v = sys.version_info[0],sys.version_info[1],sys.version_info[2]
except: v = map(int,string.split(string.split(sys.version)[0],'.'))
open('lineout','w').write('%s %s %s %s %s\\n' % (os.name,sys.platform,v[0],v[1],v[2]))
"""

    # the most portable thing to do is write pycmd to a file, and
    # have pycmd write its results to a file as well. so make
    # a temp directory to run from.
    
    savedir = os.getcwd()

    tempdir = make_tempdir()
    os.chdir(tempdir)

    f = open('test.py','w')
    f.write( pycmd )
    del f # explicit close seems to be needed under win32 (i.e. open().write() fails)

    os.system('%s test.py' % exename)
    if not os.path.isfile('lineout'):
        return None # failed to run
    
    f = open('lineout','r')
    line = f.readline()
    del f # explicitly, for win32

    unlink('lineout')
    unlink('test.py')

    os.chdir(savedir)
    os.rmdir(tempdir)

    p = line.split()
    
    return (p[0], p[1], map( int, p[2:] ))
    
def get_python_verlist():
    """
    Returns a list of all Pythons available on the host system.
    The list is guaranteed to not contain duplicates (i.e. if
    the host system has 'python' symlinked to 'python2.3', that
    will be caught and only one will be entered in the list; however
    there is no guarantee which particular one will be removed).

    Returns list of tuples:
        (exe_fullpath, info)

    Where info is the tuple from get_pyver_from_exe()
    """ 
    l = []
    fv = []
    
    for pyexe in find_all_pythons():
        v = get_pyver_from_exe(pyexe)
        if v != None and v not in fv: # watch for duplicates
            l.append( (pyexe, v) )
            fv.append(v)

    return l

def find_py_atleast( minver ):
    """
    Find a Python executable in the local machine PATH of
    at least minver.
    minver is a tuple giving the minumum version. i.e. to
    search for 2.0.x, pass (2,0,0).

    Note: There is no guarantee the returned version
    will be the highest (or lowest) version that satisfies
    the criteria.
    """
    wantver = canon_ver(minver)

    for pyexe, info in get_python_verlist():
        
        thisver = canon_ver(info[2])

        if thisver >= wantver:
            return pyexe

    # can't satisfy requirement
    return None

def find_py_between( minver, maxver ):
    """
    Find a Python executable in the local machine PATH of
    at least minver, and at most maxver.

    Note: There is no guarantee the returned version
    will be the highest (or lowest) version that satisfies
    the criteria.
    """

    minver = canon_ver(minver)
    maxver = canon_ver(maxver)

    for pyexe, info in get_python_verlist():
        
        thisver = canon_ver(info[2])

        if thisver >= minver and thisver <= maxver:
            return pyexe

    # can't satisfy requirement
    return None

def find_py_minor( ver ):
    # I make the same assumption here on max#'s as in canon_ver
    return find_py_between( [ver[0],ver[1],0],
            [ver[0],ver[1],255] )

def usage():
    print "Usage: find_python args"
    print ""
    print "Find a Python executable on the local machine satisfying"
    print "user-specified criteria."
    print ""
    print "args can be one of:"
    print ""
    print "   show"
    print "      List all Pythons in a human-readable way."
    print ""
    print "   atleast version"
    print "      Find a Python of at least the given version number."
    print "      (version can be like '2', '2.1', '2.2.1')"
    print ""
    print "   between ver1 ver2"
    print "      Find a Python at least ver1 and not higher than ver2"
    print ""
    print "   match-minor version"
    print "      Find a Python matching the minor version"
    print "      (i.e. '2.0' matches 2.0.0, 2.0.1, .. , but NOT 2.1+)"
    print ""
    sys.exit(1)

def parse_version_string( ver ):

    # this will catch errors, like passing non-numeric strings
    l = map(int, string.split(ver,'.'))
    if len(l) > 3:
        raise "Version string too long"

    # pad with 0's
    return l + [0]*(3-len(l))

if __name__ == '__main__':

    if len(sys.argv) < 2:
        usage()

    if sys.argv[1] == 'show':
        pylist = find_all_pythons()
        print "Here are all the Pythons I found:"
        for name in pylist:
            print "\t%s" % name

        print "Here is the minimum set, with version numbers:"
        verlist = get_python_verlist()
        for exe,ver in verlist:
            print "\t%s = Python %s, %s, %d.%d.%d" % (exe,ver[0],ver[1],
                                                      ver[2][0],ver[2][1],ver[2][2])
        
    if sys.argv[1] == 'atleast':
        if len(sys.argv) != 3:
            usage()

        print find_py_atleast( parse_version_string(sys.argv[2]) )

    elif sys.argv[1] == 'between':
        if len(sys.argv) != 4:
            usage()

        print find_py_between( parse_version_string(sys.argv[2]),
               parse_version_string(sys.argv[3]) )

    elif sys.argv[1] == 'match-minor':
        if len(sys.argv) != 3:
            usage()

        print find_py_minor( parse_version_string(sys.argv[2]) )
    
