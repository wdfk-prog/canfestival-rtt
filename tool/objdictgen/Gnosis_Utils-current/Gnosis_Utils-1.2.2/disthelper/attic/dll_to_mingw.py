
#
# In order to use mingw (or cygwin with 'gcc -mno-cygwin')
# to create native win32 extension modules, you must create
# import libraries for the Python DLLs that gcc can use.
#
# This script automates the procedure found at:
#     http://sebsauvage.net/python/mingw.html
#
# [excluding the parts about modifying distutils]
#
#
# frankm@hiwaay.net
#

# This is compatible back to Python 1.5.2, hence the
# old syntax here and there.

#from disthelper.find_python import *
#import disthelper.find_python as find_python
#from disthelper.misc import *
import sys, string, os
from shutil import copy2

try:
    # _winreg requires Python 2.0+, so make it optional
    from _winreg import OpenKey, HKEY_LOCAL_MACHINE, EnumKey, \
         QueryInfoKey, QueryValueEx
    HAVE_WIN32_REGISTRY = 1
except:
    HAVE_WIN32_REGISTRY = 0

try:
    # use win32all, if installed
    import win32api
    HAVE_WIN32_API = 1
except:
    HAVE_WIN32_API = 0

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
        return find_in_path( basename+'.exe' )
    else:
        return find_in_path( basename )

def make_mingw_lib_from_dll( destname, dllname ):
    """Take a win32 DLL and create a lib*.a file suitable
    for linking with mingw.

    dllname is the full pathname of the .DLL to convert.
    destname is the name for the converted mingw library.

    This is an automation of the procedure found at:
        http://sebsauvage.net/python/mingw.html"""

    # since I chdir() below ...
    destname = os.path.abspath(destname)
    
    # make sure necessary progs are available
    dlltool = find_dlltool_or_bail()
    pexports = find_pexports_or_bail()

    print "Converting %s -> %s" % (dllname,destname)
    
    savedir = os.getcwd()

    # do the work in tempdir and copy resulting .a to correct place
    tempdir = make_tempdir()
    os.chdir(tempdir)

    copy2( dllname, tempdir )

    # create .def file
    cmd = "%s %s > temp.def" % \
          (pexports, os.path.basename(dllname))
    print "CMD: ",cmd
    os.system(cmd)
    
    # create .a
    cmd = "%s --dllname %s --def temp.def --output-lib %s" % \
          (dlltool, os.path.basename(dllname), destname)
    print "CMD: ",cmd
    os.system(cmd)

    # remove temporary files & tempdir
    unlink('temp.def')
    unlink(os.path.basename(dllname))
    os.chdir(savedir)
    os.rmdir(tempdir)

def find_dlltool_or_bail():
    "Find dlltool.exe, or bail out."
    
    name = find_exe_in_path('dlltool')
    if name is None:
        print "***"
        print "*** ERROR - dlltool.exe not found in PATH."
        print "***"
        print "*** Make sure you have installed gcc from either"
        print "*** cygwin or mingw."
        print "***"
        sys.exit(1)
    else:
        return name
    
def find_pexports_or_bail():
    "Find pexports.exe or bail out."
    
    name = find_exe_in_path('pexports')
    if name is None:
        print "***"
        print "*** ERROR - pexports.exe not found in PATH."
        print "*** Please download it from:"
        print "***   http://starship.python.net/crew/kernr/mingw32/pexports-0.42h.zip"
        print "***"
        print "*** And place 'pexports.exe' in your PATH."
        print "***"
        sys.exit(1)
    else:
        return name

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

def unlink(filename):
    """An unlink() wrapper to work around win32 problems
    in some Python versions."""	
    try:
        os.unlink(filename)
    except:
        os.remove(filename)

if os.name != 'nt':
    print "*** ERROR, not running under win32."
    print "*** Make sure you're running with a win32 native Python,"
    print "*** not a cygwin version."
    sys.exit(1)
    
#print find_pexports_or_bail()
#print find_dlltool_or_bail()

if len(sys.argv) < 3:
    print "Usage: dll_to_mingw.py DLL_NAME OUT_NAME"
    print "Where:"
    print "    DLL_NAME = .DLL file to convert"
    print "    OUT_NAME = Name for lib to create (typically libNNN.a)"
    sys.exit(1)

make_mingw_lib_from_dll(sys.argv[2],sys.argv[1])
