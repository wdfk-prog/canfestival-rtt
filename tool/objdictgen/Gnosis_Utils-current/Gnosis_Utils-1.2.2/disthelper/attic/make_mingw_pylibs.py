
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

from disthelper.find_python import *
import disthelper.find_python as find_python
from disthelper.misc import *
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

def make_mingw_lib_from_dll( destname, dllname ):
    """Take a win32 DLL and create a lib*.a file suitable
    for linking with mingw.

    dllname is the full pathname of the .DLL to convert.
    destname is the name for the converted mingw library.

    This is an automation of the procedure found at:
        http://sebsauvage.net/python/mingw.html"""
    
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
    os.system(cmd)
    
    # create .a
    cmd = "%s --dllname %s --def temp.def --output-lib %s" % \
          (dlltool, os.path.basename(dllname), destname)
    os.system(cmd)

    # remove temporary files & tempdir
    unlink('temp.def')
    unlink(os.path.basename(dllname))
    os.chdir(savedir)
    os.rmdir(tempdir)

def get_winroot_from_registry():
    """Get the Windows directory, e.g. c:\WINDOWS, from the
    registry, or None if not found or error."""

    if HAVE_WIN32_REGISTRY == 0:
        return None

    try:
        # SystemRoot is found here on win9x machines		
        topkey = OpenKey(HKEY_LOCAL_MACHINE,"SOFTWARE\\Microsoft\\Windows\\CurrentVersion")

        path,typ = QueryValueEx(topkey,'SystemRoot')
        return path
    except:
        pass

    try:
        # On NT/2k/etc., SystemRoot is under 'Windows NT' instead
        topkey = OpenKey(HKEY_LOCAL_MACHINE,"SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion")
        
        path,typ = QueryValueEx(topkey,'SystemRoot')
        return path
    except:
        pass
    
    # maybe it doesn't exist under some versions of win32
    return None

def get_dll_search_path():
    """Generate a list of all the places where the pythonNN.dll
    files might be stored. This roughly duplicates the standard
    win32 rules. There may be a lot of duplicates, but this routine
    attempts to be comprehensive."""

    # searchlist contains full paths
    searchlist = []

    # check with win32api, if present
    if HAVE_WIN32_API:
        # p will be e.g. c:\windows\system
        p = win32api.GetSystemDirectory()
        searchlist.append(p)

        # on NT, p will contain SYSTEM32, so add SYSTEM as well
        p = os.path.join(os.path.dirname(p), 'SYSTEM')
        searchlist.append(p)

        # add, e.g. c:\windows
        searchlist.append( win32api.GetWindowsDirectory() )
        
    # generate some root paths, then add SYSTEM & SYSTEM32 to each
    rootlist = []

    # check the registry
    path = get_winroot_from_registry()
    if path is not None:
        rootlist.append( path )
        
    # add PATH directories
    rootlist.extend( string.split( os.environ['PATH'], os.pathsep ))

    # now, for each, add SYSTEM & SYSTEM32, in the hope
    # that one of the paths is, e.g. c:\windows
    for path in rootlist:
        searchlist.append( path )
        searchlist.append( os.path.join(path,'SYSTEM') )
        searchlist.append( os.path.join(path,'SYSTEM32') )			

    # add the .exe directory
    searchlist.append( os.path.dirname( os.path.abspath( sys.executable )))

    # add the cwd
    searchlist.append( os.getcwd() )
    
    return searchlist

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

def find_python_win32_dll( ver ):
    """Find the win32 .dll matching a given version of Python.
    ver is a 3-element tuple like (2,1,3)

    Returns full path to .dll, or None if not found."""

    name = 'python%d%d.dll' % (ver[0],ver[1])

    for path in get_dll_search_path():
        full = os.path.join( path,name )
        if os.path.isfile(full):
            return full

    return None

if len(sys.argv) < 2:
    print "\nUsage: make_mingw_pylibs.py [--all] [version]"
    print ""
    print "     Creates MinGW libraries (.a) from the Python .DLLs."
    print "     This allows you to use MinGW (or cygwin, with 'gcc -mno-cygwin')"
    print "     to create Python extension modules that will run on any win32"
    print "     system (i.e. cygwin not needed)."
    print ""
    print "     The generated .a files are automatically installed into the"
    print "     correct directory (i.e. PYTHONROOT\\libs)"
    print ""
    print "You can pass either:"
    print ""
    print "     --all: Find and convert all Python DLLs on the host system."
    print ""
    print "     version: convert DLLs for a specific version only (e.g. 2.3)"
    print ""
    
    sys.exit(1)

if sys.argv[1] == '--all':
    thelist = get_python_verlist()
else:
    try:
        pyver = find_python.parse_version_string(sys.argv[1])
    except:
        print ""
        print "** Sorry, can't parse version string '%s'." % sys.argv[1]
        print "** You must pass a string like 2, 2.1, 2.1.3."
        print ""
        sys.exit(1)

    # the DLLs are only specified to minor version (i.e. 2.1)
    # so only need to match that much of the string
    exe = find_py_minor(pyver)
    if exe is None:
        print ""
        print "** Sorry, no such Python version %s" % sys.argv[1]
        print ""
        sys.exit(1)
            
    thelist = [(exe,pyver[:2]+[0])]

had_failure = 0

for exe,ver in thelist:
    dll = find_python_win32_dll( ver )
    print "%s, version = %d.%d.%d, dll = %s" % \
          (exe,ver[0],ver[1],ver[2],dll)
    
    # the .a file will be placed in $PYTHONDIR\libs
    libdir = os.path.join( os.path.dirname(exe), 'libs' )

    if os.access(libdir, os.W_OK) == 0:
        print "**** WARNING ****"
        print "**** Can't write to %s, skipping!" % libdir
        had_failure = 1
    else:
        libname = os.path.splitext(os.path.basename(dll))[0]
        libname = os.path.join( libdir,"lib%s.a" % libname)

        make_mingw_lib_from_dll(libname,dll)

if had_failure:
    print "*** WARNING ***"
    print "*** There were one or more errors in the conversion process."
    print "*** Recommend you correct the errors and try again."
else:
    print "**"
    print "** Conversion complete"
    print "**"
    print "** Now, to build native win32 extension modules, all you"
    print "** have to do is call your setup.py like this:"
    print "**"
    print "**       python setup.py -cmingw32"
    print "**"
    print "**   [This works for both MinGW & cygwin with 'gcc -mno-cygwin']"
    print "**"
    print "** NOTE: If you let distutils run SWIG for you, and you"
    print "**       are using Python <= 2.2, read the notes at:"
    print "**"
    print "**          http://sebsauvage.net/python/mingw.html"
    print "**"
    print "**       On minor changes you'll need to make to distutils."
    print "**"

    
