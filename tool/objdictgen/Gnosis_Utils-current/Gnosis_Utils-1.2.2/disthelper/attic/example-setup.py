#
# Obsolete
#
#
# frankm@hiwaay.net
#

from distutils.core import setup, Extension
import os, sys, re
from stat import *
from glob import glob
import distutils.ccompiler
from disthelper.misc import *

PYVER = "%d.%d" % (sys.version_info[0],sys.version_info[1])

def find_swig():
    # first check for SWIG tagged with the Python version#
    # (I'm probably the only one who sets their system up like this :-)
    swig = find_exe_in_path('swig%s' % PYVER)
    if swig is not None:
        return swig

    # else, find regular swig in path
    return find_exe_in_path('swig')

SWIG_PROG = find_swig()

swig_c_ext = "c"
swig_cpp_ext = "cxx"

distutils.ccompiler.compiler_class['cyg4win32'] = \
                                                ('cyg4win32',
                                                 'Cygwin4Win32Compiler',
                                                 'Customized "cygwin -mno-cygwin"')

def run(cmd,ignore_err=0):
    print "Command: ", cmd, ", cwd: ",os.getcwd()
    if os.system(cmd) != 0 and not ignore_err:
        print "ERROR"
        sys.exit(1)
        
def mtime(file):
    return os.stat(file)[ST_MTIME]

def fix_wrapper(file,modname):

    if os.name == 'posix': # fix only required on win32
        return

    print "Fixing %s ..." % file

    fout = open(file,'at')
    # for some reason, SWIG is defining "init_modname", but python
    # is looking for "initmodname" ... weird. so, add trampoline
    # entry point
    fout.write('#ifdef __cplusplus\n')
    fout.write('extern "C"\n')
    fout.write('#endif\n')
    fout.write("SWIGEXPORT(void)init%s(void){SWIG_init();}\n"%modname)

def gen_swig(basename,swig_prog,wrap_ext,swig_opts):
    
    hfile = '%s.h' % basename
    ifile = '%s.i' % basename
    pyfile = '%s.py' % basename
    wrapfile = '%s_wrap.%s' % (basename, wrap_ext)
    
    # if mod .h/.i newer than .py/_wrap, regenerate
    if not os.path.isfile(pyfile) or \
           not os.path.isfile(wrapfile) or \
           mtime(hfile) > mtime(pyfile) or \
           mtime(ifile) > mtime(pyfile) or \
           mtime(hfile) > mtime(wrapfile) or \
           mtime(ifile) > mtime(wrapfile):

        print "Creating %s & %s" % (pyfile,wrapfile)
        run('%s %s %s' % (swig_prog,swig_opts,ifile))
        fix_wrapper(wrapfile,basename)
            
def gen_c_swigs(modlist):
    
    for mod in modlist:
        gen_swig(mod, SWIG_PROG, swig_c_ext, '-python')

def gen_cpp_swigs(modlist,extra_swig_args=''):
    
    for mod in modlist:
        gen_swig(mod, SWIG_PROG, swig_cpp_ext, '-c++ -shadow -python')

class SWIG_Extension(Extension):

    def __init__(self,name,sources,libs=[]):
        Extension.__init__(self,name=name,sources=sources,
                           libraries=libs)
        
class C_SWIG(SWIG_Extension):
    """A C extension module using SWIG.
    Expects three files in dir:
        name.c = Module source
        name.h = Module header
        name.i = SWIG interface for module.

    extra_sources is a list of filenames to include in
    the compilation."""
    def __init__(self,name,extra_sources=[]):
        SWIG_Extension.__init__(self,name=name,
                                sources=['%s.c' % name,
                                         '%s_wrap.%s' % \
                                         (name,swig_c_ext)] + extra_sources)

class CPP_SWIG(SWIG_Extension):
    """A C++ extension module using SWIG.
    Expects three files in dir:
        name.cpp = Module source
        name.h   = Module header
        name.i   = SWIG interface for module.

    extra_sources is a list of filenames to include in
    the compilation."""
    def __init__(self,name,extra_sources=[]):

        libs = []

        if os.name == 'posix':
            libs.append('stdc++')
            
        SWIG_Extension.__init__(self,name=name,
                                sources=['%s.cpp' % name,
                                         '%s_wrap.%s' % \
                                         (name,swig_cpp_ext)] + extra_sources,
                                libs=libs)

C_MODS = ['test_c']
CPP_MODS = ['test_cpp']

ext_list = []

ext_list.append(C_SWIG('test_c'))
ext_list.append(CPP_SWIG('test_cpp'))

def do_setup():
    #try:
        setup ( name		= 'test',
                version		= '0.1',
                description = "testing extension modules",
                author		= "Frank McIngvale",
                author_email= "frankm@hiwaay.net",
                url			= 'localhost',
                ext_modules = ext_list,
                license		= "GPL",
              )
    #	return 1
    #except:
    #	return 0

def copy_libs_to_cwd(modlist):

    # g_patt is where setup placed the built extensions
    if os.name == 'posix':
        g_patt = 'build/lib.linux*'
    else:
        g_patt = 'build/lib.win32*'

    g = glob(g_patt)
    if len(g) != 1:
        print "Can't find libdir!"
        sys.exit(1)

    for mod in modlist:
        # under Linux/posix, rename mod.so to _mod.so.
        # under win32, rename mod.pyd to _mod.pyd		
        if os.name == 'posix':
            cmd = "cp %s/%s.so _%s.so" %(g[0],mod,mod)
        else:
            cmd = "copy %s\\%s.pyd _%s.pyd" %(g[0],mod,mod)		
        print cmd
        run(cmd)

if 'build' in sys.argv:

    gen_c_swigs(C_MODS)
    gen_cpp_swigs(CPP_MODS)
    
    #do_setup()
    
    #copy_libs_to_cwd(C_MODS+CPP_MODS)
    
elif 'info' in sys.argv:
    print "Installed programs:"

    print "  SWIG = %s" % SWIG_PROG
    print distutils.ccompiler.new_compiler(compiler='mingw32')
    
    
elif 'clean' in sys.argv:
    os.system('rm -rf build *~ *.so *.pyd *_wrap.%s *_wrap.%s' % (swig_c_ext,swig_cpp_ext))
    
