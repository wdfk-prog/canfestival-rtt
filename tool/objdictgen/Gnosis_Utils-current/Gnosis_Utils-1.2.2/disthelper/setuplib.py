#
# I found myself writing the same little functions over
# and over in setup.py files, so I've collected them here.
#
# (See also disthelper.misc for more generally useful functions.)
#
# frankm@hiwaay.net
#

from distutils.core import setup, Extension
import os, re
from disthelper.misc import *
from glob import glob
import shutil
from disthelper.treeops import TreeOps, TreeOptParser

# make 'import *' safe
__all__ = ['run','gen_all_swigs','clean_all','C_SWIG','CPP_SWIG',
           'zip_current_dir','increment_build_nr']

# file extensions for C/C++ files that SWIG generates
SWIG_c_ext = "c"
SWIG_cpp_ext = "cxx"

def run(cmd,ignore_err=0):
    print "Command: ", cmd, ", cwd: ",os.getcwd()
    if os.system(cmd) != 0 and not ignore_err:
        print "ERROR"
        sys.exit(1)

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

def gen_swig(swig_obj,swig_prog,wrap_ext,swig_opts):

    basename = swig_obj.swig_basename
    
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
            
def gen_c_swig(swig_exe, mod):
    gen_swig(mod, swig_exe, SWIG_c_ext,
             '-python' + ' '.join(mod.swig_extra_args))

def gen_cpp_swig(swig_exe, mod):
    gen_swig(mod, swig_exe, SWIG_cpp_ext,
             '-c++ -shadow -python ' + ' '.join(mod.swig_extra_args))

def gen_all_swigs(swig_exe, modlist):

    for mod in modlist:
        if isinstance(mod, C_SWIG):
            gen_c_swig(swig_exe, mod)
        elif isinstance(mod, CPP_SWIG):
            gen_cpp_swig(swig_exe, mod)
        else:
            print "********* UNKNOWN SWIG TYPE *************"
            raise Exception()

def clean_all_swigs(modlist):

    for mod in modlist:
        for name in ['%s.py' % mod.swig_basename,
                     '%s_wrap.%s' % (mod.swig_basename,SWIG_c_ext),
                     '%s_wrap.%s' % (mod.swig_basename,SWIG_cpp_ext)]:
            
            if os.path.isfile(name):
                print 'del %s' % name
                unlink(name)

def clean_all(ext_list, extra_patt=[]):
    clean_all_swigs(ext_list)
    shutil.rmtree('build',1)

    rmfiles = []
    for patt in ['*.pyc','*~','*.so','*.pyd',
                 '*.o','core','core.*'] + extra_patt:
        rmfiles.append( glob(patt) )
                 
    for name in rmfiles:
        unlink(name)
    
class SWIG_Extension(Extension):

    def __init__(self,name,sources,libs=[],incdirs=[],libdirs=[],
                 define_macros=[],swig_args=[]):
        Extension.__init__(self,name=name,sources=sources,
                           libraries=libs,include_dirs=incdirs,
                           library_dirs=libdirs,define_macros=define_macros)

        self.swig_basename = name
        self.swig_extra_args = swig_args
        
class C_SWIG(SWIG_Extension):
    """A C extension module using SWIG.
    Expects three files in dir:
        name.c = Module source
        name.h = Module header
        name.i = SWIG interface for module.

    extra_sources is a list of filenames to include in
    the compilation."""
    def __init__(self,name,extra_sources=[],define_macros=[],
                 swig_args=[]):
        SWIG_Extension.__init__(self,name=name,
                                sources=['%s.c' % name,
                                         '%s_wrap.%s' % \
                                         (name,SWIG_c_ext)] + extra_sources,
                                define_macros=define_macros,
                                swig_args=swig_args)

class CPP_SWIG(SWIG_Extension):
    """A C++ extension module using SWIG.
    Expects three files in dir:
        name.cpp = Module source
        name.h	 = Module header
        name.i	 = SWIG interface for module.

    extra_sources is a list of filenames to include in
    the compilation."""
    def __init__(self,name,extra_sources=[],extra_libs=[],define_macros=[],
                 incdirs=[],libdirs=[],swig_args=[]):

        libs = extra_libs

        if os.name == 'posix':
            libs.append('stdc++')
            
        SWIG_Extension.__init__(self,name=name,
                                sources=['%s.cpp' % name,
                                         '%s_wrap.%s' % \
                                         (name,SWIG_cpp_ext)] + extra_sources,
                                incdirs=incdirs,
                                libs=libs,
                                libdirs=libdirs,
                                define_macros=define_macros,
                                swig_args=swig_args)

# I don't use gnosis.pyconfig here because I want disthelper
# to be able to be used completely independently
try:
    import zipfile, zlib
    HAVE_ZIPSTUFF = 1
except: HAVE_ZIPSTUFF = 0

try:
    import tarfile
    HAVE_TAR = 1
except: HAVE_TAR = 0

def _error_no_zip():
    print "** Sorry, this version of Python cannot create .zip files."

def _error_no_tar():
    print "** Sorry, this version of Python cannot create .tar files."
    
class zip_tree(TreeOps):
    "Worker class for 'zip -r' functionality."
    def __init__(self, outname, add_prefix=None, exclude_re_list=[],
                 overwrite=1):
        TreeOps.__init__(self)
        if overwrite:
            if os.path.isfile(outname):
                unlink(outname)
            mode = 'w'
        else:
            mode = 'a'

        self.zipname = outname
        self.zf = zipfile.ZipFile(outname, mode, zlib.DEFLATED)
        self.add_prefix = add_prefix
        self.exclude_re_list = exclude_re_list
        
    def process_one_file(self,name,opts):
        #print "Adding",name

        # don't add zipfile to self
        if samepath(name, self.zipname):
            return

        for r in self.exclude_re_list:
            if r.match(name):
                #print "*** EXCLUDE ",name
                return # matched exclusion list
            
        if self.add_prefix is None:
            self.zf.write(name)
        else:
            self.zf.write(name,os.path.normpath(os.path.join(self.add_prefix,name)))
            
    def process_one_dir(self,name):
        #print "DIR ",name
        pass

    def on_end_processing(self):
        self.zf.close()
        
class tar_tree(TreeOps):
    "Worker class for tar functionality."
    def __init__(self, outname, mode, add_prefix=None, exclude_re_list=[],
                 overwrite=1):
        TreeOps.__init__(self)		
        if overwrite:
            if os.path.isfile(outname):
                unlink(outname)

        self.tarname = outname
        self.tarfile = tarfile.open(outname, mode)
        self.add_prefix = add_prefix
        self.exclude_re_list = exclude_re_list
        
    def process_one_file(self,name,opts):
        #print "Adding",name

        # don't add tarfile to self
        if samepath(name, self.tarname):
            return

        for r in self.exclude_re_list:
            if r.match(name):
                #print "** EXCLUDE",name
                return # matched exclusion list
            
        if self.add_prefix is None:
            self.tarfile.add(name)
        else:
            self.tarfile.add(name, os.path.normpath(
                os.path.join(self.add_prefix,name)))
                    
    def process_one_dir(self,name):
        pass

    def on_end_processing(self):
        self.tarfile.close()
        
def _zip_current_dir( zfilename, add_prefix=None, exclude_re_list=[] ):
    """
    Zip up the current directory, just as if you had
    done 'zip -r zfilename *' (except it will grab all
    .dotfiles as well).

    Does NOT require a 'zip' program to be installed.
    
    zfilename = Output filename.
    add_prefix = Prefix to prepend to all filenames added.
                 If None, names will not be modified.
                 (This is useful when you want a toplevel directory
                 to appear in the zipfile, without having to
                 chdir('..') and zipping the directory from there.
                 add_prefix will be os.path.joined with each filename.)
    exclude_re_list = Files will be excluded if they match one of these
                      (compiled) regexes.
    """
    
    top = TreeOptParser('dummy','dummy')
    # call as if used passed args 'prog -R"*,.*" .'
    # [must glob with '*' and '.*' at each level]
    opts,args = top.parse_argv(['dummy','-R*,.*','.'])	

    zt = zip_tree(zfilename,add_prefix,exclude_re_list)
    zt.runtree(opts,args)

if HAVE_ZIPSTUFF:
    zip_current_dir = _zip_current_dir
else:
    zip_current_dir = _error_no_zip
    
# convenience
def tar_bz2_current_dir( bz2filename, add_prefix=None,
                         exclude_re_list=[] ):
    tar_current_dir( bz2filename, 'w:bz2', add_prefix, exclude_re_list )

def tar_gz_current_dir( gzfilename, add_prefix=None,
                        exclude_re_list=[] ):
    tar_current_dir( gzfilename, 'w:gz', add_prefix, exclude_re_list )	

def _tar_current_dir( tarfilename, mode, add_prefix=None,
                      exclude_re_list=[] ):
    """
    Tar up the current directory, just as if you had
    done 'tar . - |(bzip2|gzip) -9 > zfilename' (except it will grab all
    .dotfiles as well).

    Does NOT require tar, gzip, or bzip2 program to be installed.
    
    tarfilename = Output filename.
    mode = 'w:gz' or 'w:bz2', to specify gzip/bzip2
    add_prefix = Prefix to prepend to all filenames added.
                 If None, names will not be modified.
                 (This is useful when you want a toplevel directory
                 to appear in the tarfile, without having to
                 chdir('..') and tarring the directory from there.
                 add_prefix will be os.path.joined with each filename.)
    exclude_re_list = Files will be excluded if they match one of these
                      (compiled) regexes.
    """
    
    top = TreeOptParser('dummy','dummy')
    # call as if used passed args 'prog -R"*,.*" .'
    # [must glob with '*' and '.*' at each level]
    opts,args = top.parse_argv(['dummy','-R*,.*','.'])	

    t = tar_tree(tarfilename,mode,add_prefix,exclude_re_list)
    t.runtree(opts,args)

if HAVE_TAR:
    tar_current_dir = _tar_current_dir
else:
    tar_current_dir = _error_no_tar
    
def increment_build_nr():
    #tempdir = mkdtemp()
    tempdir = make_tempdir()
    tempfile = os.path.join(tempdir,'v.out')
    
    fin = open('version.py','r')
    fout = open(tempfile,'w')

    while 1:
        line = fin.readline()
        if len(line) == 0:
            break

        m = re.match(r'BUILD_NR\s*=\s*([0-9]+)',line)
        if m:
            fout.write('BUILD_NR = %d\n' % (int(m.group(1)) + 1))
        else:
            # keep version.py in unix format
            while len(line) and line[-1] in '\r\n':
                line = line[:-1]
                
            fout.write('%s\n' % line)

    del fin
    del fout

    unlink('version.py')
    shutil.copy(tempfile,'version.py')

    unlink(tempfile)
    os.rmdir(tempdir)
    
