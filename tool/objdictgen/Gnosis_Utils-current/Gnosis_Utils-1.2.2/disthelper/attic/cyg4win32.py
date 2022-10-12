# this is cyg4win32.py from gnosis.disthelper
#
# Do NOT edit/delete the first line of this file!! (it is used
# as a sanity check to prevent accidentally overwriting files)

#
# This is a lightly modified version of the Mingw32CCompiler.
#
# It is meant for using cygwin's 'gcc -mno-cygwin' to compile
# Python extensions that will run under a win32-native (VC++
# compiled) version of Python [note: I believe that is SUPPOSED
# to work out of the box using -cmingw32, but it must be somewhat
# out of date, since I had to make the hacks here.]
#
# To use it, call your setup.py like this:
#
#      python setup.py build -ccyg4win32
#

# This file must be installed in distutils (alongside ccompiler.py)
# or distutils can't find it. ugh.

# There are some hacks here that seem out of place, however, since
# this class is meant for a specific compiler on a specific platform,
# having the hacks here prevents ugliness in setup.py

#
# frankm@hiwaay.net
#

from distutils.cygwinccompiler import Mingw32CCompiler
import re

class Cygwin4Win32Compiler(Mingw32CCompiler):

    compiler_type = 'cyg4win32'

    def __init__(self, verbose=0, dry_run=0, force=0 ):
        Mingw32CCompiler.__init__(self,verbose,dry_run,force)

    def check_for_gcc(self, where):
        # ensure gcc has been set for these attributes ...
        checklist = ['compiler_cxx','compiler_so','linker_exe',
                     'linker_so','compiler']

        for k in checklist:
            if getattr(self,k)[0] != 'gcc':
                print "**** WARNING in %s: Expecting gcc for '%s', got '%s'" % \
                      (where,k,getattr(self,k))
                        
    def _compile(self, obj, src, ext, cc_args, extra_postargs, pp_opts):
        # make sure gcc is being used
        self.check_for_gcc('_compile')
        
        Mingw32CCompiler._compile(self, obj, src, ext, cc_args, extra_postargs,
                                  pp_opts)

    def link (self, target_desc, objects, output_filename, output_dir=None,
              libraries=None, library_dirs=None, runtime_library_dirs=None,
              export_symbols=None, debug=0, extra_preargs=None,
              extra_postargs=None, build_temp=None, target_lang=None):

        # for some reason, this is being set to 'cc', which is wrong
        self.compiler_cxx = ['gcc','-mno-cygwin']

        # make sure gcc is being used
        self.check_for_gcc('_link')		
        
        if re.search(r'c\+\+',target_lang,re.I):
            # stdc++ isn't being included since platform != posix
            if 'stdc++' not in libraries:
                libraries.append('stdc++')
                
        Mingw32CCompiler.link(self,target_desc,objects,output_filename,
                              output_dir,libraries,library_dirs,
                              runtime_library_dirs,export_symbols,
                              debug,extra_preargs,extra_postargs,
                              build_temp,target_lang)
        
