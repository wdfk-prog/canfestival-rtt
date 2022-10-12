#
# Obsolete
#
#
# frankm@hiwaay.net
#

from distutils.unixccompiler import UnixCCompiler

class FrankCompiler(UnixCCompiler):
    compiler_type = 'frankcc'

    def preprocess(self, source,
                   output_file=None, macros=None, include_dirs=None,
                   extra_preargs=None, extra_postargs=None):
        print "PREPROCESS"
        UnixCCompiler.preprocess(self,source,output_file,macros,
                                 include_dirs,extra_preargs,exta_postargs)

    def _compile(self,obj,src,ext,cc_args,extra_postargs,pp_opts):
        print "_compile"
        UnixCCompiler._compiler(self,obj,src,ext,cc_args,extra_postargs,pp_opts)

    def link(self, target_desc, objects,
             output_filename, output_dir=None, libraries=None,
             library_dirs=None, runtime_library_dirs=None,
             export_symbols=None, debug=0, extra_preargs=None,
             extra_postargs=None, build_temp=None, target_lang=None):

        print "LINK"
        UnixCCompiler.link(self, target_desc, objects,
                           output_filename, output_dir, libraries,
                           library_dirs, runtime_library_dirs,
                           export_symbols, debug, extra_preargs,
                           extra_postargs, build_temp, target_lang)

    def library_dir_option(self,dir):
        return UnixCCompiler.library_dir_option(self,dir)


    def find_library_file(self,dirs,lib,debug=0):
        return UnixCCompiler.find_library_file(self,dirs,lib,debug)
    
