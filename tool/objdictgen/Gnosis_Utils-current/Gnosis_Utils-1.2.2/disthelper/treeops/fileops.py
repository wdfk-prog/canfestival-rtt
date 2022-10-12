
#
# A FileTransform takes an input filename, does some
# processing on it, and writes the processed file
# to the original filename.
#
# frankm@hiwaay.net
#

from disthelper.misc import make_tempdir, unlink
from disthelper.treeops.treeops import *
import os
from shutil import copy2

class FileTransform:
    def __init__(self):
        pass

    def run(self, filename):
        """Run processing on a given file, with results
        being written back to same filename."""

        # make a tempdir so I can use a file-like object
        # instead of an OS-level handle (like with mkstemp)		
        tdir = make_tempdir()
        tname = os.path.join(tdir,'process.out')

        #print "TEMPDIR = ",tdir
        #print "TEMPFILE = ",tname
        
        f_out = open(tname,'wb')
        f_in = open(filename,'rb')

        # process in->out
        self.process(f_out, f_in)

        del f_out
        del f_in

        # copy tempfile -> filename

        #print "COPY %s -> %s" % (tname,filename)
        
        # I think this is secure ... since caller owns filename
        # there isn't a race, right? (unlike writing into a tempdir
        # which could have malicious symlinks in it)
        copy2( tname, filename )

        #print "RMDIR %s" % tname
        
        # clean up tempdir
        unlink(tname)
        os.rmdir(tdir)
            
    # -*- internal API -*-
    
    def process(self,fileobj_out,fileobj_in):
        """Subclasses override this to do their processing.
        Inputs:
           fileobj_out - a file-like object to write to.
           fileobj_in - a file-like object to read from."""
        pass

class TreeOpFromFileTransform(TreeOps):
    """Turn a FileTransform into a TreeOps"""
    def __init__(self, filetransform):
        "filetransform is the transform to use"
        TreeOps.__init__(self)
        self.filetransform = filetransform

    # - internal API - called by TreeOps -
    def process_one_file(self,fullname,opts):
        "Called for each matched file."
        if opts.verbose:
            print fullname
            
        #print "RUN FILE XFORM %s ON %s" % (fullname,str(self.filetransform))
        self.filetransform.run(fullname)
        
    def process_one_dir(self,fullname):
        "Called for each directory along the way."
        pass

    def dir_noaccess(self,fullname):
        """Called when access is denied to a directory
        (strictly informational, there is no provision to
        retry the operation)."""
        pass
    
class FileTransformFromLineOp(FileTransform):
    """Turn a LineOp into a FileTransform"""
    
    def __init__(self, lineop):
        self.set_lineop( lineop )
        
    # - internal API -

    def set_lineop(self, op):
        self.lineop = op
        
    def process(self,fileobj_out,fileobj_in):
        # do in awkward-but-backward-compatible way ..
        line = fileobj_in.readline()

        while len(line) != 0:
            #print "LINE ",repr(line)
            
            buf = line
            buf = self.lineop(line)

            fileobj_out.write(buf)

            line = fileobj_in.readline()
            
        
            
        
