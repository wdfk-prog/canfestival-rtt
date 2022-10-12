
# diff two directories, using TreeOps
#
# Should give the same output as "diff -r", although the
# ordering of files might be different.
#
# frankm@hiwaay.net

# make sure I can import disthelper
import sys
sys.path.insert(0,'.')
import grab_disthelper
    
# now the real code begins ...

from disthelper.treeops.treeops import *
import os, re
from stat import *
from difflib import unified_diff
from disthelper.misc import mtime
import time

class TreeDiffer(TreeOps):
    """Diff two trees using TreeOps"""
    
    def __init__(self):
        TreeOps.__init__(self)
        
    def run(self, argv):
        # parse standard tree options (-r, -R, -x, etc.)
        p = TreeOptParser('difftree.py','Diff two directories.')		
        opts,args = p.parse_argv(argv)

        if len(args) != 2 or \
           not os.path.isdir(args[0]) or \
           not os.path.isdir(args[1]):
            print "** Must give two directories to diff."
            p.show_usage()
            sys.exit(1)

        # remember which files/dirs I couldn't access
        self.nofile = []
        self.nodir = []

        # I'm going to chdir() to the first dir - this way
        # I know that all paths I get will be of the form './ ...'
        # which I can just tack on to the second directory root.
        # However, I need to remember the relative paths, for
        # display purposes
        self.first_dir_rel = args[0]
        self.second_dir_rel = args[1]

        # remember the absolute path of the second dir
        self.second_dir_abs = os.path.abspath(args[1])

        # save current dir so I can restore at the end
        self.start_dir = os.getcwd()
        os.chdir(self.first_dir_rel)

        # now args will just be '.'

        # walk the tree with globbing, etc.
        self.runtree(opts,['.'])

        # tell user which files/dirs I couldn't access
        if len(self.nofile):
            print "I could not access these files:"
            for f in self.nofile:
                print "	 %s" % f

        if len(self.nodir):
            print "I could not access these directories:"
            for d in self.nodir:
                print "	 %s" % d
    
    # - internal API - called as the tree is walked -
    def process_one_file(self,fullname,opts):
        # name for open()
        name_open1 = fullname
        # name for display
        name_disp1 = os.path.join(self.first_dir_rel,fullname[2:])
        # name for open()
        name_open2 = os.path.join(self.second_dir_abs,fullname[2:])
        # name for display
        name_disp2 = os.path.join(self.second_dir_rel,fullname[2:])

        #print "diff %s, %s," % (name_disp1,name_disp2)
        
        # how to open a file in binary mode?
        if os.name == 'posix':
            mode = 'r'
        else:
            mode = 'rb'

        if not os.path.isfile(name_open1) or \
           not os.path.isfile(name_open2):
              return # missing file - will be caught in process_one_dir()
          
        buf_1 = open(name_open1,mode).read()
        buf_2 = open(name_open2,mode).read()

        binary_regex = r'[^\x09\x0a\x0d\x20-\x7f]'
        # is either file binary?
        if re.search(binary_regex,buf_1) or \
           re.search(binary_regex,buf_2):
            # yes, just display if it differs
            if buf_1 != buf_2:
                print "Files %s and %s differ." % \
                      (name_disp1,name_disp2)

        else:
            lines_1 = buf_1.splitlines()
            if not len(lines_1):
                lines_1 = ['']
            lines_2 = buf_2.splitlines()
            if not len(lines_2):
                lines_2 = ['']
                
            diffs = list(unified_diff(lines_1,lines_2,
                                      name_disp1,name_disp2,
                                      time.ctime(mtime(name_open1)),
                                      time.ctime(mtime(name_open2))))
            if len(diffs):
                for line in diffs:
                    while line[-1] in '\r\n':
                        line = line[:-1]
                        
                    print line
        
    def process_one_dir(self,fullname):
        # at each dir, compare a list of names to see if
        # any are unique to either
        
        names1 = os.listdir(fullname)
        names2 = os.listdir(os.path.join(self.second_dir_abs,fullname[2:]))

        for name in names1:
            if name not in names2:
                print "Only in %s: %s" % (fullname,name)

        for name in names2:
            if name not in names1:
                print "Only in %s: %s" % (os.path.join(self.second_dir_abs,
                                                       fullname[2:]), name)
                
    def dir_noaccess(self,fullname):
        self.nodir.append(fullname)

    def on_end_processing(self):
        # restore initial directory
        os.chdir(self.start_dir)
        
t = TreeDiffer()
t.run(sys.argv)




