
#
# TreeOps encompasses common code for working with
# directory trees - walking the tree, matching the
# filenames vs. a set of regexes, etc.
#
# To do something useful, derive a class from TreeOps and
# override the private API (process_one_file, process_one_dir, etc.)
#
# Note this module should be compatible to Python 1.5.2
#
# frankm@hiwaay.net
#

import os, sys, re
from disthelper.util.cmdline import *
from types import *
from glob import glob

# public API
__all__ = ['TreeOps','TreeOptParser']

class TreeOps:

    def __init__(self):
        self.__recurse_level = 0
        pass
    
    def runtree(self,opts,things):
        """Run using opts and a list of things to process
        (typically these are the (opts,args) returned from
        TreeOptParser.parse_argv)."""

        self.__recurse_level = self.__recurse_level + 1
        
        if type(things) not in [ListType,TupleType]:
            self.on_end_processing() # let caller cleanup before bailing
            raise "'things' must be a sequence"
        
        # this could be more compact, but have to maintain 1.5.2 compat

        descend = []
        
        for thing in things:

            matched = 0
            for r in opts.exclude_list:
                if r.match(thing):
                    matched = 1
                    break

            if matched:
                #print "EXCLUDE ",thing
                continue # skip name in exclusion list
                
            if os.path.islink(thing):
                continue
            
            if os.path.isfile(thing):
                # match on filename
                matched = 0
                for r in opts.regexlist:
                    if r.match(thing):
                        matched = 1
                        break

                if matched or len(opts.regexlist) == 0:
                    # this is still a relative path, i.e. relative
                    # to 'thing', so it is correct to pass it here
                    self.process_one_file(thing,opts)				
                
            elif os.path.isdir(thing):
                self.process_one_dir(thing)

                try:
                    names = os.listdir(thing)
                except:
                    self.dir_noaccess(thing)
                    continue

                for name in names:
                    full = os.path.join(thing,name)

                    # always descend dir, no matching
                    if os.path.isdir( full ) and opts.recursive:
                        # finish processing all the files in this
                        # directory before descending subdirectories.
                        # this keeps things nice and simple for subclasses,
                        # knowing that we'll finish a directory completely
                        # before switching to another one
                        descend.append( full )
                        
                    elif os.path.isfile( full ):

                        # this is still a relative path, i.e. relative
                        # to 'thing', so it is correct to pass it here
                        # (to be checked in block above)
                        self.runtree(opts, [full])

        # process subdirectories found
        if len(descend):
            self.runtree( opts, descend )

        self.__recurse_level = self.__recurse_level - 1
        if self.__recurse_level == 0:
            self.on_end_processing()
            
    # - internal API - subclasses do their work here -
    # General note: Why relative paths here?
    # Think e.g. 'zip -r' -- you need to know the relative
    # path that the user intended, since you don't want to
    # store the entire path from / (or c:).
    def process_one_file(self, name, opts):
        """
        Called for each matched file (name is relative path).
        'opts' are the same as were passed to process().
        """
        pass

    def process_one_dir(self, name):
        "Called for each directory (name is relative path)."
        pass

    def dir_noaccess(self, name):
        """Called when access is denied to a directory (name is relative path).
        Strictly informational, there is no provision to
        retry the operation."""
        pass

    def on_end_processing(self):
        """Called after all processing has completed, so
        subclasses can do any necessary cleanup."""
        pass
    
class TreeOptParser(BasicOptParser):
    """A specialization of BasicOptParser, which adds options
    common to programs that need to recurse a tree, selecting
    files by certain options."""
    
    def __init__(self,name,info):
        """
        name: Program name (for help text)
        info: One-liner description (for help text)
        """
        BasicOptParser.__init__(self,name,info)
        
        # options common to all TreeOpts
        self.add_boolopt( 'h', 'help', 'help', 'Show this help screen' )
        self.add_boolopt( 'r', 'recursive', 'recursive',
                          'Recurse subdirectories')
        self.add_listopt( 'R', 'recursive-glob', 'globlist',
                         'Like -r, but match filenames to pattern(s).\n\t(Seperate multiple patterns with commas.)')
        self.add_listopt( 'x', 'extension', 'extlist',
                         'Give a list of file extensions to match.\n\t(Separate multiple extensions with commas.)')
        self.add_boolopt( 'i', 'ignore-case', 'nocase',
                          'Ignore case when matching filenames.'),
        self.add_boolopt( 'v', 'verbose', 'verbose',
                          'Be verbose while running')
        self.add_listopt( '', 'exclude', 'exclude',
                          'Regular expression list of names to exclude')

    def parse_argv(self, argv, glob_args=1):
        """Parse command line args (typically you'll pass sys.argv,
        though any list of strings will do). Note that if you pass
        a list of strings, the first one must be the program name.

        Returns (opts,args), just like BasicOptParser.process(),
        with these specializations:

           1. extlist & globlist are converted to regexes and
              stored in attr 'regexlist'.
           2. If -R given, -r is turned on as well.
           3. If glob_args == 1, args will be glob-expanded before
              returning.
           4. if --exclude given, turns patterns into regex list
              and stored in attr 'exclude_list'
           """
        
        opts,args = self.process(argv[1:])

        if opts.help:
            self.show_usage()
            sys.exit(0)
            
        regexlist = []
        
        # First, make a regex out of each extension match
        for ext in opts.extlist:
            regexlist.append( r'^.+\.%s$' % ext )

        # Now add any glob pattern the user specified with -R
        for glob_arg in opts.globlist:
            # turn the shell-style glob into a regex
            g = glob_arg.replace('.',r'\.').replace('*','.*')
            g = '^' + g + '$'
            regexlist.append(g)

        # compile them all
        if opts.nocase:
            regexlist = map(lambda x: re.compile(x,re.I), regexlist)
        else:
            regexlist = map(lambda x: re.compile(x), regexlist)
            
        # save into opts
        setattr(opts, 'regexlist', regexlist)

        # -R implies -r (simplifies checks later)
        if len(opts.globlist):
            setattr(opts, 'recursive', 1)

        # turn --exclude into regexes
        exclude_list = []
        
        for excl_arg in opts.exclude:
            # turn the shell-style glob into a regex
            g = excl_arg.replace('.',r'\.').replace('*','.*')
            g = '^' + g + '$'			
            exclude_list.append(g)

        # compile them all
        if opts.nocase:
            exclude_list = map(lambda x: re.compile(x,re.I), exclude_list)
        else:
            exclude_list = map(lambda x: re.compile(x), exclude_list)

        # save into opts
        setattr(opts, 'exclude_list', exclude_list)
        
        # hm, initially I was appending '.' if args was empty,
        # but that needs to be a program-specific decision,
        # since it's not always desired

        if glob_args:
            newargs = []
            for arg in args:
                newargs.extend( glob(arg) )

            args = newargs
            
        return opts,args
    



