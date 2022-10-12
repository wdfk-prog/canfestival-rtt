#
# BasicOptParser - minimal, portable, optparse substitute.
#
# Works with Python 1.5.2 and up.
#
# written by frankm@hiwaay.net
#

# public API
__all__ = ['BasicOptParser']

# Unfortunately, gnu_getopt is only available on Python 2.2+,
# so have to stick with basic getopt (otherwise, you'd confuse
# users since option parsing would change depending on which
# Python they used to run their scripts). I guess I could do
# the parsing here too, but getopt is 200+ lines of debugged
# code, so use it.
from getopt import getopt
import string

OPT_STRING = 0
OPT_BOOL = 1
OPT_LIST = 2
OPT_INT = 3

class BasicOptParser:
    """Minimal optparse substitute, since optparse requires Python 2.3+.
    The API is similar to optparse, but fewer types/options,
    and subject to the limitations of getopt (i.e. no multiple instances
    of options). Works with Python 1.5.2 and up.

    Like optparse, options are set as attributes of an object returned
    from process() [see process() below]. The 'attr' argument of
    add_stropt/add_boolopt specifies the named attribute (analogous
    to the 'dest' argument in optparse.add_option)."""
    
    def __init__(self,prog_name,prog_info):
        """		
        prog_name & prog_info are used for the help text.
        prog_name is the actual program name.
        prog_info is a one-liner description.
        """
        self.opts = [] # keep a linear list, for help generation
        self.optmap = {} # keep a mapping as well for fast lookup
        self.prog_name = prog_name
        self.prog_info = prog_info
        
    def add_stropt(self, shortopt, longopt, attr, help=''):
        """Add an option taking a string argument, i.e. '-R value'.
        Defaults to None if option not used."""
        self.add_typeopt( OPT_STRING, shortopt, longopt, attr, help )
        
    def add_listopt(self, shortopt, longopt, attr, help=''):		
        """Add an option taking a list of comma-separated strings,
        i.e. '-R foo,bar,baz'. Strings are stored as a list.
        Defaults to [] if option not used."""
        self.add_typeopt( OPT_LIST, shortopt, longopt, attr, help )

    def add_boolopt(self, shortopt, longopt, attr, help=''):
        """Add a boolean (on/off) option, i.e. '-r'.
        Stores 1 if option used, 0 if not."""
        self.add_typeopt( OPT_BOOL, shortopt, longopt, attr, help )

    def add_intopt(self, shortopt, longopt, attr, help=''):
        """Add am integer option, i.e. '-r NN'.
        Stores None if option not used."""
        self.add_typeopt( OPT_INT, shortopt, longopt, attr, help )		

    def process(self, argv):
        """Returns (opts,args) where attributes are set in 'opts' for
        each option (like with optparse), and args is the list of
        non-option strings."""

        # convert opts to getopt args & call getopt
        sshort, llong = self.make_getopt_args()
        opts, args = getopt(argv, sshort, llong)

        # process getopt results
        ropt = BasicOptDataVal(self.opts)

        for opt, val in opts:
            tup = self.optmap[opt]

            if tup[0] == OPT_STRING:
                setattr(ropt,tup[3],val)
            elif tup[0] == OPT_LIST:
                setattr(ropt,tup[3],string.split(val,','))
            elif tup[0] == OPT_BOOL:
                setattr(ropt,tup[3],1)
            elif tup[0] == OPT_INT:
                setattr(ropt,tup[3],int(val))				
            else:
                raise "* internal error *"
            
        return ropt,args

    def show_usage(self):
        """Print usage/help information"""

        print "%s - %s\n\nUsage: %s [options] arg, ...\n" % \
              (self.prog_name,self.prog_info,self.prog_name)

        for otype, s, l, attr, help in self.opts:
            
            if otype in [OPT_STRING,OPT_INT]:
                arg = ' arg'
            elif otype == OPT_LIST:
                arg = ' arg,...'
            else:
                arg = ''

            hs = ''
            
            if len(s):
                hs = hs + '-%s%s, ' % (s,arg)
            if len(l):
                hs = hs + '--%s%s' % (l,arg)

            if len(help):
                hs = hs + ':\n\t' + help

            print hs

    # -*- internal API below -*-
    
    def add_typeopt(self, otype, shortopt, longopt, attr, help=''):
        if shortopt != '' and len(shortopt) != 1:
            raise "shortopt must be a single char, or ''"
                
        tup = (otype, shortopt, longopt, attr, help)

        # add to linear list (for help)
        self.opts.append( tup )

        # add to map, prepending '-' or '--'
        # (getopt leaves the -/-- on the options it returns)		
        if len(shortopt):
            self.optmap['-'+shortopt] = tup
        if len(longopt):
            self.optmap['--'+longopt] = tup

    def make_getopt_args(self):

        sshort = '' # getopt() short-arg string
        llong = []  # getopt() long-arg list

        # what we add to arg, based on type
        shortadd = {OPT_STRING: ':', OPT_LIST: ':', OPT_BOOL: '',
                    OPT_INT: ':'}
        longadd = {OPT_STRING: '=', OPT_LIST: '=', OPT_BOOL: '',
                   OPT_INT: '='}
        
        for otype, shortopt, longopt, attr, help in self.opts:
            if len(shortopt):
                sshort = sshort + shortopt + shortadd[otype]
            if len(longopt):
                llong.append( longopt + longadd[otype] )
                
        return (sshort, llong)
    
# this is the datatype returned as the 'opts' object from
# BasicOptParser.process(). Basically just an empty object
# to hold attributes, but performs initialization of defaults.

class BasicOptDataVal:
    def __init__(self,opts):		
        for otype,s,l,attr,h in opts:
            # string & int default to None			
            if otype in [OPT_STRING,OPT_INT]:
                setattr(self,attr,None)
                
            # string list defaults to []				
            elif otype == OPT_LIST: 
                setattr(self,attr,[])
                
            # bool defaults to 0 (off)				
            elif otype == OPT_BOOL: 
                setattr(self,attr,0)
            else:
                raise "* internal error *"
            
    def __str__(self):
        s = 'BasicOptDataVal:\n'
        for k,v in self.__dict__.items():
            s = s + '    %s: %s\n' % (str(k),str(v))

        return s
    
