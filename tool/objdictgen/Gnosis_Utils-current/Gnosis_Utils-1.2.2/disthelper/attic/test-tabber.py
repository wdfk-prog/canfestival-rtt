
#
# Test program for tabber.py
#
# Note, it is very important NOT to tabify/untabify/reformat
# the input files (t*.txt) -- the test code expects them
# to have either tabs or spaces, and changing them will
# give false failures.
#

#
# frankm@hiwaay.net
#

from difflib import Differ, unified_diff
import string
import os, sys
from shutil import copy as filecopy

def test_tabber( filename, test1, test2 ):

    name2 = '%s.a' % filename
    name3 = '%s.b' % filename

    # copy name -> name.a and run test1 on name.a
    filecopy( filename, name2 )
    cmd = 'python tab.py %s %s' % (test1,name2)
    print "Command: ",cmd
    os.system(cmd)

    # load both files
    lines_a = open( filename, 'rb' ).readlines()
    lines_b = open( name2, 'rb' ).readlines()	

    # now, a straight diff should show changes
    df = list(unified_diff(lines_a,lines_b))
    if len(df) == 0:
        print "*** ERROR - expected a change when tabifying %s" % filename
        sys.exit(1)
    #else:
    #	print ''.join(df)

    # however, after lstripping each line, they should be
    # equal again
    lines_a = map( string.lstrip, lines_a )
    lines_b = map( string.lstrip, lines_b )
    
    df = list(unified_diff(lines_a,lines_b))
    if len(df) != 0:
        print "*** ERROR - not the same after %s %s" % (test1,filename)
    else:
        print 'OK - leading whitespace changed as expected.'
        
    # copy name -> name.b and run test2 on name.b
    filecopy(name2, name3)
    cmd = 'python tab.py %s %s' % (test2,name3)
    print "Command: ",cmd
    os.system(cmd)

    # now original & name3 should be identical
    lines_a = open( filename, 'rb' ).readlines()
    lines_b = open( name3, 'rb' ).readlines()	

    df = list(unified_diff(lines_a,lines_b))
    if len(df) != 0:
        print "*** ERROR - lossage after %s %s" % (test2,filename)
        sys.exit(1)
    else:
        print "OK - no changes after %s -> %s" % (test1,test2)

# t1 has spaces, so run --tabify first
test_tabber('t1.txt','--tabify','--untabify')

# t2 has tabs, so run --untabify first
test_tabber('t2.txt','--untabify','--tabify')

# t3 has spaces, so run --tabify first
test_tabber('t3.txt','--tabify','--untabify')

# t4 has spaces, so run --tabify first
test_tabber('t4.txt','--tabify','--untabify')
