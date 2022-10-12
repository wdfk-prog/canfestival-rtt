
#
# run an untab/tab/untab cycle over a copy of the entire
# gnosis & disthelper trees and diff the results
#

import os,shutil,sys
from stat import *

if not os.path.isfile('setup.py'):
    raise "You must run this from the toplevel"

if os.path.isdir('ttt'):
    raise "ttt already exists - bailing out!"

os.mkdir('ttt')
os.chdir('ttt')

shutil.copytree('../gnosis','gnosis')
shutil.copytree('../disthelper','disthelper')

# untabbing is the most foolproof, so use untabbed sources as the reference
print "Untab first copy ..."
os.system('%s disthelper/scripts/untabtree.py -w 4 -r -x py disthelper gnosis' % sys.executable)

os.rename('gnosis','UT-gnosis')
os.rename('disthelper','UT-disthelper')

shutil.copytree('../gnosis','gnosis')
shutil.copytree('../disthelper','disthelper')

# untab copy
print "Untab second copy ..."
os.system('%s disthelper/scripts/untabtree.py -w 4 -r -x py disthelper gnosis' % sys.executable)
# then tab
print "Tab second copy ..."
os.system('%s disthelper/scripts/tabtree.py -r -x py disthelper gnosis' % sys.executable)
# and untab again, hopefully to original condition :-)
print "Untab second copy ..."
os.system('%s disthelper/scripts/untabtree.py -w 4 -r -x py disthelper gnosis' % sys.executable)

print "Diff first and second copies ..."
#os.system('diff --exclude="*.pyc" -u -r UT-disthelper disthelper > diff.disthelper')
#os.system('diff --exclude="*pyc" -u -r UT-gnosis gnosis > diff.gnosis')

os.system('%s disthelper/scripts/difftree.py --exclude="*.pyc" -r UT-disthelper disthelper > diff.disthelper' % sys.executable)
os.system('%s disthelper/scripts/difftree.py --exclude="*pyc" -r UT-gnosis gnosis > diff.gnosis' % sys.executable)

err = 0

if os.stat('diff.disthelper')[ST_SIZE] != 0:
    print "******* WARNING: ttt/diff.disthelper not 0 bytes,"
    err = 1
    
if os.stat('diff.gnosis')[ST_SIZE] != 0:
    print "******* WARNING: ttt/diff.disthelper not 0 bytes,"
    err = 1

if err == 0:
    print "** SUCCESS!! Both diffs were zero bytes! **\n"
    print "** You should `rm -rf ttt` after you inspect the results."
    
