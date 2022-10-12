
#
# shortcut script to install Gnosis for all Python versions
# installed on the local machine
#
# Two ways to run:
#
#   1. If sitting in toplevel dir (above gnosis/), will build
#      and install Gnosis from gnosis/ for all python versions,
#      first removing any existing Gnosis installation.
#
#   2. If sitting in dist/, unpack the built sdist and install
#      from it for all Python versions, first removing any
#      existing Gnosis installation.
#
# Requires: tar, rm
#

import os, sys, re
from glob import glob

# so I don't have to do this everywhere below ..
if os.name not in ['posix','os2','nt']:
    # need to know pathsep in PATH, .exe extension (see below)
    print "***"
    print "*** SORRY - don't know how to run on this platform."
    print "***" 
    sys.exit(1)

def run(cmd):
    print "%s" % cmd
    if os.system(cmd) != 0:
        print "ERROR"
        sys.exit(1)

def exe_in_path( name ):
    if os.name == 'posix':
        pathlist = os.getenv('PATH').split(':')     
    else:
        pathlist = os.getenv('PATH').split(';')
        
    for path in pathlist:
        full = os.path.join(path,name)
        if os.path.isfile( full ):
            return 1

    return 0

def enum_pythons():
    # check for versioned binaries first
    vers = ['2.0','2.1','2.2','2.3']
    
    if os.name == 'posix':
        patt = ['python%s'] * len(vers)
    else:
        patt = ['python%s.exe'] * len(vers)

    allpyvers = map(lambda x,y: x % y, patt, vers)
    existpy = filter(lambda x: exe_in_path(x), allpyvers)

    if len(existpy) == 0:
        # on a system w/out versioned names, use default
        if exe_in_path(patt[0] % ''):
            existpy = [patt[0] % '']
            
    return existpy

for py in enum_pythons():
    if os.name == 'posix':
        # remove previous version (only on posix, where it's
        # in a standard location)
        run('rm -rf /usr/lib/%s/site-packages/gnosis' % py)

    l = glob('Gnosis_Utils-*.tar.gz')
    if len(l) == 2:
        print "** Hey, delete the -master first, if you're sitting in dist/"
        sys.exit(1)
        
    if len(l) == 1:
        # I'm sitting in dist/ - unpack the sdist & install
        m = re.match('(Gnosis_Utils-[0-9\.]+)\.tar\.gz',l[0])
        if not m:
            raise "Yikes - what happened?"

        run('tar zxvf Gnosis_Utils*.tar.gz')
        os.chdir(m.group(1))

        run('%s setup.py build' % py)
        run('%s setup.py install' % py)

        os.chdir('..')
        run('rm -rf %s' % m.group(1))       
    else:
        # I'm sitting above gnosis/ - build & install directly
        run('rm -rf build')
        run('%s setup.py build' % py)
        run('%s setup.py install' % py)
    
