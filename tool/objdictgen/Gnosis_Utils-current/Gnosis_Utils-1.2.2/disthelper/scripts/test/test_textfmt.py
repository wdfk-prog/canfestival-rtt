

# test plattext.py and porttext.py
#
# raises exception on error

lines_1 = ['aaa','bbb','ccc','ddd','eee']

import sys, os

def test_plattext( txt_port, txt_plat ):
    open('a.txt','wb').write(txt_port)

    os.system('%s %s a.txt' % (sys.executable, '../plattext.py'))

    buf = open('a.txt','rb').read()
    if buf != txt_plat:
        print "GOT ",repr(buf)
        print "EXPECT ",repr(txt_plat)
        raise Exception("plattext FAILED")
    else:
        print "plattext: OK"

def test_porttext( txt_plat, txt_port ):
    open('a.txt','wb').write(txt_plat)

    os.system('%s %s a.txt' % (sys.executable, '../porttext.py'))

    buf = open('a.txt','rb').read()
    if buf != txt_port:
        print "GOT ",repr(buf)
        print "EXPECT ",repr(txt_port)
        raise Exception("plattext FAILED")
    else:
        print "plattext: OK"		

txt_port = '\n'.join(lines_1) + '\n' # make sure last line has '\n'
txt_plat = os.linesep.join(lines_1) + os.linesep # make sure last line as separator

print "TEST %s, %s" % (repr(txt_port),repr(txt_plat))
test_plattext( txt_port, txt_plat )
test_porttext( txt_plat, txt_port )

# show that 'foreign' line endings are converted to portable
# format correctly

print "TEST %s, %s" % (repr(txt_port),repr(txt_plat))
txt_port = '\n'.join(lines_1) + '\n' 
txt_plat = '\n\r'.join(lines_1) 

test_porttext( txt_plat, txt_port )
