
# test tabify.py and untabify.py
#
# raises exception on error

import sys, os

def test_untabify( txt_tabbed, txt_untabbed ):
    open('a.txt','wb').write(txt_tabbed)

    os.system('%s %s -w 4 a.txt' % (sys.executable, '../untabtree.py'))

    buf = open('a.txt','rb').read()
    if buf != txt_untabbed:
        raise Exception("untabify FAILED")
    else:
        print "untabify: OK"

def test_tabify( txt_untabbed, txt_tabbed ):
    open('a.txt','wb').write(txt_untabbed)

    os.system('%s %s a.txt' % (sys.executable, '../tabtree.py'))

    buf = open('a.txt','rb').read()
    if buf != txt_tabbed:
        raise Exception("tabify FAILED")
    else:
        print "tabify: OK"		

# test 1
tabbed_text_1 = """
def foo( a, b, c ):
\tj = 0
\tfor i in range(10):
\t\tj += i

\treturn j
"""

untabbed_text_1 = """
def foo( a, b, c ):
\x20\x20\x20\x20j = 0
\x20\x20\x20\x20for i in range(10):
\x20\x20\x20\x20\x20\x20\x20\x20j += i

\x20\x20\x20\x20return j
"""

# test 2
tabbed_text_2 = """
def foo( a, b, c ):
\tj = 0
\tfor i in range(10):
\t\tfor k in range(20):
\t\t\tj += i*k

\treturn j
"""

untabbed_text_2 = """
def foo( a, b, c ):
\x20\x20\x20\x20j = 0
\x20\x20\x20\x20for i in range(10):
\x20\x20\x20\x20\x20\x20\x20\x20for k in range(20):
\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20j += i*k

\x20\x20\x20\x20return j
"""

# test 3
tabbed_text_3 = """
def foo( a, b, c ):
\tj = 0
\tfor i in range(10):
\t\tfor k in range(20):
\t\t\t  j += i*k # 2 extra spaces in this line

\treturn j
"""

untabbed_text_3 = """
def foo( a, b, c ):
\x20\x20\x20\x20j = 0
\x20\x20\x20\x20for i in range(10):
\x20\x20\x20\x20\x20\x20\x20\x20for k in range(20):
\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20j += i*k # 2 extra spaces in this line

\x20\x20\x20\x20return j
"""


print tabbed_text_1
test_untabify(tabbed_text_1,untabbed_text_1)
test_tabify(untabbed_text_1,tabbed_text_1)

print tabbed_text_2
test_untabify(tabbed_text_2,untabbed_text_2)
test_tabify(untabbed_text_2,tabbed_text_2)

print tabbed_text_3
test_untabify(tabbed_text_3,untabbed_text_3)
test_tabify(untabbed_text_3,tabbed_text_3)


