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


untabbed_1 = """
\"\"\"
  Two spaces inside a comment.
\"\"\"

def aaa():
\x20\x20\x20\x20i = 1
"""

tabbed_1 = """
\"\"\"
  Two spaces inside a comment.
\"\"\"

def aaa():
\ti = 1
"""

test_tabify(untabbed_1, tabbed_1)
