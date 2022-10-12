
#
# a lineop takes a single line of text and performs
# a transformation on it, returning the new text.
#
# frankm@hiwaay.net
#

# public interface
__all__ = ['to_platform_text','to_portable_text',
           'tabify_line', 'untabify_line', 'copy_line']

import os, re

# a null transform for convenience
def copy_line(line):
    return line

def strip_line_ending( line ):
    """strip off all \r,\n (yes, really do all combinations -
    this cleans up lines in the case of editing a text file
    on the 'wrong' platform and ending up with weird endings.
    Also check the front since 'foreign' line endings can be
    split strangely.)"""

    # surely there's a better way?
    while len(line) and line[-1] in '\n\r':
        line = line[:-1]

    while len(line) and line[0] in '\n\r':
        line = line[1:]
        
    return line

#
# I define '\n' endings as 'portable' text since you can
# take a file in '\n' format and do this:
#
#     open( out, 'w' ).write( open( in, 'r' ) )
#
# ... and end up with the correct line endings on all
# platforms. The same is NOT true if you take a DOS file
# and do the above on Unix - it'll have wrong line endings.
# Therefore, '\n' is the defacto 'portable' format.
#

def to_platform_text( line ):
    """Convert line to platform-specific format, stripping
    any existing line ending characters."""
    
    # in case it's zero-length, don't add chars
    if not len(line):
        return ''

    return strip_line_ending(line) + os.linesep

def to_portable_text( line ):
    """Convert line to portable format, stripping
    any existing line ending characters."""
    
    # in case it's zero-length, don't add chars
    if not len(line):
        return ''

    return strip_line_ending(line) + '\n'

#-----------------------------------------------------
# start of code adapted from IDLE
#-----------------------------------------------------

# modified from tabify/untabify_region_event to
# just do a single line
def tabify_line( line, TABWIDTH=4 ):
    "Tabify a line (replace spaces with tabs)"

    #print "LINE ",line
    raw, effective = classifyws(line, TABWIDTH)
    #print "RAW,EFF",raw,effective
    ntabs, nspaces = divmod(effective, TABWIDTH)
    #print "NRTAB,NRSPACE",ntabs,nspaces
    
    return '\t' * ntabs + ' ' * nspaces + line[raw:]

def untabify_line( line, TABWIDTH=4 ):
    "Untabify a line (replace tabs with spaces)"
    #return line.expandtabs(TABWIDTH)

    # note - I only want to replace leading tabs.
    # line.expandtabs() replaces embedded tabs also.
    
    i = 0
    out = ''
    while line[i] == '\t':
        out += '\x20' * TABWIDTH
        i += 1

    out += line[i:]
    
    return out

# Look at the leading whitespace in s.
# Return pair (# of leading ws characters,
#			   effective # of leading blanks after expanding
#			   tabs to width tabwidth)
def classifyws(s, tabwidth):
    raw = effective = 0
    for ch in s:
        if ch == ' ':
            raw = raw + 1
            effective = effective + 1
        elif ch == '\t':
            raw = raw + 1
            effective = (effective // tabwidth + 1) * tabwidth
        else:
            break
    return raw, effective

#-----------------------------------------------------
# end of code adapted from IDLE
#-----------------------------------------------------

