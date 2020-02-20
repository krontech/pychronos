#!/usr/bin/python3
import json
import re

def __getindent(line):
    """Count the number of leading whitespace characters in the line"""
    indent = 0
    for ch in line:
        if ch.isspace():
            indent += 1
        else:
            return indent
    return 0

def __blocklen(doclines, baseindent):
    """Count the number of consecutive lines at the same, or deeper indenting as the first"""
    count = 0
    for line in doclines:
        if (len(line) == 0) or line.isspace():
            count += 1
        elif __getindent(line) > baseindent:
            count += 1
        else:
            break
    return count

def __parse_args(doclines):
    # Get the indent at which we expect new arguments.
    baseindent = __getindent(doclines[0])
    arg = None
    results = {}

    # Arguments are declared in the form:
    #   name [optional type and flags] : descriptive text
    argregex = re.compile("\s{%s}([a-z][a-z0-9]*)\s+([^:]*):(.*)" % (baseindent), re.I)
    for line in doclines:
        m = argregex.match(line)
        if (m):
            arg = { "doc": m.group(3).strip() }
            argtype = m.group(2).strip()
            if len(argtype) and argtype[0] == "(" and argtype[-1] == ")":
                arg['type'] = argtype[1:-1].strip()
            results[m.group(1)] = arg
        elif __getindent(line) > baseindent:
            arg["doc"] += " " + line.strip()

    return results

def __parse_block(doclines):
    result = ""
    for line in doclines:
        # Otherwise, it's just text.
        lstrip = line.strip()
        if len(result):
            result += (lstrip + " ") if len(lstrip) else "\n"
        else:
            result = lstrip
    return result

def parse(doc):
    """Parse a google docstring into a dictionary of its components.
    
    Foo bar bazoo, this is a really long line of crazy text that
    spans multiple lines, and is full of run-on sentences that go nowhere
    and have odd stuff.
       It's also badly formatted
          and is trying to do things wrong
    
    Oh, and we have paragraphs too
       that are being difficult

    Args:
        something (int, optional): This is a test of my sanity.
        st34 : This is just being mean
          super duper
            really evil
             crazy mean
    
    Returns:
        A dict containing the google docstring broken down into its
        component sections, for example:
        {'brief': 'Parse a google docstring into a dictionary fo its components.',
         'description': 'Foo bar bazoo, ...'
         'args': {
            'something': {
                'type': 'int, optional',
                'doc': 'This is a tet of my sanity.'
                }
            }
        }

    """
    doclines = doc.splitlines()
    verbose = ""
    result = {
        "brief": doclines[0].strip()
    }

    # Find the starting indent of the second non-empty line.
    # This sets the 'normal' level of indenting that any
    # un-nested line would expect.
    baseindent = 0
    for line in doclines[1:]:
        if len(line) and not line.isspace():
            baseindent = __getindent(line)
            break

    # Prepare some regex to match new blocks.
    bregex = re.compile("^\s{%s}([a-z]*):$" % baseindent, re.I)

    # Parse the docstring line-by-line.
    index = 1
    while (index < len(doclines)):
        line = doclines[index]
        index += 1

        # Check if this starts a new block.
        m = bregex.match(line)
        blocklen = __blocklen(doclines[index:], baseindent)
        if (m and blocklen):
            block = doclines[index:index+blocklen]
            index += blocklen
            if (m.group(1) == "Args"):
                result['args'] = __parse_args(block)
            elif (m.group(1) == "Returns"):
                result['returns'] = __parse_block(block)
            elif (m.group(1) == "Yields"):
                result['yields'] = __parse_block(block)
            continue
        
        # Otherwise, it's just text.
        line = line.strip()
        if len(verbose):
            verbose += (line + " ") if len(line) else "\n"
        else:
            verbose = line

    # Return the parsed docstring.
    if (len(verbose)): result['description'] = verbose.strip()
    return result

# If run as a script, parse ourself
if __name__ == "__main__":
    p = parse(parse.__doc__)
    print(json.dumps(p, indent=3, sort_keys=True))
