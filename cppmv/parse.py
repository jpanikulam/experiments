def splitstrip(text, delim=','):
    return list(map(lambda o: o.strip(), text.split(delim)))


def needs(text, requirements):
    for requirement in requirements:
        assert requirement in text, "{} missing {}".format(text, requirement)


def mustnot(text, disquirements):
    for requirement in disquirements:
        assert requirement not in text, "{} may not have {}".format(text, requirement)


def between(text, ldelim='(', rdelim=')'):
    needs(text, [ldelim, rdelim])
    start = text.find(ldelim)
    end = text.rfind(rdelim)
    assert end - start > 1, "Delimeters empty"
    return text[start + 1: end]
