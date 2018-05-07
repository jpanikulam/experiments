import json
import struct


packing_rules = {
    'pad byte': 'x',
    'char': 'c',
    'signed char': 'b',
    'unsigned char': 'B',
    '_Bool': '?',
    'short': 'h',
    'unsigned short': 'H',
    'int': 'i',
    'unsigned int': 'I',
    'long': 'l',
    'unsigned long': 'L',
    'long long': 'q',
    'unsigned long long': 'Q',
    'float': 'f',
    'double': 'd',
}


def form_packing_string(data):
    return map(packing_rules.get, data)


def retab(txt, state):
    replace = {
        ',': ',\n',
        '}': '}\n',
        '{': '{\n',
        '[': '[\n',
        ']': ']\n',
    }

    for char in txt:

        if char == '"':
            if not state['in_quotes']:
                state['field_name'] = ""
            state['in_quotes'] = not state['in_quotes']

        if state['in_quotes']:
            state['field_name'] += char

        if not state['in_quotes']:
            if char in ('}'):
                state['scope_level'] -= 1
            elif char in ['{']:
                state['scope_level'] += 1

        new_char = replace.get(char, char)
        state['new_text'] += new_char
        if new_char != char:
            state['new_text'] += '  ' * state['scope_level']


def accumulate(txt, state):
    replace = {
        ',': ',\n',
        '}': '}\n',
        '{': '{\n',
        '[': '[\n',
        ']': ']\n',
    }

    field_stack = []

    for char in txt:

        if char == '"':
            if not state['in_quotes']:
                state['field_name'] = ""
            state['in_quotes'] = not state['in_quotes']

        if state['in_quotes']:
            state['field_name'] += char

        if not state['in_quotes']:
            if char in ('}'):
                state['scope_level'] -= 1
            elif char in ['{']:
                state['scope_level'] += 1

        new_char = replace.get(char, char)
        state['new_text'] += new_char
        if new_char != char:
            state['new_text'] += '  ' * state['scope_level']


def load_first_n_features(txt, n):
    rdata = json.loads(txt)
    new_json = rdata['features'][:n]

    with open('reduced.json', 'w') as foop:
        foop.write(json.dumps(new_json))


def main():
    # file = "/home/jacob/Downloads/doop.geojsoin"
    # file = "/home/jacob/Downloads/Allegheny_County_Parcel_Boundaries.geojson"
    file = "/home/jacob/Downloads/Allegheny_County_Parcel_Boundaries.geojson"

    state = {
        'scope_level': 0,
        'in_quotes': False,
        'field_name': None,
        'field_name_complete': True,
        'new_text': "",
    }

    with open(file) as f:
        txt = f.read()
        print 'Finished reading'
        load_first_n_features(txt, 100000)
        # while(True):
        #     file_string = f.read(10)
        #     if file_string == '':
        #         break
        #     retab(file_string, state)
        # print state['new_text']


if __name__ == '__main__':
    main()
