"""AeroCore 0.41: Simplified Chinese from the Traditional: lang/zh_tw.json to lang/zh_cn.json, the same keys on the same
lines (only the values change). OpenCC's tw2sp (Taiwan phrases to mainland ones), then the mainland terms of
scripts/zh_cn_terms.json. Run after any change to zh_tw:

    py -3 -m pip install --user opencc-python-reimplemented    (once)
    py -3 scripts/zh_cn.py

Other text (READMEs, release pages): py -3 scripts/zh_cn.py <file in> <file out>.
"""
import json
import os
import re
import sys

import opencc

HERE = os.path.dirname(os.path.abspath(__file__))
LANG = os.path.join(HERE, '..', 'neoforge', 'src', 'main', 'resources', 'assets', 'aerocore', 'lang')
TERMS = json.load(open(os.path.join(HERE, 'zh_cn_terms.json'), encoding='utf-8'))
TERMS = sorted(((k, v) for k, v in TERMS.items() if k != '_'), key=lambda kv: -len(kv[0]))
CC = opencc.OpenCC('tw2sp')
PROTECTED = re.compile(r'(\{[^{}]*\}|%(?:\d+\$)?[sd]|`[^`]*`|https?://\S+)')
VALUE = re.compile(r'^(\s*"[^"]+":\s*)"((?:[^"\\]|\\.)*)"(,?)\s*$')


def convert(text):
    """Simplified Chinese of a Traditional text; {placeholders}, %s, `code` and links are kept as they are."""
    out = []
    for part in PROTECTED.split(text):
        if not part:
            continue
        if PROTECTED.fullmatch(part):
            out.append(part)
            continue
        s = CC.convert(part)
        for a, b in TERMS:
            s = s.replace(a, b)
        out.append(s)
    return ''.join(out)


def write(path, text):
    old = open(path, encoding='utf-8').read() if os.path.exists(path) else None
    if old != text:
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        return 1
    return 0


def main():
    if len(sys.argv) == 3:
        write(sys.argv[2], convert(open(sys.argv[1], encoding='utf-8').read()))
        return
    src = os.path.join(LANG, 'zh_tw.json')
    out = []
    for line in open(src, encoding='utf-8').read().split('\n'):
        m = VALUE.match(line)
        if not m:
            out.append(line)
            continue
        out.append(m.group(1) + json.dumps(convert(json.loads('"' + m.group(2) + '"')), ensure_ascii=False) + m.group(3))
    text = '\n'.join(out)
    if list(json.loads(open(src, encoding='utf-8').read())) != list(json.loads(text)):
        sys.exit('zh_cn lost or moved a key')
    print('zh_cn:', write(os.path.join(LANG, 'zh_cn.json'), text), 'file written')


if __name__ == '__main__':
    main()
