#!/usr/bin/env python3
"""Tally lines of code across a git repo.

Walks `git ls-files` (so the count matches what's actually tracked on
GitHub — respects .gitignore and excludes large binary artefacts),
classifies each file by extension, and counts:

  total    — every line, including blank
  blank    — lines that are entirely whitespace
  comment  — lines that are entirely a comment (line + block, per language)
  code     — total - blank - comment

Output: per-language summary table, top-20 largest files, grand total.

Usage:
  python3 repo_loc.py                          # current directory's repo
  python3 repo_loc.py /path/to/another/repo
  python3 repo_loc.py --detail                 # also list every file
  python3 repo_loc.py --exclude-generated      # drop auto-generated JSON
                                               # (routines, jacobian dumps,
                                               # ODrive endpoint tables,
                                               # tuning data, training data)
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from collections import defaultdict
from pathlib import Path


# ---------- Language classification by extension --------------------------

# Each entry: extension → (language label, line-comment prefix tuple,
# block-comment open/close tuple or None).
LANGUAGES: dict[str, tuple[str, tuple[str, ...], tuple[str, str] | None]] = {
    '.py':    ('Python',     ('#',),         None),
    '.pyx':   ('Python',     ('#',),         None),
    '.js':    ('JavaScript', ('//',),        ('/*', '*/')),
    '.mjs':   ('JavaScript', ('//',),        ('/*', '*/')),
    '.ts':    ('TypeScript', ('//',),        ('/*', '*/')),
    '.tsx':   ('TypeScript', ('//',),        ('/*', '*/')),
    '.html':  ('HTML',       (),             ('<!--', '-->')),
    '.htm':   ('HTML',       (),             ('<!--', '-->')),
    '.css':   ('CSS',        (),             ('/*', '*/')),
    '.scss':  ('CSS',        ('//',),        ('/*', '*/')),
    '.c':     ('C',          ('//',),        ('/*', '*/')),
    '.h':     ('C',          ('//',),        ('/*', '*/')),
    '.cpp':   ('C++',        ('//',),        ('/*', '*/')),
    '.hpp':   ('C++',        ('//',),        ('/*', '*/')),
    '.cc':    ('C++',        ('//',),        ('/*', '*/')),
    '.java':  ('Java',       ('//',),        ('/*', '*/')),
    '.go':    ('Go',         ('//',),        ('/*', '*/')),
    '.rs':    ('Rust',       ('//',),        ('/*', '*/')),
    '.sh':    ('Shell',      ('#',),         None),
    '.bash':  ('Shell',      ('#',),         None),
    '.zsh':   ('Shell',      ('#',),         None),
    '.yaml':  ('YAML',       ('#',),         None),
    '.yml':   ('YAML',       ('#',),         None),
    '.toml':  ('TOML',       ('#',),         None),
    '.ini':   ('INI',        (';', '#'),     None),
    '.cfg':   ('INI',        (';', '#'),     None),
    '.json':  ('JSON',       (),             None),
    '.xml':   ('XML',        (),             ('<!--', '-->')),
    '.md':    ('Markdown',   (),             None),
    '.rst':   ('reStructuredText', (),       None),
    '.tex':   ('LaTeX',      ('%',),         None),
    '.makefile': ('Make',    ('#',),         None),
    '.cmake': ('CMake',      ('#',),         None),
    '.dockerfile': ('Dockerfile', ('#',),    None),
}

# Files whose name (not extension) determines the language.
LANGUAGE_BY_NAME: dict[str, str] = {
    'Makefile':     '.makefile',
    'CMakeLists.txt': '.cmake',
    'Dockerfile':   '.dockerfile',
    'package.xml':  '.xml',
    'setup.py':     '.py',
    'setup.cfg':    '.cfg',
}


def classify(path: Path) -> tuple[str, tuple[str, ...], tuple[str, str] | None] | None:
    """Return (language, line_prefixes, block_comment) or None for binary
    / unknown / blacklisted files."""
    name = path.name
    ext = path.suffix.lower()
    if name in LANGUAGE_BY_NAME:
        ext = LANGUAGE_BY_NAME[name]
    if ext not in LANGUAGES:
        return None
    return LANGUAGES[ext]


# ---------- Per-file line counter ----------------------------------------

def count_file(path: Path,
               line_prefixes: tuple[str, ...],
               block: tuple[str, str] | None
               ) -> tuple[int, int, int]:
    """Return (blank, comment, code). total = sum of the three."""
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            text = f.read()
    except (OSError, UnicodeDecodeError):
        return (0, 0, 0)
    blank = comment = code = 0
    in_block = False
    block_open = block[0] if block else None
    block_close = block[1] if block else None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            blank += 1
            continue
        # Block-comment state machine. Opens and closes can be on the
        # same line; if a line is "code /* comment" we count it as
        # code (it has non-comment content).
        line_consumed_by_block = False
        if in_block:
            close_idx = line.find(block_close) if block_close else -1
            if close_idx == -1:
                # Whole line inside block → comment.
                comment += 1
                line_consumed_by_block = True
            else:
                # Block ends; remainder may be code or comment.
                rest = line[close_idx + len(block_close):].strip()
                in_block = False
                if not rest:
                    comment += 1
                    line_consumed_by_block = True
                else:
                    # Re-process the remainder as a fresh line.
                    line = rest
        if line_consumed_by_block:
            continue
        # Look for new block-comment open. If it opens AND closes on
        # the same line with no other content, count as comment.
        if block_open and line.startswith(block_open):
            close_idx = (line.find(block_close, len(block_open))
                         if block_close else -1)
            if close_idx == -1:
                in_block = True
                comment += 1
                continue
            # Same-line open and close. Check for trailing content.
            rest = line[close_idx + len(block_close):].strip()
            if not rest:
                comment += 1
                continue
            # Trailing content present → treat the whole line as code
            # (it has non-comment substance).
            code += 1
            continue
        # Line-comment prefixes
        if any(line.startswith(p) for p in line_prefixes):
            comment += 1
            continue
        code += 1
    return (blank, comment, code)


# ---------- Repo walker ---------------------------------------------------

def list_tracked(repo: Path) -> list[Path]:
    """git ls-files inside the repo. Excludes gitignored + untracked.
    This matches what's actually pushed to GitHub."""
    try:
        r = subprocess.run(
            ['git', '-C', str(repo), 'ls-files'],
            capture_output=True, text=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f'git ls-files failed: {e}', file=sys.stderr)
        sys.exit(2)
    return [repo / line for line in r.stdout.splitlines()
            if line.strip()]


# Path patterns whose contents are auto-generated and shouldn't count
# as "lines of code." Matched against the repo-relative path; a path
# is excluded if any of these strings appears anywhere in it.
GENERATED_PATTERNS: tuple[str, ...] = (
    'tuning_data/',                    # auto-tune session JSONs + bags
    'training_data/',                  # YOLO labels + cached metadata
    'stewart_bringup/config/routines/',  # waypoint dumps from path-recorder
    'stewart_bringup/data/odrive_',    # ODrive endpoint table dumps
    '/jacobian.json',                  # system-ID Jacobian dumps
)


def is_generated(rel_path: str) -> bool:
    return any(pat in rel_path for pat in GENERATED_PATTERNS)


# ---------- Reporting ----------------------------------------------------

def fmt_int(n: int) -> str:
    return f'{n:,}'


def render_table(headers: list[str], rows: list[list[str]],
                 aligns: list[str]) -> str:
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))
    out: list[str] = []
    def fmt_row(cells: list[str]) -> str:
        parts = []
        for i, cell in enumerate(cells):
            if aligns[i] == 'l':
                parts.append(cell.ljust(widths[i]))
            else:
                parts.append(cell.rjust(widths[i]))
        return '  '.join(parts)
    out.append(fmt_row(headers))
    out.append('  '.join('-' * w for w in widths))
    for row in rows:
        out.append(fmt_row(row))
    return '\n'.join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('repo', nargs='?', default='.',
                        help='path to the git repo (default: current dir)')
    parser.add_argument('--detail', action='store_true',
                        help='print every counted file in addition to the summary')
    parser.add_argument('--top', type=int, default=20,
                        help='number of largest files to list (default: 20)')
    parser.add_argument('--exclude-generated', action='store_true',
                        help=('drop auto-generated JSON (routine waypoints, '
                              'jacobian dumps, ODrive endpoint tables, '
                              'tuning_data/, training_data/) from the count'))
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    if not (repo / '.git').exists() and not (repo.parent / '.git').exists():
        # Allow running inside a subdir of the repo.
        try:
            r = subprocess.run(
                ['git', '-C', str(repo), 'rev-parse', '--show-toplevel'],
                capture_output=True, text=True, check=True)
            repo = Path(r.stdout.strip()).resolve()
        except subprocess.CalledProcessError:
            print(f'not a git repo: {repo}', file=sys.stderr)
            sys.exit(2)

    files = list_tracked(repo)
    by_lang: dict[str, dict[str, int]] = defaultdict(
        lambda: {'files': 0, 'total': 0, 'blank': 0,
                 'comment': 0, 'code': 0})
    per_file: list[tuple[str, str, int, int, int, int]] = []
    skipped = 0
    excluded_generated = 0
    for f in files:
        rel = str(f.relative_to(repo))
        if args.exclude_generated and is_generated(rel):
            excluded_generated += 1
            continue
        cls = classify(f)
        if cls is None:
            skipped += 1
            continue
        lang, line_prefixes, block = cls
        if not f.is_file():
            skipped += 1
            continue
        blank, comment, code = count_file(f, line_prefixes, block)
        total = blank + comment + code
        if total == 0:
            continue
        per_file.append((lang, rel, total, blank, comment, code))
        d = by_lang[lang]
        d['files'] += 1
        d['total'] += total
        d['blank'] += blank
        d['comment'] += comment
        d['code'] += code

    # Header.
    print(f'Repo: {repo}')
    try:
        head = subprocess.run(
            ['git', '-C', str(repo), 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, check=True).stdout.strip()
        branch = subprocess.run(
            ['git', '-C', str(repo), 'rev-parse', '--abbrev-ref', 'HEAD'],
            capture_output=True, text=True, check=True).stdout.strip()
        print(f'  {branch} @ {head}')
    except subprocess.CalledProcessError:
        pass
    print(f'  tracked files:  {fmt_int(len(files))}')
    extra = ''
    if args.exclude_generated:
        extra = f', excluded {fmt_int(excluded_generated)} generated'
    print(f'  counted:        {fmt_int(len(per_file))}  '
          f'(skipped {fmt_int(skipped)} non-source / unknown{extra})')
    print()

    # By language.
    rows: list[list[str]] = []
    lang_sorted = sorted(by_lang.items(),
                         key=lambda kv: kv[1]['code'], reverse=True)
    for lang, d in lang_sorted:
        rows.append([
            lang,
            fmt_int(d['files']),
            fmt_int(d['total']),
            fmt_int(d['blank']),
            fmt_int(d['comment']),
            fmt_int(d['code']),
        ])
    print(render_table(
        ['Language', 'files', 'total', 'blank', 'comment', 'code'],
        rows,
        ['l', 'r', 'r', 'r', 'r', 'r']))
    print()

    # Grand totals.
    g_files = sum(d['files'] for d in by_lang.values())
    g_total = sum(d['total'] for d in by_lang.values())
    g_blank = sum(d['blank'] for d in by_lang.values())
    g_comment = sum(d['comment'] for d in by_lang.values())
    g_code = sum(d['code'] for d in by_lang.values())
    print(f'Total: {fmt_int(g_files)} files, '
          f'{fmt_int(g_total)} lines '
          f'({fmt_int(g_blank)} blank, '
          f'{fmt_int(g_comment)} comment, '
          f'{fmt_int(g_code)} code)')
    print()

    # Top-N largest by code lines.
    if args.top > 0 and per_file:
        per_file.sort(key=lambda r: r[5], reverse=True)
        top_rows = []
        for lang, rel, total, blank, comment, code in per_file[:args.top]:
            top_rows.append([
                rel, lang,
                fmt_int(total), fmt_int(blank),
                fmt_int(comment), fmt_int(code),
            ])
        print(f'Top {min(args.top, len(per_file))} largest files (by code):')
        print(render_table(
            ['file', 'lang', 'total', 'blank', 'comment', 'code'],
            top_rows,
            ['l', 'l', 'r', 'r', 'r', 'r']))
        print()

    if args.detail:
        per_file.sort(key=lambda r: (r[0], r[1]))
        print('All counted files:')
        for lang, rel, total, blank, comment, code in per_file:
            print(f'  {lang:14s}  '
                  f'{fmt_int(code):>8s} code  '
                  f'{fmt_int(total):>8s} total  '
                  f'{rel}')


if __name__ == '__main__':
    main()
