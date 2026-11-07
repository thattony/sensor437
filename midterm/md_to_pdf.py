# -*- coding: utf-8 -*-
"""
md_to_pdf.py - turn the report draft (Markdown) into a PDF.

    python md_to_pdf.py midterm_report1.md midterm_report1.pdf

Handles what the report uses: # headings, paragraphs, - / 1. lists, | tables |,
**bold**, `code`, ![caption](image.png). Renders to HTML, then prints it to PDF
with Microsoft Edge in headless mode (installed on every lab PC).
"""
import html
import os
import re
import subprocess
import sys

EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"

CSS = """
body { font-family: "Segoe UI", Arial, sans-serif; font-size: 10.5pt; line-height: 1.38; color: #111;
       max-width: 7.2in; margin: 0 auto; }
h1 { font-size: 17pt; margin: 0 0 4pt; }
h2 { font-size: 13pt; margin: 16pt 0 4pt; border-bottom: 1px solid #999; padding-bottom: 2pt; break-after: avoid; }
h3 { font-size: 11pt; margin: 11pt 0 3pt; }
p { margin: 4pt 0; }
table { border-collapse: collapse; margin: 6pt 0; font-size: 9.5pt; }
th, td { border: 1px solid #888; padding: 2pt 6pt; text-align: left; vertical-align: top; }
th { background: #eee; }
code { font-family: Consolas, monospace; font-size: 9.5pt; background: #f3f3f3; padding: 0 2px; }
pre { font-family: Consolas, monospace; font-size: 9pt; background: #f3f3f3; padding: 5pt; white-space: pre-wrap; }
img { max-width: 100%; display: block; margin: 6pt auto 2pt; }
.cap { text-align: center; font-size: 9pt; color: #333; margin-bottom: 8pt; }
ul, ol { margin: 3pt 0 3pt 18pt; padding: 0; }
li { margin: 1pt 0; }
@page { size: Letter; margin: 0.6in; }
"""


def inline(s):
    s = html.escape(s, quote=False)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", s)
    s = re.sub(r"(?<![\w*])\*([^*\s][^*]*)\*(?![\w*])", r"<i>\1</i>", s)
    return s


def convert(md, base):
    out = []
    lines = md.splitlines()
    i = 0
    para = []

    def flush():
        if para:
            out.append("<p>" + inline(" ".join(para)) + "</p>")
            para.clear()

    while i < len(lines):
        ln = lines[i].rstrip()
        if not ln.strip():
            flush(); i += 1; continue
        if ln.startswith("```"):
            flush()
            j = i + 1
            block = []
            while j < len(lines) and not lines[j].startswith("```"):
                block.append(lines[j]); j += 1
            out.append("<pre>" + html.escape("\n".join(block)) + "</pre>")
            i = j + 1; continue
        if re.fullmatch(r"-{3,}", ln.strip()):
            flush()
            out.append("<hr>")
            i += 1; continue
        m = re.match(r"^(#{1,3})\s+(.*)", ln)
        if m:
            flush()
            n = len(m.group(1))
            out.append("<h%d>%s</h%d>" % (n, inline(m.group(2)), n))
            i += 1; continue
        m = re.match(r"^!\[(.*)\]\((.*)\)", ln.strip())
        if m:
            flush()
            src = m.group(2)
            if not os.path.isabs(src):
                src = os.path.join(base, src)
            out.append('<img src="file:///%s"><div class="cap">%s</div>' % (src.replace("\\", "/"), inline(m.group(1))))
            i += 1; continue
        if ln.lstrip().startswith("|"):
            flush()
            rows = []
            while i < len(lines) and lines[i].lstrip().startswith("|"):
                cells = lines[i].strip().strip("|").replace("\\|", "\x00").split("|")
                rows.append([c.strip().replace("\x00", "|") for c in cells])
                i += 1
            t = ["<table>"]
            for k, r in enumerate(rows):
                if k == 1 and all(re.fullmatch(r":?-+:?", c) for c in r):
                    continue
                tag = "th" if k == 0 else "td"
                t.append("<tr>" + "".join("<%s>%s</%s>" % (tag, inline(c), tag) for c in r) + "</tr>")
            t.append("</table>")
            out.append("".join(t)); continue
        m = re.match(r"^(\s*)([-*]|\d+\.)\s+(.*)", ln)
        if m:
            flush()
            ordered = m.group(2)[0].isdigit()
            items = []
            while i < len(lines):
                mm = re.match(r"^(\s*)([-*]|\d+\.)\s+(.*)", lines[i])
                if mm:
                    items.append(mm.group(3)); i += 1
                elif lines[i].startswith("   ") and lines[i].strip() and items:
                    items[-1] += " " + lines[i].strip(); i += 1
                else:
                    break
            tag = "ol" if ordered else "ul"
            out.append("<%s>%s</%s>" % (tag, "".join("<li>%s</li>" % inline(x) for x in items), tag))
            continue
        para.append(ln.strip())
        i += 1
    flush()
    return "<!doctype html><html><head><meta charset='utf-8'><style>%s</style></head><body>%s</body></html>" % (
        CSS, "\n".join(out))


def main():
    src, dst = sys.argv[1], sys.argv[2]
    src, dst = os.path.abspath(src), os.path.abspath(dst)
    base = os.path.dirname(src)
    page = convert(open(src, encoding="utf-8").read(), base)
    tmp = os.path.splitext(dst)[0] + ".html"
    open(tmp, "w", encoding="utf-8").write(page)
    subprocess.run([EDGE, "--headless", "--disable-gpu", "--no-pdf-header-footer",
                    "--print-to-pdf=" + dst, "file:///" + tmp.replace("\\", "/")], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("wrote", dst)


if __name__ == "__main__":
    main()
