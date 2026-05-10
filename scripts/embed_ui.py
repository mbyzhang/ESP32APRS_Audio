#!/usr/bin/env python3
"""Regenerate include/embedded_ui.h from data/index.html, app.css, app.js.

Run this whenever you edit the chat UI files in data/ — the firmware
serves these embedded copies (not the LittleFS originals) because the
AsyncFileResponse / beginResponse_P streaming paths have proven flaky
on this device (responses arriving without HTTP/1.1 status lines, which
Safari rejects with "cannot parse response").

Usage: python3 scripts/embed_ui.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [
    ("data/index.html", "EMBEDDED_INDEX_HTML"),
    ("data/app.css",    "EMBEDDED_APP_CSS"),
    ("data/app.js",     "EMBEDDED_APP_JS"),
]
OUT = os.path.join(ROOT, "include", "embedded_ui.h")

def main():
    chunks = [
        "// Auto-generated from data/* by scripts/embed_ui.py — do not edit by hand.",
        "// Re-run scripts/embed_ui.py after changing the source UI files.",
        "",
        "#pragma once",
        "",
    ]
    for rel, sym in FILES:
        path = os.path.join(ROOT, rel)
        text = open(path, encoding="utf-8").read()
        # Pick a raw-string delimiter that doesn't collide with the file's content.
        delim = "EMBED"
        while ")" + delim + '"' in text:
            delim += "X"
        chunks.append(f'static const char *{sym} = R"{delim}(' + text + f'){delim}";')
        chunks.append("")
    body = "\n".join(chunks)
    open(OUT, "w", encoding="utf-8").write(body)
    print(f"wrote {OUT}: {os.path.getsize(OUT)} bytes")

if __name__ == "__main__":
    main()
