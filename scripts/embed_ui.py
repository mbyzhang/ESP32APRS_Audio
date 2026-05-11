#!/usr/bin/env python3
"""Regenerate include/embedded_ui.h from data/index.html, app.css, app.js.

The chat UI assets are embedded into firmware as **gzip-compressed byte
arrays**.  At runtime the device serves them with `Content-Encoding: gzip`,
which the browser transparently decompresses.  Two reasons we do this:

1. The kv4p-ht runs on a fragmented heap; a 7.8 KB inline `index.html`
   needs ~6 KB contiguous for ESPAsyncWebServer's per-ACK buffer and
   reliably hangs.  Gzipped it's 2-3 KB and fits.
2. App.js (56 KB) compresses to ~12 KB — much faster page loads even on
   strong WiFi.

Re-run after any edit to data/*:

    python3 scripts/embed_ui.py
"""
import gzip
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [
    ("data/index.html", "EMBEDDED_INDEX_HTML_GZ"),
    ("data/app.css",    "EMBEDDED_APP_CSS_GZ"),
    ("data/app.js",     "EMBEDDED_APP_JS_GZ"),
]
OUT = os.path.join(ROOT, "include", "embedded_ui.h")

def to_c_array(name: str, data: bytes) -> str:
    out = [f"// {name}: {len(data)} bytes (gzipped)",
           f"static const unsigned int  {name}_LEN = {len(data)};",
           f"static const unsigned char {name}[] = {{"]
    line = []
    for i, b in enumerate(data):
        line.append(f"0x{b:02x}")
        if len(line) >= 16:
            out.append("    " + ",".join(line) + ",")
            line = []
    if line:
        out.append("    " + ",".join(line))
    out.append("};")
    return "\n".join(out)

def main():
    chunks = [
        "// Auto-generated from data/* by scripts/embed_ui.py — do not edit by hand.",
        "// Re-run after changing any UI source file.",
        "",
        "#pragma once",
        "",
    ]
    total_src = 0
    total_gz = 0
    for rel, sym in FILES:
        path = os.path.join(ROOT, rel)
        with open(path, "rb") as f:
            raw = f.read()
        # mtime=0 makes the gzip output deterministic across rebuilds.
        gz = gzip.compress(raw, compresslevel=9, mtime=0)
        total_src += len(raw)
        total_gz += len(gz)
        chunks.append(to_c_array(sym, gz))
        chunks.append("")
        print(f"  {rel:24}  {len(raw):6} → {len(gz):6} bytes ({len(gz)*100//max(len(raw),1)}%)")
    open(OUT, "w", encoding="utf-8").write("\n".join(chunks))
    print(f"wrote {OUT}: {os.path.getsize(OUT)} bytes "
          f"(source {total_src} → gz {total_gz})")

if __name__ == "__main__":
    main()
