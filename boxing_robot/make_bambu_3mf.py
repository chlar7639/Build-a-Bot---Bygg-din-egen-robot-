#!/usr/bin/env python3
"""
make_bambu_3mf.py - pack the chassis STL files onto print plates and write Bambu Studio
project files (.3mf). Standard library only.

Usage (from the boxing_robot folder):
    python3 make_bambu_3mf.py            # reads ./stl, writes ./bambu/plate_N.3mf
Open a plate_N.3mf in Bambu Studio. Each file is one plate with all its parts laid out.
"""
import os, struct, zipfile, glob
from xml.sax.saxutils import escape

BED_X = 220          # usable plate size (mm). Bambu P1S/A1/X1 are 256; 220 is the spec value
BED_Y = 220
GAP = 5.0            # clearance between parts on a plate (mm)
STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bambu")


def read_stl(path):
    """Return (vertices, triangles) for a binary or ASCII STL, with duplicate vertices merged."""
    with open(path, "rb") as f:
        data = f.read()
    raw = []  # list of 3 (x,y,z) per triangle
    n = struct.unpack("<I", data[80:84])[0] if len(data) >= 84 else -1
    if len(data) == 84 + 50 * n:                                  # binary STL
        for i in range(n):
            vals = struct.unpack("<12f", data[84 + 50 * i: 84 + 50 * i + 48])
            raw.append([vals[3:6], vals[6:9], vals[9:12]])
    else:                                                          # ASCII STL
        tri = []
        for line in data.decode("ascii", errors="ignore").splitlines():
            parts = line.split()
            if parts and parts[0] == "vertex":
                tri.append(tuple(float(v) for v in parts[1:4]))
                if len(tri) == 3:
                    raw.append(tri)
                    tri = []
    index, verts, tris = {}, [], []
    for t in raw:
        idx = []
        for v in t:
            key = tuple(round(c, 4) for c in v)
            if key not in index:
                index[key] = len(verts)
                verts.append(key)
            idx.append(index[key])
        if len(set(idx)) == 3:                                     # drop degenerate triangles
            tris.append(tuple(idx))
    return verts, tris


def bbox(verts):
    xs, ys, zs = zip(*verts)
    return min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)


def pack(items):
    """Shelf-pack (name, verts, tris, w, d) items onto plates. Returns list of plates;
    each plate is a list of (name, verts, tris, x_offset, y_offset)."""
    plates, cur = [], []
    x = y = row_h = 0.0
    for name, verts, tris, w, d in sorted(items, key=lambda i: -i[4]):
        if w > BED_X or d > BED_Y:
            raise SystemExit(f"{name} ({w:.0f} x {d:.0f} mm) does not fit on a {BED_X} x {BED_Y} plate")
        if x + w > BED_X:                       # next row
            x = 0.0
            y += row_h + GAP
            row_h = 0.0
        if y + d > BED_Y:                       # next plate
            plates.append(cur)
            cur, x, y, row_h = [], 0.0, 0.0, 0.0
        cur.append((name, verts, tris, x, y))
        x += w + GAP
        row_h = max(row_h, d)
    if cur:
        plates.append(cur)
    return plates


def write_3mf(path, plate):
    ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
    objs = []
    for i, (name, verts, tris, ox, oy) in enumerate(plate, start=1):
        x0, y0, z0, _, _, _ = bbox(verts)
        dx, dy = ox - x0, oy - y0
        vx = "".join(f'<vertex x="{x + dx:.4f}" y="{y + dy:.4f}" z="{z:.4f}"/>' for x, y, z in verts)
        tr = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in tris)
        objs.append(f'<object id="{i}" name="{escape(name)}" type="model">'
                    f'<mesh><vertices>{vx}</vertices><triangles>{tr}</triangles></mesh></object>')
    items = "".join(f'<item objectid="{i}"/>' for i in range(1, len(plate) + 1))
    model = (f'<?xml version="1.0" encoding="UTF-8"?>\n'
             f'<model unit="millimeter" xml:lang="en-US" xmlns="{ns}">'
             f'<resources>{"".join(objs)}</resources><build>{items}</build></model>')
    content_types = ('<?xml version="1.0" encoding="UTF-8"?>\n'
                     '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                     '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                     '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
                     '</Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)


def main():
    files = sorted(glob.glob(os.path.join(STL_DIR, "*.stl")))
    if not files:
        raise SystemExit(f"No STL files in {STL_DIR}. Run boxing_robot_chassis.py first.")
    items = []
    for f in files:
        verts, tris = read_stl(f)
        x0, y0, z0, x1, y1, z1 = bbox(verts)
        items.append((os.path.splitext(os.path.basename(f))[0], verts, tris, x1 - x0, y1 - y0))
    os.makedirs(OUT_DIR, exist_ok=True)
    plates = pack(items)
    for n, plate in enumerate(plates, start=1):
        path = os.path.join(OUT_DIR, f"plate_{n}.3mf")
        write_3mf(path, plate)
        print(f"plate_{n}.3mf: {len(plate)} parts")
    print(f"Wrote {len(plates)} plate file(s) to {OUT_DIR}")


if __name__ == "__main__":
    main()
