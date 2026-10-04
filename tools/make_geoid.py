#!/usr/bin/env python3
"""Convert the NGA EGM96 15-minute undulation grid (WW15MGH.GRD, public domain) to the compact binary
`data/egm96_15min.bin` read by nav::Geoid: magic "EGM96I16", int32 rows, int32 cols, float64 lat_top_deg,
lon_left_deg, dlat_deg, dlon_deg, then rows*cols int16 undulations in centimetres, north to south, west to east.
    python3 tools/make_geoid.py Cobra/.venv/navtk/WW15MGH.GRD data/egm96_15min.bin
"""
import struct, sys

src, dst = sys.argv[1], sys.argv[2]
with open(src) as f:
    header = f.readline().split()
    lat_s, lat_n, lon_w, lon_e, dlat, dlon = map(float, header)
    values = [float(v) for line in f for v in line.split()]
rows = int(round((lat_n - lat_s) / dlat)) + 1
cols = int(round((lon_e - lon_w) / dlon)) + 1
assert rows * cols == len(values), (rows, cols, len(values))
with open(dst, 'wb') as out:
    out.write(b'EGM96I16')
    out.write(struct.pack('<ii', rows, cols))
    out.write(struct.pack('<dddd', lat_n, lon_w, dlat, dlon))  # first row is the northern edge
    out.write(struct.pack(f'<{rows * cols}h', *[int(round(v * 100)) for v in values]))
print(f'{dst}: {rows} x {cols}, first {values[0]} m (lat {lat_n}), last {values[-1]} m; '
      f'N(0,0)={values[(rows // 2) * cols]} m; N(45N,0)={values[int((lat_n - 45) / dlat) * cols]} m; '
      f'N(40N,255E)={values[int((lat_n - 40) / dlat) * cols + int(255 / dlon)]} m')
