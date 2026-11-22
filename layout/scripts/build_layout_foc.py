#!/usr/bin/env python3
"""
Physical Design Layout & GDSII Generator for Sensorless FOC Motor Controller ASIC
Generates full-chip layout stream format (GDSII stream version 7) including:
- Die Boundary (1300 um x 1300 um)
- Power Ring (VDD / VSS on Metal 4 & Metal 5)
- Standard Cell Placement Rows
- Sub-block modules (Clarke, CORDIC 16-stage pipeline, Park, PI Controllers, SVPWM, SMO)
- I/O Wirebond Pads
"""

import struct
import os
import sys

def gds_record(record_type, data_type, data=b""):
    length = len(data) + 4
    header = struct.pack(">HH", length, (record_type << 8) | data_type)
    return header + data

def write_gds(filename):
    out = bytearray()

    # HEADER (record 0x00, type 0x02: 2-byte int)
    out.extend(gds_record(0x00, 0x02, struct.pack(">H", 600)))

    # BGNLIB (record 0x01, type 0x02: 12 2-byte ints)
    out.extend(gds_record(0x01, 0x02, struct.pack(">12H", 2026, 11, 22, 12, 0, 0, 2026, 11, 22, 12, 0, 0)))

    # LIBNAME (record 0x02, type 0x06: ASCII string)
    lib_name = b"FOC_MOTOR_CONTROLLER_LIB\x00"
    if len(lib_name) % 2 != 0: lib_name += b"\x00"
    out.extend(gds_record(0x02, 0x06, lib_name))

    # UNITS (record 0x03, type 0x05: two 8-byte doubles - user units in m, db units in m)
    # 0.001 um database unit (1 nm = 1e-9 m), user unit = 1 um (1e-6 m)
    out.extend(gds_record(0x03, 0x05, struct.pack(">dd", 0.001, 1e-9)))

    # BGNSTR (record 0x05, type 0x02)
    out.extend(gds_record(0x05, 0x02, struct.pack(">12H", 2026, 11, 22, 12, 0, 0, 2026, 11, 22, 12, 0, 0)))

    # STRNAME (record 0x06, type 0x06)
    str_name = b"foc_controller_top\x00"
    if len(str_name) % 2 != 0: str_name += b"\x00"
    out.extend(gds_record(0x06, 0x06, str_name))

    def add_box(layer, datatype, x1, y1, x2, y2):
        b = bytearray()
        b.extend(gds_record(0x08, 0x00)) # BOUNDARY
        b.extend(gds_record(0x0D, 0x02, struct.pack(">H", layer)))
        b.extend(gds_record(0x0E, 0x02, struct.pack(">H", datatype)))
        # XY coordinates (5 points, closed loop)
        coords = struct.pack(">10i", x1, y1, x2, y1, x2, y2, x1, y2, x1, y1)
        b.extend(gds_record(0x10, 0x03, coords))
        b.extend(gds_record(0x11, 0x00)) # ENDEL
        return b

    # 1. Die Boundary (Layer 100): 1300 um x 1300 um (1300000 nm x 1300000 nm)
    out.extend(add_box(100, 0, 0, 0, 1300000, 1300000))

    # 2. Core Area (Layer 108): 100 um margin -> 100,000 to 1,200,000
    out.extend(add_box(108, 0, 100000, 100000, 1200000, 1200000))

    # 3. Power Rings (Metal 4: Layer 31, Metal 5: Layer 32)
    # VDD Outer Ring
    out.extend(add_box(31, 0, 90000, 90000, 1210000, 105000))
    out.extend(add_box(31, 0, 90000, 1195000, 1210000, 1210000))
    out.extend(add_box(32, 0, 90000, 90000, 105000, 1210000))
    out.extend(add_box(32, 0, 1195000, 90000, 1210000, 1210000))

    # VSS Inner Ring
    out.extend(add_box(31, 1, 110000, 110000, 1190000, 125000))
    out.extend(add_box(31, 1, 110000, 1175000, 1190000, 1190000))
    out.extend(add_box(32, 1, 110000, 110000, 125000, 1190000))
    out.extend(add_box(32, 1, 1175000, 110000, 1190000, 1190000))

    # 4. Functional Subsystem Floorplanning Blocks (Layer 40)
    # CORDIC 16-Stage Rotation Engine (Center Core: 400 um x 400 um)
    out.extend(add_box(40, 1, 450000, 450000, 850000, 850000))
    # Clarke & Park Forward Transforms (West: 250 um x 300 um)
    out.extend(add_box(40, 2, 150000, 500000, 400000, 800000))
    # Dual PI Current Controllers (North: 350 um x 250 um)
    out.extend(add_box(40, 3, 450000, 900000, 850000, 1150000))
    # Inverse Park & SVPWM Generator (East: 300 um x 350 um)
    out.extend(add_box(40, 4, 900000, 450000, 1180000, 800000))
    # Sliding Mode Observer (South: 350 um x 250 um)
    out.extend(add_box(40, 5, 450000, 150000, 850000, 400000))

    # 5. Standard Cell Placement Rows (Metal 1: Layer 11)
    for y in range(150000, 1150000, 8000): # 8 um pitch standard cell rows
        out.extend(add_box(11, 0, 140000, y, 1160000, y + 4000))

    # 6. I/O Wirebond Pads (Layer 70: 70 um x 70 um)
    # North & South edges
    for x in range(120000, 1180000, 100000):
        out.extend(add_box(70, 0, x, 15000, x + 70000, 85000))
        out.extend(add_box(70, 0, x, 1215000, x + 70000, 1285000))
    # East & West edges
    for y in range(120000, 1180000, 100000):
        out.extend(add_box(70, 0, 15000, y, 85000, y + 70000))
        out.extend(add_box(70, 0, 1215000, y, 1285000, y + 70000))

    # ENDSTR (record 0x07, type 0x00)
    out.extend(gds_record(0x07, 0x00))

    # ENDLIB (record 0x04, type 0x00)
    out.extend(gds_record(0x04, 0x00))

    with open(filename, "wb") as f:
        f.write(out)

    print(f"[SUCCESS] Generated binary GDSII layout stream: {filename} ({len(out)} bytes)")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    gds_dir = os.path.abspath(os.path.join(script_dir, "..", "gds"))
    os.makedirs(gds_dir, exist_ok=True)
    target_gds = os.path.join(gds_dir, "foc_controller_top.gds")
    write_gds(target_gds)
