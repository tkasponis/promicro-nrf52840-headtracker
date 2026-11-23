"""U formos salono remelis ir prieSpadeliai Bambu Lab A1 Mini spausdintuvui.

Spausdinimo koordinates: X = remelio ilgis (skersai masinos), Y = aukstis
(Y = 0 yra skersinio virsus, kojos eina i -Y), Z = gylis. Z = 0 yra galine
plokstuma, kuri guli ant stalo.
"""
import math
import os

import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule, Manifold

from params import *

SEG = 64
EPS = 0.05
BED = 180.0


def section(points):
    return CrossSection([points], FillRule.NonZero)


def extrude_wv_along_u(points_wv, u0, u1):
    """Profilis (w, v) plokstumoje, istemptas per remelio ilgi u."""
    cs = section([(-w, -v) for w, v in points_wv])
    return cs.extrude(u1 - u0).rotate([0, 90, 0]).translate([u0, 0, 0])


def extrude_uv_along_w(points_uv, w0, w1):
    cs = section([(u, -v) for u, v in points_uv])
    return cs.extrude(w1 - w0).translate([0, 0, w0])


def extrude_uw_along_v(points_uw, v0, v1):
    cs = section(points_uw)
    return cs.extrude(v1 - v0).rotate([90, 0, 0]).translate([0, -v0, 0])


def cyl_along_x(d, x0, x1, y, z):
    return Manifold.cylinder(x1 - x0, d / 2, circular_segments=SEG).rotate([0, 90, 0]).translate([x0, y, z])


def pocket_down_from_top(d_top, d_bottom, depth, u, w):
    """Anga skersinio virsuje (Y = 0), einanti zemyn i -Y."""
    c = Manifold.cylinder(depth + EPS, d_top / 2 + (d_top - d_bottom) / 2 * EPS / depth, d_bottom / 2, circular_segments=SEG)
    return c.rotate([90, 0, 0]).translate([u, EPS, w])


def leg_profile():
    pts = [(0, 0), (LEG_W, 0), (LEG_W, LEG_H)]
    for t in np.linspace(0, math.pi, 33)[1:-1]:
        pts.append((END_R + END_R * math.cos(t), LEG_H + END_R * math.sin(t)))
    pts.append((0, LEG_H))
    return pts


def rib(left):
    ramp = [(EPS * 6, RIB_FROM), (-RIB_H, RIB_FROM + RIB_RAMP), (-RIB_H, RIB_TO - RIB_RAMP), (EPS * 6, RIB_TO)]
    chamfer = [(EPS * 6, END_R - RIB_W / 2 - RIB_H), (EPS * 6, END_R + RIB_W / 2),
               (-RIB_H, END_R + RIB_W / 2), (-RIB_H, END_R - RIB_W / 2)]
    if not left:
        ramp = [(OUTER_W - u, v) for u, v in ramp]
        chamfer = [(OUTER_W - u, w) for u, w in chamfer]
    a = extrude_uv_along_w(ramp, 0, LEG_W)
    b = extrude_uw_along_v(chamfer, RIB_FROM - 1, RIB_TO + 1)
    return a ^ b


def frame():
    legs = extrude_wv_along_u(leg_profile(), 0, LEG_T) + extrude_wv_along_u(leg_profile(), OUTER_W - LEG_T, OUTER_W)
    bar = extrude_wv_along_u([(0, 0), (FLANGE_D, 0), (FLANGE_D, FLANGE_H), (LEG_W, FLANGE_H), (LEG_W, BAR_H), (0, BAR_H)], 0, OUTER_W)
    g = BAR_H - EPS
    gussets = extrude_uv_along_w([(LEG_T - EPS, g), (LEG_T + GUSSET, g), (LEG_T - EPS, BAR_H + GUSSET)], 0, LEG_W)
    gussets += extrude_uv_along_w([(OUTER_W - LEG_T + EPS, g), (OUTER_W - LEG_T - GUSSET, g), (OUTER_W - LEG_T + EPS, BAR_H + GUSSET)], 0, LEG_W)
    body = legs + bar + gussets + rib(True) + rib(False)

    body -= cyl_along_x(HOLE_D, -RIB_H - 2, OUTER_W + RIB_H + 2, -LEG_H, END_R)
    for mx in (CX - MAG_PITCH / 2, CX + MAG_PITCH / 2):
        body -= pocket_down_from_top(MAG_D, MAG_D, MAG_DEPTH, mx, FLANGE_D / 2)
    for cx in (CX - CONE_PITCH / 2, CX + CONE_PITCH / 2):
        body -= pocket_down_from_top(CONE_D, CONE_D / 2, CONE_DEPTH, cx, FLANGE_D / 2)
    return body


def pad():
    """Spausdinama klijuojama puse zemyn: magnetas ir kaistis zvelgia i virsu."""
    box = Manifold.cube([PAD_L, PAD_W, PAD_T])
    box -= Manifold.cylinder(MAG_DEPTH + EPS, MAG_D / 2, circular_segments=SEG).translate([PAD_MAG_X, PAD_W / 2, PAD_T - MAG_DEPTH])
    pin = Manifold.cylinder(PIN_H + EPS, PIN_D_BASE / 2, PIN_D_TIP / 2, circular_segments=SEG)
    return box + pin.translate([PAD_PIN_X, PAD_W / 2, PAD_T - EPS])


def to_trimesh(m):
    mesh = m.to_mesh()
    verts = np.asarray(mesh.vert_properties)[:, :3]
    faces = np.asarray(mesh.tri_verts)
    return trimesh.Trimesh(vertices=verts, faces=faces, process=True)


def place_on_bed(tm, x, y):
    lo = tm.bounds[0]
    tm.apply_translation([x - lo[0], y - lo[1], -lo[2]])
    return tm


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "out")
    os.makedirs(out, exist_ok=True)

    parts = {
        "remelis_U": place_on_bed(to_trimesh(frame()), 20, 40),
        "priespadelis_1": place_on_bed(to_trimesh(pad()), 20, 15),
        "priespadelis_2": place_on_bed(to_trimesh(pad()), 60, 15),
    }

    ok = True
    for name, tm in parts.items():
        size = tm.bounds[1] - tm.bounds[0]
        fits = tm.bounds[1][0] <= BED and tm.bounds[1][1] <= BED and size[2] <= BED
        print(f"{name:16s} dydis {size[0]:6.2f} x {size[1]:6.2f} x {size[2]:5.2f} mm  "
              f"tūris {tm.volume / 1000:5.2f} cm3  uždaras={tm.is_watertight}  telpa={fits}")
        ok &= tm.is_watertight and fits
        tm.export(os.path.join(out, f"{name}.stl"))

    scene = trimesh.Scene()
    for name, tm in parts.items():
        scene.add_geometry(tm, node_name=name, geom_name=name)
    scene.export(os.path.join(out, "remelis_A1mini.3mf"))
    print("OK" if ok else "KLAIDA")


if __name__ == "__main__":
    main()
