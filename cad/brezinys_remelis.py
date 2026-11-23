import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from params import *

INK = "#111111"
DIM = "#1f4e9c"
HID = "#555555"
FILL = "#e6e9ee"

fig = plt.figure(figsize=(18, 11), dpi=150)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 360)
ax.set_ylim(0, 220)
ax.set_aspect("equal")
ax.axis("off")
fig.patch.set_facecolor("white")

ax.add_patch(Polygon([(2, 2), (358, 2), (358, 218), (2, 218)], closed=True, fill=False, lw=1.2, ec=INK))


def view(ox, oy, s, flip_v=True):
    def f(u, v):
        return (ox + u * s, oy - v * s) if flip_v else (ox + u * s, oy + v * s)
    return f


def body(pts, fill=FILL, hatch=None, lw=1.4):
    ax.add_patch(Polygon(pts, closed=True, fc=fill, ec=INK, lw=lw, hatch=hatch, joinstyle="round"))


def line(p, q, style="-", color=INK, lw=0.8):
    ax.plot([p[0], q[0]], [p[1], q[1]], linestyle=style, color=color, lw=lw, solid_capstyle="butt")


def hidden(p, q):
    line(p, q, style=(0, (3, 2)), color=HID, lw=0.8)


def center(p, q):
    line(p, q, style=(0, (10, 2, 2, 2)), color="#888888", lw=0.6)


def text(p, s, size=9, rot=0, ha="center", va="center", color=DIM, weight="normal"):
    ax.text(p[0], p[1], s, fontsize=size, rotation=rot, ha=ha, va=va, color=color, fontweight=weight)


def dim(p1, p2, off, label, tshift=0.0, size=9):
    p1, p2 = np.array(p1, float), np.array(p2, float)
    d = (p2 - p1) / np.linalg.norm(p2 - p1)
    n = np.array([-d[1], d[0]])
    sg = 1 if off >= 0 else -1
    q1, q2 = p1 + n * off, p2 + n * off
    line(p1 + n * sg * 0.8, q1 + n * sg * 1.5, color=DIM, lw=0.5)
    line(p2 + n * sg * 0.8, q2 + n * sg * 1.5, color=DIM, lw=0.5)
    ax.annotate("", xy=q2, xytext=q1, arrowprops=dict(arrowstyle="<|-|>", color=DIM, lw=0.7, shrinkA=0, shrinkB=0, mutation_scale=9))
    ang = math.degrees(math.atan2(d[1], d[0]))
    if ang > 90:
        ang -= 180
    if ang <= -90:
        ang += 180
    mid = (q1 + q2) / 2 + n * sg * 2.4 + d * tshift
    text(mid, label, size=size, rot=ang)


def leader(p, q, label, size=9, ha="left"):
    ax.annotate(label, xy=p, xytext=q, fontsize=size, color=DIM, ha=ha, va="center",
                arrowprops=dict(arrowstyle="-|>", color=DIM, lw=0.6, shrinkA=1, shrinkB=0, mutation_scale=8))


def title(p, s):
    text(p, s, size=10.5, ha="left", color=INK, weight="bold")


# ---------------- Front view ----------------
S = 1.5
F = view(30, 190, S)
R = RIB_H
front = [(0, 0), (OUTER_W, 0), (OUTER_W, RIB_FROM), (OUTER_W + R, RIB_FROM + RIB_RAMP), (OUTER_W + R, RIB_TO - RIB_RAMP),
         (OUTER_W, RIB_TO), (OUTER_W, LEG_H + END_R), (OUTER_W - LEG_T, LEG_H + END_R), (OUTER_W - LEG_T, BAR_H + GUSSET),
         (OUTER_W - LEG_T - GUSSET, BAR_H), (LEG_T + GUSSET, BAR_H), (LEG_T, BAR_H + GUSSET), (LEG_T, LEG_H + END_R),
         (0, LEG_H + END_R), (0, RIB_TO), (-R, RIB_TO - RIB_RAMP), (-R, RIB_FROM + RIB_RAMP), (0, RIB_FROM)]
body([F(*p) for p in front])
title(F(0, -14), "VAIZDAS IŠ PRIEKIO  (1:1)")

for mx in (CX - MAG_PITCH / 2, CX + MAG_PITCH / 2):
    a, b = mx - MAG_D / 2, mx + MAG_D / 2
    hidden(F(a, 0), F(a, MAG_DEPTH)); hidden(F(a, MAG_DEPTH), F(b, MAG_DEPTH)); hidden(F(b, MAG_DEPTH), F(b, 0))
for cx in (CX - CONE_PITCH / 2, CX + CONE_PITCH / 2):
    hidden(F(cx - CONE_D / 2, 0), F(cx - CONE_D / 4, 3)); hidden(F(cx - CONE_D / 4, 3), F(cx + CONE_D / 4, 3))
    hidden(F(cx + CONE_D / 4, 3), F(cx + CONE_D / 2, 0))
for x0 in (0, OUTER_W - LEG_T):
    hidden(F(x0, LEG_H - HOLE_D / 2), F(x0 + LEG_T, LEG_H - HOLE_D / 2))
    hidden(F(x0, LEG_H + HOLE_D / 2), F(x0 + LEG_T, LEG_H + HOLE_D / 2))
    center(F(x0 - 5, LEG_H), F(x0 + LEG_T + 5, LEG_H))
center(F(CX, 20), F(CX, LEG_H + 6))

line(F(-6, 45), F(-1, 45), lw=2); line(F(3, 45), F(9, 45), lw=2)
text(F(-8, 42.5), "A", size=11, color=INK, weight="bold"); text(F(11, 42.5), "A", size=11, color=INK, weight="bold")
line(F(CX, -4), F(CX, -0.5), lw=2); line(F(CX, BAR_H + 0.5), F(CX, BAR_H + 4), lw=2)
text(F(CX + 3, -4), "B", size=11, color=INK, weight="bold"); text(F(CX + 3, BAR_H + 5), "B", size=11, color=INK, weight="bold")

dim(F(0, 0), F(OUTER_W, 0), 7, "84")
dim(F(LEG_T, LEG_H + END_R), F(OUTER_W - LEG_T, LEG_H + END_R), -6, "80  (vidus = važiuoklės plotis)")
dim(F(0, LEG_H), F(0, 0), 12, "70  (iki skylės centro)")
dim(F(OUTER_W, 0), F(OUTER_W, BAR_H), 9, "7")
leader(F(LEG_T + 2.5, BAR_H + 2.5), F(18, 24), "kampo sustiprinimas 8 × 8")
leader(F(OUTER_W + R, 40), F(OUTER_W + 7, 40), "2 + briauna 1,5")

# ---------------- Side view ----------------
Sd = view(190, 190, S)
side = [(0, 0), (FLANGE_D, 0), (FLANGE_D, FLANGE_H), (LEG_W, FLANGE_H), (LEG_W, LEG_H)]
side += [(END_R + END_R * math.cos(t), LEG_H + END_R * math.sin(t)) for t in np.linspace(0, math.pi, 25)]
side += [(0, LEG_H)]
body([Sd(*p) for p in side])
title(Sd(0, -14), "IŠ ŠONO")
rib_side = [(END_R - RIB_W / 2, RIB_FROM), (END_R + RIB_W / 2, RIB_FROM), (END_R + RIB_W / 2, RIB_TO), (END_R - RIB_W / 2, RIB_TO)]
ax.add_patch(Polygon([Sd(*p) for p in rib_side], closed=True, fill=False, ec=INK, lw=0.7))
a, b = FLANGE_D / 2 - MAG_D / 2, FLANGE_D / 2 + MAG_D / 2
hidden(Sd(a, 0), Sd(a, MAG_DEPTH)); hidden(Sd(a, MAG_DEPTH), Sd(b, MAG_DEPTH)); hidden(Sd(b, MAG_DEPTH), Sd(b, 0))
ax.add_patch(Circle(Sd(END_R, LEG_H), HOLE_D / 2 * S, fc="white", ec=INK, lw=1))
center(Sd(-3, LEG_H), Sd(LEG_W + 3, LEG_H)); center(Sd(END_R, LEG_H - 5), Sd(END_R, LEG_H + 5))
dim(Sd(0, 0), Sd(FLANGE_D, 0), 7, "10")
dim(Sd(0, LEG_H + END_R), Sd(LEG_W, LEG_H + END_R), -6, "5")
dim(Sd(FLANGE_D, 0), Sd(FLANGE_D, FLANGE_H), 6, "4")
leader(Sd(END_R + 1.8, LEG_H + 1.8), Sd(16, LEG_H + 6), "R2,5")
leader(Sd(END_R + 0.8, LEG_H - 0.8), Sd(16, LEG_H - 6), "Ø2,2  M2 medvaržčiui")
leader(Sd(END_R + RIB_W / 2, 38), Sd(16, 38), "briauna 1,6 × 1,5")
leader(Sd(END_R, RIB_FROM), Sd(16, RIB_FROM + 4), "briauna nuo 15 mm,\nnuožulnus 5 mm perėjimas", size=8)

# ---------------- Top view ----------------
T = view(30, 32, S, flip_v=False)
body([T(0, 0), T(OUTER_W, 0), T(OUTER_W, FLANGE_D), T(0, FLANGE_D)])
for x0, x1 in ((-R, 0), (OUTER_W, OUTER_W + R)):
    body([T(x0, END_R - RIB_W / 2), T(x1, END_R - RIB_W / 2), T(x1, END_R + RIB_W / 2), T(x0, END_R + RIB_W / 2)], lw=0.8)
for mx in (CX - MAG_PITCH / 2, CX + MAG_PITCH / 2):
    ax.add_patch(Circle(T(mx, FLANGE_D / 2), MAG_D / 2 * S, fc="white", ec=INK, lw=1))
for cx in (CX - CONE_PITCH / 2, CX + CONE_PITCH / 2):
    ax.add_patch(Circle(T(cx, FLANGE_D / 2), CONE_D / 2 * S, fc="white", ec=INK, lw=1))
    ax.add_patch(Circle(T(cx, FLANGE_D / 2), CONE_D / 4 * S, fc="none", ec=INK, lw=0.6))
center(T(-4, FLANGE_D / 2), T(OUTER_W + 4, FLANGE_D / 2))
title(T(0, 21), "IŠ VIRŠAUS")
dim(T(CX - CONE_PITCH / 2, FLANGE_D / 2), T(CX + CONE_PITCH / 2, FLANGE_D / 2), 9.5, "36  kūgio angos Ø3,2 (gylis 3)")
dim(T(CX - MAG_PITCH / 2, FLANGE_D / 2), T(CX + MAG_PITCH / 2, FLANGE_D / 2), -12, "60  magnetai Ø6,2 × 3,2")
dim(T(0, 0), T(0, FLANGE_D), 6, "10")

# ---------------- Pad ----------------
P = view(182, 33, 2.0, flip_v=False)
body([P(0, 0), P(PAD_L, 0), P(PAD_L, PAD_W), P(0, PAD_W)])
ax.add_patch(Circle(P(5, PAD_W / 2), MAG_D / 2 * 2, fc="white", ec=INK, lw=1))
ax.add_patch(Circle(P(17, PAD_W / 2), CONE_D / 2 * 2 * 0.95, fc="#c9ced6", ec=INK, lw=1))
ax.add_patch(Circle(P(17, PAD_W / 2), CONE_D / 4 * 2, fc="none", ec=INK, lw=0.6))
title(P(0, 18), "PRIEŠPADELIS ×2  (iš apačios / iš šono, 2:1)")
dim(P(0, PAD_W), P(PAD_L, PAD_W), 4.5, "22")
dim(P(0, 0), P(0, PAD_W), 5, "12")
dim(P(5, 0), P(17, 0), -4, "12")

Q = view(182, 22, 2.0)
pad_side = [(0, 0), (PAD_L, 0), (PAD_L, PAD_T), (17 + 1.5, PAD_T), (17 + 0.8, PAD_T + PIN_H), (17 - 0.8, PAD_T + PIN_H), (17 - 1.5, PAD_T), (0, PAD_T)]
body([Q(*p) for p in pad_side])
a, b = 5 - MAG_D / 2, 5 + MAG_D / 2
hidden(Q(a, PAD_T), Q(a, PAD_T - MAG_DEPTH)); hidden(Q(a, PAD_T - MAG_DEPTH), Q(b, PAD_T - MAG_DEPTH)); hidden(Q(b, PAD_T - MAG_DEPTH), Q(b, PAD_T))
dim(Q(PAD_L, 0), Q(PAD_L, PAD_T), 5, "4")
leader(Q(17.8, PAD_T + 1.5), Q(25, PAD_T + 1.5), "kaištis 2,5", size=8)
text(Q(-1.5, 1), "prie salono ↑", size=8, color=INK, ha="right")

# ---------------- Section A-A ----------------
A = view(298, 180, 6.0, flip_v=False)
secA = [(0, 0), (LEG_W, 0), (LEG_W, LEG_T), (END_R + RIB_W / 2, LEG_T), (END_R + RIB_W / 2, LEG_T + RIB_H),
        (END_R - RIB_W / 2, LEG_T + RIB_H), (END_R - RIB_W / 2, LEG_T), (0, LEG_T)]
body([A(*p) for p in secA], fill="white", hatch="////")
title((272, 213), "PJŪVIS A–A  (6:1), briauna į išorę")
dim(A(0, 0), A(LEG_W, 0), -5, "5  (prie važiuoklės)")
dim(A(LEG_W, 0), A(LEG_W, LEG_T), -6, "2")
dim(A(END_R + RIB_W / 2, LEG_T), A(END_R + RIB_W / 2, LEG_T + RIB_H), -16.2, "1,5")
dim(A(END_R - RIB_W / 2, LEG_T + RIB_H), A(END_R + RIB_W / 2, LEG_T + RIB_H), 3, "1,6")

# ---------------- Section B-B ----------------
B = view(298, 158, 4.5)
secB = [(0, 0), (FLANGE_D, 0), (FLANGE_D, FLANGE_H), (LEG_W, FLANGE_H), (LEG_W, BAR_H), (0, BAR_H)]
body([B(*p) for p in secB], fill="white", hatch="////")
title((272, 165), "PJŪVIS B–B  (4,5:1), magnetai viršuje")
dim(B(0, BAR_H), B(LEG_W, BAR_H), -4, "5")
dim(B(0, BAR_H), B(FLANGE_D, BAR_H), -9.5, "10")
dim(B(0, BAR_H), B(0, 0), 5, "7")
dim(B(FLANGE_D, 0), B(FLANGE_D, FLANGE_H), 5, "4")

# ---------------- Title block ----------------
ax.add_patch(Polygon([(268, 4), (356, 4), (356, 44), (268, 44)], closed=True, fill=False, ec=INK, lw=1.2))
line((268, 34), (356, 34), lw=0.8)
text((312, 39), "U FORMOS SALONO RĖMELIS", size=11, color=INK, weight="bold")
notes = [
    "RGT EX86170 PRO Challenger 1/10, FPV salono laikiklis",
    "Medžiaga: PETG, 4 perimetrai, 40 % užpildas",
    "Spausdinti gulint galine plokštuma ant stalo, be atramų",
    "Tvirtinimas: 2× M2 medvaržtis, 4× magnetas Ø6 × 3 N52",
    "Matmenys mm, bendra tolerancija ±0,2",
    "Kiekis: rėmelis ×1, priešpadelis ×2",
]
for i, s in enumerate(notes):
    text((271, 30 - i * 4.4), s, size=8.2, ha="left", color=INK)

# ---------------- 3D view ----------------
ax3 = fig.add_axes([262 / 360, 46 / 220, 96 / 360, 58 / 220], projection="3d")
ax3.set_facecolor((1, 1, 1, 0))
faces = []


def extrude(poly, axis, a0, a1):
    def P3(p, a):
        x, y = p
        if axis == "x":
            return (a, x, -y)
        if axis == "y":
            return (x, a, -y)
    faces.append([P3(p, a0) for p in poly])
    faces.append([P3(p, a1) for p in poly])
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        faces.append([P3(p, a0), P3(q, a0), P3(q, a1), P3(p, a1)])


leg_prof = [(0, 0), (LEG_W, 0), (LEG_W, LEG_H)] + [(END_R + END_R * math.cos(t), LEG_H + END_R * math.sin(t)) for t in np.linspace(0, math.pi, 13)] + [(0, LEG_H)]
extrude(leg_prof, "x", 0, LEG_T)
extrude(leg_prof, "x", OUTER_W - LEG_T, OUTER_W)
extrude(secB, "x", 0, OUTER_W)
extrude([(LEG_T, BAR_H), (LEG_T + GUSSET, BAR_H), (LEG_T, BAR_H + GUSSET)], "y", 0, LEG_W)
extrude([(OUTER_W - LEG_T, BAR_H), (OUTER_W - LEG_T - GUSSET, BAR_H), (OUTER_W - LEG_T, BAR_H + GUSSET)], "y", 0, LEG_W)
rib_prof = [(END_R - RIB_W / 2, RIB_FROM + RIB_RAMP), (END_R + RIB_W / 2, RIB_FROM + RIB_RAMP), (END_R + RIB_W / 2, RIB_TO - RIB_RAMP), (END_R - RIB_W / 2, RIB_TO - RIB_RAMP)]
extrude(rib_prof, "x", -RIB_H, 0)
extrude(rib_prof, "x", OUTER_W, OUTER_W + RIB_H)
ax3.add_collection3d(Poly3DCollection(faces, facecolor="#b8c4d6", edgecolor="#2b2b2b", linewidths=0.25))
for mx in (CX - MAG_PITCH / 2, CX + MAG_PITCH / 2):
    t = np.linspace(0, 2 * math.pi, 40)
    ax3.plot(mx + MAG_D / 2 * np.cos(t), FLANGE_D / 2 + MAG_D / 2 * np.sin(t), np.zeros_like(t) + 0.05, color="k", lw=0.8)
for cx in (CX - CONE_PITCH / 2, CX + CONE_PITCH / 2):
    t = np.linspace(0, 2 * math.pi, 30)
    ax3.plot(cx + CONE_D / 2 * np.cos(t), FLANGE_D / 2 + CONE_D / 2 * np.sin(t), np.zeros_like(t) + 0.05, color="k", lw=0.8)
ax3.set_xlim(-4, OUTER_W + 4)
ax3.set_ylim(-20, 30)
ax3.set_zlim(-LEG_H - 4, 4)
ax3.set_box_aspect((OUTER_W + 8, 50, LEG_H + 8))
ax3.view_init(elev=24, azim=-58)
ax3.axis("off")
title((272, 108), "3D VAIZDAS")

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(out, exist_ok=True)
path = os.path.join(out, "brezinys_remelis.png")
fig.savefig(path, dpi=150, facecolor="white")
print(path)
