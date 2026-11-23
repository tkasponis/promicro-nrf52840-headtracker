import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from params import *
from manifold3d import Manifold
from remelis_3d import frame, pad, to_trimesh

here = os.path.dirname(os.path.abspath(__file__))
fr_m = frame()
pd_m = pad()
fr = to_trimesh(fr_m)
pd = to_trimesh(pd_m)


class Probe:
    def __init__(self, m):
        self.m = m

    def contains(self, pts):
        x, y, z = pts[0]
        cube = Manifold.cube([0.1, 0.1, 0.1], center=True).translate([x, y, z])
        return [(self.m ^ cube).volume() > 0.0009]

checks = [
    ("magneto įduba tuščia", Probe(fr_m), (CX - MAG_PITCH / 2, -1.0, FLANGE_D / 2), False),
    ("po magnetu lieka dugnas", Probe(fr_m), (CX - MAG_PITCH / 2, -3.6, FLANGE_D / 2), True),
    ("kūgio anga tuščia", Probe(fr_m), (CX - CONE_PITCH / 2, -1.0, FLANGE_D / 2), False),
    ("kojos skylė tuščia", Probe(fr_m), (LEG_T / 2, -LEG_H, END_R), False),
    ("aplink skylę medžiaga", Probe(fr_m), (LEG_T / 2, -LEG_H + 1.9, END_R), True),
    ("kairė briauna", Probe(fr_m), (-RIB_H / 2, -40, END_R), True),
    ("dešinė briauna", Probe(fr_m), (OUTER_W + RIB_H / 2, -40, END_R), True),
    ("po briauna nuožulna tuščia", Probe(fr_m), (-RIB_H + 0.1, -40, 0.3), False),
    ("kampo sustiprinimas", Probe(fr_m), (LEG_T + 2, -BAR_H - 2, 2.5), True),
    ("skersinio lentyna", Probe(fr_m), (CX, -2, FLANGE_D - 1), True),
    ("po lentyna tuščia", Probe(fr_m), (CX, -5.5, FLANGE_D - 1), False),
    ("vidus tarp kojų tuščias", Probe(fr_m), (CX, -40, 2.5), False),
    ("priešpadelio magneto įduba", Probe(pd_m), (PAD_MAG_X, PAD_W / 2, PAD_T - 1), False),
    ("priešpadelio kaištis", Probe(pd_m), (PAD_PIN_X, PAD_W / 2, PAD_T + PIN_H - 0.3), True),
]
bad = 0
for name, m, p, want in checks:
    got = bool(m.contains([p])[0])
    bad += got != want
    print(f"{'OK ' if got == want else 'BLOGAI'}  {name}")
print("Visi patikrinimai praėjo" if bad == 0 else f"{bad} patikrinimai nepraėjo")

fig = plt.figure(figsize=(16, 7), dpi=130)
fig.patch.set_facecolor("white")
light = np.array([0.4, -0.5, 0.75])
light /= np.linalg.norm(light)


def draw(ax, tm, elev, azim, title, box):
    n = tm.face_normals
    shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
    colors = np.c_[0.45 * shade + 0.2, 0.6 * shade + 0.2, 0.85 * shade + 0.1, np.ones_like(shade)]
    pc = Poly3DCollection(tm.vertices[tm.faces], facecolors=np.clip(colors, 0, 1), edgecolor="none")
    ax.add_collection3d(pc)
    lo, hi = tm.bounds
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, fontsize=11)
    ax.axis("off")


fr_print = fr.copy()
fr_upright = fr.copy()
fr_upright.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
draw(fig.add_subplot(1, 3, 1, projection="3d"), fr_upright, 18, -60, "Rėmelis kaip stovės mašinoje", None)
draw(fig.add_subplot(1, 3, 2, projection="3d"), fr_print, 55, -70, "Kaip guli ant A1 Mini stalo", None)
draw(fig.add_subplot(1, 3, 3, projection="3d"), pd, 35, -55, "Priešpadelis ×2 (spausdinimo padėtis)", None)
plt.tight_layout()
path = os.path.join(here, "out", "perziura_3d.png")
fig.savefig(path, facecolor="white")
print(path)
