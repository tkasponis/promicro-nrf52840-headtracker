"""DJI Avata akiniu headtracker dezute: Pro Micro + MPU6050 + RP1.

Vidinis bloko guolis 22.6 x 20.6 x 35.6 mm (vartotojo 22 x 20 x 35 mm
plius tarpas, kad po FDM susitraukimo blokas iliptu).
Sonine vielu juosta 2.4 mm atskirta briauna, kad laidai nebutu prispausti.

Trys dangciai:
  A begielis  – dangtis nuslenka i prieki, be lanksciu fiksatorių
  B vyriai    – dangtis verciasi ant 1.75 mm gijos asies
  C stalcius  – blokas istraukiamas pro gala, dureles laiko ta pati gija

Koordinates: X i prieki (USB), X didelis = galas (antena).
Y centruotas, teigiama puse yra vielu juosta. Z aukstyn, Z=0 dugnas.
"""

import math
import os

import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule, Manifold
from PIL import Image, ImageDraw, ImageFont

SEG = 40
N_CORNER = 8

BLOCK_W = 22.6
BLOCK_L = 35.6
BLOCK_H = 20.6
RIB_W = 0.9
RIB_H = 1.6
LANE = 2.4
W = 2.6
R = 2.0
FLOOR = 2.0
LID_T = 2.0
SEAM = 0.30

CAV_W = BLOCK_W + RIB_W + LANE
CAV_L = BLOCK_L
CAV_H = BLOCK_H
OX = CAV_L + 2 * W
OY = CAV_W + 2 * W
OZ = FLOOR + CAV_H

Y_IN = CAV_W / 2
BLOCK_Y0 = -Y_IN
BLOCK_Y1 = BLOCK_Y0 + BLOCK_W
BLOCK_YC = (BLOCK_Y0 + BLOCK_Y1) / 2
USB_ZC = FLOOR + 6.4

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


def box(x0, y0, z0, x1, y1, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_x(r, x0, x1, y, z):
    return Manifold.cylinder(x1 - x0, r, circular_segments=SEG).rotate([0, 90, 0]).translate([x0, y, z])


def cyl_y(r, y0, y1, x, z):
    return Manifold.cylinder(y1 - y0, r, circular_segments=SEG).rotate([-90, 0, 0]).translate([x, y0, z])


def rounded_rect(x0, x1, y0, y1, r, n=N_CORNER):
    hw = (x1 - x0) / 2
    hh = (y1 - y0) / 2
    r = min(r, hw - 0.05, hh - 0.05)
    cx = (x0 + x1) / 2
    cy = (y0 + y1) / 2
    spans = (
        (hw - r, hh - r, 0.0, math.pi / 2),
        (-hw + r, hh - r, math.pi / 2, math.pi),
        (-hw + r, -hh + r, math.pi, 3 * math.pi / 2),
        (hw - r, -hh + r, 3 * math.pi / 2, 2 * math.pi),
    )
    pts = []
    for ox, oy, a0, a1 in spans:
        for i in range(n):
            a = a0 + (a1 - a0) * i / n
            pts.append((cx + ox + r * math.cos(a), cy + oy + r * math.sin(a)))
    return pts


def extrude(pts, z0, z1):
    return CrossSection([pts], FillRule.NonZero).extrude(z1 - z0).translate([0, 0, z0])


def stadium_x(x0, x1, y, z, w, h):
    r = h / 2
    inset = w / 2 - r
    return (
        cyl_x(r, x0, x1, y - inset, z)
        + cyl_x(r, x0, x1, y + inset, z)
        + box(x0, y - inset, z - r, x1, y + inset, z + r)
    )


def u_slot_x(x0, x1, y, z_bot, z_top, w):
    """Vertikali U ispjova, atvira i virsu, apvaliu dugnu."""
    r = w / 2
    return cyl_x(r, x0, x1, y, z_bot + r) + box(x0, y - r, z_bot + r, x1, y + r, z_top)


def mesh_of(m):
    raw = m.to_mesh()
    v = np.asarray(raw.vert_properties)[:, :3].copy()
    f = np.asarray(raw.tri_verts).copy()
    tm = trimesh.Trimesh(vertices=v, faces=f, process=False)
    tm.remove_unreferenced_vertices()
    return tm


def bounds(m):
    tm = mesh_of(m)
    return tm.bounds[0], tm.bounds[1]


def probe(m, x, y, z, s=0.35):
    bit = Manifold.cube([s, s, s], center=True).translate([float(x), float(y), float(z)])
    return (m ^ bit).volume() > s ** 3 * 0.35


def open_tub():
    """Vonia: atviras virsus, USB priekyje, antenos U gale, vielu briauna."""
    outer = extrude(rounded_rect(0, OX, -OY / 2, OY / 2, R), 0, OZ)
    inner = extrude(
        rounded_rect(W, OX - W, -Y_IN, Y_IN, 0.6),
        FLOOR,
        OZ + 2,
    )
    body = outer - inner
    rib = box(W + 1.2, BLOCK_Y1, FLOOR - 0.05, OX - W - 1.2, BLOCK_Y1 + RIB_W, FLOOR + RIB_H)
    body = body + rib
    usb = stadium_x(-1, W + 0.6, BLOCK_YC, USB_ZC, 10.5, 6.0)
    body = body - usb
    ant = u_slot_x(OX - W - 0.4, OX + 1, BLOCK_YC, OZ - 7.2, OZ + 2, 4.4)
    body = body - ant
    return body


def rail_void(sign):
    """Begio ispjova sonineje sienoje. sign = +1 (vielu puse) arba -1."""
    y_in = sign * Y_IN

    def y_at(t0, t1):
        a = y_in + sign * t0
        b = y_in + sign * t1
        return (a, b) if a < b else (b, a)

    x_stop = OX - W - 0.6
    lip_t = 1.18
    slot_t0, slot_t1 = 1.18, 1.80
    ch_t0, ch_t1 = 0.12, 1.80
    # Lietimos tikrinimas daromas cia, kad keiciant skaicius matytusi klaida.
    assert slot_t1 <= W - 0.75, "per plona isorine sienele virs begio"
    assert ch_t0 < 0.30 and slot_t0 <= lip_t + 0.02

    ch = box(*_xyz(0.0, x_stop, *y_at(ch_t0, ch_t1), 19.90, 21.00))
    web = box(*_xyz(0.0, x_stop, *y_at(slot_t0, slot_t1), 19.90, OZ + 3))
    return ch + web, lip_t, x_stop


def _xyz(x0, x1, y0, y1, z0, z1):
    return (min(x0, x1), min(y0, y1), min(z0, z1), max(x0, x1), max(y0, y1), max(z0, z1))


def lid_plate():
    z0 = OZ + SEAM
    plate = extrude(rounded_rect(0, OX, -OY / 2, OY / 2, R), z0, z0 + LID_T)
    notch = box(-1, -5.2, z0 - 1, 2.4, 5.2, z0 + LID_T + 1)
    return plate - notch


def slide_foot(sign, x0, x1):
    y_in = sign * Y_IN

    def y_at(t0, t1):
        a = y_in + sign * t0
        b = y_in + sign * t1
        return (min(a, b), max(a, b))

    bar = box(*_xyz(x0, x1, *y_at(0.30, 1.68), 20.10, 20.78))
    web = box(*_xyz(x0, x1, *y_at(1.32, 1.68), 20.70, OZ + SEAM + 0.08))
    return bar + web


# A variantas: kapsule su snapeliu. Mazas X = USB priekis, didelis X = antena.
# Begeliai lieka tie patys, tik korpusas ilgesnis del kistuko guolio.
POD_XF = 17.4          # virsutinio kuno priekis
POD_BAY0 = POD_XF + W  # bloko priekis
POD_BAY1 = POD_BAY0 + BLOCK_L
POD_XR = POD_BAY1 + W
POD_R = 5.8
POD_YT = Y_IN + W
POD_YB = POD_YT + 1.7
POD_BELLY = POD_YT + 2.85
POD_LID_T = 1.8
POD_SEAM = 0.28


def cyl_z(r, z0, z1, x, y):
    return Manifold.cylinder(z1 - z0, r, circular_segments=SEG).translate([x, y, z0])


def pod_shell():
    """Pilvas platesnis už padą ir už viršų, todėl šonas yra švelnus išlinkis."""
    upper = extrude(rounded_rect(POD_XF, POD_XR, -POD_YT, POD_YT, POD_R), 15.6, OZ)
    base = extrude(rounded_rect(POD_XF - 0.2, POD_XR + 1.4, -POD_YB, POD_YB, 7.2), 0.0, 1.7)
    belly = extrude(rounded_rect(POD_XF, POD_XR, -POD_BELLY, POD_BELLY, 8.6), 6.2, 8.6)
    neck = extrude(rounded_rect(POD_XF, POD_XR, -POD_YT, POD_YT, POD_R), 15.0, 16.4)
    return upper + Manifold.batch_hull([base, belly, neck])


def pod_snout():
    """Siauras apvalus snapelis. Kištukas gula į lovį, laidas išeina per priekinę lūpą."""
    tip = extrude(rounded_rect(0.0, 3.4, -10.6, 10.6, 5.2), 0.0, 11.0)
    root = extrude(rounded_rect(POD_XF - 2.4, POD_XF + 5.0, -POD_YB, POD_YB, 6.4), 0.0, 1.8)
    crown = extrude(rounded_rect(POD_XF - 1.5, POD_XF + 2.4, -11.2, 11.2, 5.4), 9.4, 14.8)
    return Manifold.batch_hull([tip, root, crown])


def front_rake():
    """Vidurinė priekinė siena krenta atgal. Šonai su bėgeliais lieka sveiki."""
    front = box(POD_XF - 3, -9.0, 15.2, POD_XF + 0.3, 9.0, OZ + 4)
    back = box(POD_XF + 7.4, -9.0, OZ - 0.15, POD_XF + 9.2, 9.0, OZ + 4)
    return Manifold.batch_hull([front, back])


def pod_rails(body):
    x_stop = POD_XR - W - 0.55
    lip_t = 1.18
    slot_t0, slot_t1 = 1.18, 1.80
    ch_t0, ch_t1 = 0.12, 1.80
    assert slot_t1 <= W - 0.75
    for sign in (1, -1):
        y_in = sign * Y_IN

        def y_at(t0, t1, y_in=y_in, sign=sign):
            a = y_in + sign * t0
            b = y_in + sign * t1
            return (min(a, b), max(a, b))

        y0, y1 = y_at(ch_t0, ch_t1)
        ch = box(POD_XF - 0.3, y0, 19.90, x_stop, y1, 21.00)
        y0, y1 = y_at(slot_t0, slot_t1)
        web = box(POD_XF - 0.3, y0, 19.90, x_stop, y1, OZ + 3)
        body = body - (ch + web)
    return body


def pod_lid():
    z0 = OZ + POD_SEAM
    z1 = z0 + POD_LID_T
    rear = extrude(
        rounded_rect(POD_XF + 0.4, POD_XR - 0.2, -(POD_YT - 0.45), POD_YT - 0.45, 7.4),
        z0, z1,
    )
    nose = extrude(rounded_rect(POD_XF - 6.6, POD_XF + 2.4, -8.2, 8.2, 4.0), z0, z1)
    plate = Manifold.batch_hull([rear, nose])
    plate = plate - cyl_z(4.6, z0 - 1, z1 + 1, POD_XF - 9.6, 0.0)
    feet = (
        slide_foot(1, POD_XF + 0.8, POD_XR - W - 1.25)
        + slide_foot(-1, POD_XF + 0.8, POD_XR - W - 1.25)
    )
    lid = plate + feet
    lid = lid - u_slot_x(POD_XR - 12.0, POD_XR + 2.5, BLOCK_YC, z0 - 0.3, z1 + 1, 5.6)
    return lid


def variant_a():
    body = pod_shell() + pod_snout()
    body = body - front_rake()
    inner = extrude(rounded_rect(POD_BAY0, POD_BAY1, -Y_IN, Y_IN, 0.6), FLOOR, OZ + 3)
    body = body - inner
    # Apvalus lovys, atviras iš viršaus, kad bloką su kištuku nuleistum.
    body = body - extrude(rounded_rect(2.8, POD_BAY0 + 0.45, -8.4, 8.4, 3.4), FLOOR, 16.4)
    body = body - u_slot_x(-0.8, 4.8, 0.0, 4.2, 16.4, 7.6)
    # Žema briauna prieš plokštę, kad 18 mm lenta neįlįstų į lovį.
    body = body + box(POD_BAY0 - 1.55, BLOCK_Y0 + 0.25, FLOOR, POD_BAY0 - 0.08, BLOCK_Y1 - 0.2, FLOOR + 0.85)
    rib = box(
        POD_BAY0 + 1.4, BLOCK_Y1, FLOOR - 0.05,
        POD_BAY1 - 1.6, BLOCK_Y1 + RIB_W, FLOOR + RIB_H,
    )
    body = body + rib
    body = pod_rails(body)
    body = body - u_slot_x(POD_XR - W - 0.45, POD_XR + 1.2, BLOCK_YC, 11.2, OZ + 2, 5.4)
    # Plona linija ten, kur pilvas susiaurėja į dangčio apykaklę.
    band = extrude(rounded_rect(POD_XF + 1.0, POD_XR - 0.4, -24, 24, 6.0), 14.85, 15.55)
    keep = extrude(
        rounded_rect(POD_XF - 0.4, POD_XR + 0.6, -(POD_YT - 0.55), POD_YT - 0.55, 7.6),
        14.65, 15.75,
    )
    body = body - (band - keep)
    # Velcro zonos žymės abiejuose šonuose, dugnas lieka plokščias.
    for x0 in (POD_BAY0 + 3.2, POD_BAY1 - 6.4):
        for sign in (1, -1):
            y_face = sign * POD_YB
            y_in = y_face - sign * 1.15
            body = body - box(
                x0, min(y_face, y_in), 0.35,
                x0 + 1.5, max(y_face, y_in), 2.15,
            )
    return body, pod_lid()


def knuckle(x0, x1, radius):
    return cyl_x(radius, x0, x1, -OY / 2, OZ)


def teardrop_x(r, x0, x1, y, z, point_down):
    hole = cyl_x(r, x0, x1, y, z)
    # Stogo atsarga, kad horizontalios skyles tiltas neuzgriutu.
    rise = r * 0.95
    if point_down:
        roof = box(x0, y - r * 0.55, z - rise, x1, y + r * 0.55, z + 0.2)
    else:
        roof = box(x0, y - r * 0.55, z - 0.2, x1, y + r * 0.55, z + rise)
    return hole + roof


AXIS_Y = -OY / 2 - 1.55
AXIS_Z = OZ - 0.2


def variant_b():
    body = open_tub()
    k_r = 2.15
    gap = 0.42
    spans_body = ((2.6, 11.2), (OX - 11.2, OX - 2.6))
    span_lid = (11.2 + gap, OX - 11.2 - gap)
    # Korpuse ispjova, kad dangcio narvelis suktusi neliesdamas sienos.
    body = body - cyl_x(k_r + 0.34, span_lid[0] - 0.2, span_lid[1] + 0.2, AXIS_Y, AXIS_Z)
    for a, b in spans_body:
        body = body + cyl_x(k_r, a, b, AXIS_Y, AXIS_Z)
    body = body - teardrop_x(1.28, 0.4, OX - 0.4, AXIS_Y, AXIS_Z, point_down=False)
    lid = lid_plate()
    # Plokštė neužlenda ant korpuso narvelių, viduryje lieka savo narvelis.
    for a, b in spans_body:
        lid = lid - cyl_x(k_r + 0.28, a - 0.25, b + 0.25, AXIS_Y, AXIS_Z)
    lid = lid + cyl_x(k_r, span_lid[0], span_lid[1], AXIS_Y, AXIS_Z)
    lid = lid - teardrop_x(1.28, span_lid[0] - 0.15, span_lid[1] + 0.15, AXIS_Y, AXIS_Z, point_down=True)
    # Nago ispjova desineje, kad dangti butu uz ka pakelti.
    lid = lid - box(OX * 0.55, OY / 2 - 1.4, OZ + SEAM - 0.2, OX * 0.55 + 8, OY / 2 + 1, OZ + SEAM + 1.3)
    return body, lid


def closed_shell():
    top = 2.2
    height = OZ + top
    outer = extrude(rounded_rect(0, OX, -OY / 2, OY / 2, R), 0, height)
    inner = extrude(rounded_rect(W, OX - W, -Y_IN, Y_IN, 0.6), FLOOR, OZ)
    # Galas atviras: blokas isstumiamas atgal.
    inner = inner + box(OX - W - 0.2, -Y_IN, FLOOR, OX + 1, Y_IN, OZ)
    body = outer - inner
    body = body + box(W + 1.2, BLOCK_Y1, FLOOR - 0.05, OX - W - 8, BLOCK_Y1 + RIB_W, FLOOR + RIB_H)
    body = body - stadium_x(-1, W + 0.6, BLOCK_YC, USB_ZC, 10.5, 6.0)
    body = body - u_slot_x(OX - 12, OX + 1, BLOCK_YC, OZ - 7.2, height + 1, 4.6)
    return body, height


def variant_c():
    body, height = closed_shell()
    g_x0, g_x1 = OX - 2.50, OX - 0.28
    g_deep = 1.30
    for sign in (1, -1):
        y0 = sign * Y_IN - (g_deep if sign < 0 else 0.15)
        y1 = sign * Y_IN + (0.15 if sign < 0 else g_deep)
        body = body - box(g_x0, min(y0, y1), 1.15, g_x1, max(y0, y1), height + 8)

    # Ausis kyla pro lubu plysi. Kaistis eina is galo i prieki, per ausi ir stulpeli.
    ear_y0, ear_y1 = -10.4, -4.6
    hy = (ear_y0 + ear_y1) / 2
    hz = height + 2.7
    body = body - box(g_x0 - 0.2, ear_y0 - 0.45, OZ - 0.5, OX + 0.4, ear_y1 + 0.45, height + 0.2)
    lug_x0, lug_x1 = OX - 9.4, OX - 4.8
    body = body + box(lug_x0, ear_y0 + 0.4, height - 0.05, lug_x1, ear_y1 - 0.4, height + 5.5)
    body = body - cyl_x(1.15, lug_x0 - 0.3, lug_x1 + 0.25, hy, hz)

    door = box(
        g_x0 + 0.18, -(Y_IN + g_deep - 0.24), 2.18,
        g_x1 - 0.18, Y_IN + g_deep - 0.24, OZ - 0.25,
    )
    door = door - u_slot_x(g_x0 - 1, g_x1 + 1, BLOCK_YC, OZ - 7.4, height + 6, 5.4)
    door = door + box(g_x0 + 0.18, ear_y0, OZ - 0.9, g_x1 - 0.18, ear_y1, height + 5.1)
    door = door - cyl_x(1.15, g_x0 - 0.15, g_x1 + 0.35, hy, hz)
    # Pirsto lupa uz korpuso.
    door = door + box(g_x1 - 0.55, 2.4, 7.2, OX + 1.6, 10.2, 13.2)
    return body, door, hy, hz


def service_pin():
    """C variantui: galvute ant stalo, kotas i virsu. Skyle 2.3 mm, kotas 1.7 mm."""
    shaft = Manifold.cylinder(7.4, 0.85, circular_segments=32)
    head = Manifold.cylinder(1.6, 2.2, circular_segments=32).translate([0, 0, -1.6])
    return shaft + head


def to_trimesh(m):
    return mesh_of(m)


def on_bed(tm):
    out = tm.copy()
    lo = out.bounds[0]
    out.apply_translation([-lo[0], -lo[1], -lo[2]])
    return out


def rot(tm, axis, angle):
    out = tm.copy()
    m = trimesh.transformations.rotation_matrix(angle, axis, out.bounds.mean(axis=0))
    out.apply_transform(m)
    return out


def check(name, m, point, want_solid):
    got = probe(m, *point)
    ok = got == want_solid
    print(f"  {'OK ' if ok else 'BLOGAI'}  {name}")
    return ok


def validate(a_body, a_lid, b_body, b_lid, c_body, c_door):
    print("Tikrinimas")
    ok = True
    samples = [
        ("A ertme tuscia", a_body, ((POD_BAY0 + POD_BAY1) / 2, BLOCK_YC, FLOOR + 8), False),
        ("A dugnas vientisas", a_body, ((POD_BAY0 + POD_BAY1) / 2, 0, 1.0), True),
        ("A kistuko lovys", a_body, (POD_BAY0 - 8, BLOCK_YC, FLOOR + 5), False),
        ("A laido isejimas", a_body, (1.6, 0.0, 8.0), False),
        ("A lupa laiko kistuka", a_body, (1.6, 6.4, 6.2), True),
        ("A antenos ispjova", a_body, (POD_XR - 1.0, BLOCK_YC, OZ - 3.0), False),
        ("A begio kanalas tuscias", a_body, (POD_BAY0 + 16, Y_IN + 0.7, 20.4), False),
        ("A begelio lupos medzaga", a_body, (POD_BAY0 + 16, Y_IN + 0.5, 21.5), True),
        ("A vielu juosta tuscia", a_body, ((POD_BAY0 + POD_BAY1) / 2, Y_IN - 1.1, FLOOR + 4), False),
        ("A briauna laiko bloka", a_body, ((POD_BAY0 + POD_BAY1) / 2, BLOCK_Y1 + RIB_W / 2, FLOOR + 0.6), True),
        ("A virs snapelio tuscia", a_body, (8.0, 0.0, 18.5), False),
        ("B ertme tuscia", b_body, (OX / 2, BLOCK_YC, FLOOR + 8), False),
        ("B vyrio skyle", b_body, (7.0, AXIS_Y, AXIS_Z), False),
        ("B vyrio kunas", b_body, (7.0, AXIS_Y, AXIS_Z + 1.45), True),
        ("C ertme tuscia", c_body, (OX / 2, BLOCK_YC, FLOOR + 8), False),
        ("C virsus vientisas", c_body, (OX / 2, BLOCK_YC, OZ + 1.0), True),
    ]
    for item in samples:
        ok &= check(*item)

    clash_a = (a_body ^ a_lid).volume()
    clash_b = (b_body ^ b_lid).volume()
    clash_c = (c_body ^ c_door).volume()
    print(f"  A sankirta {clash_a:.2f} mm3")
    print(f"  B sankirta {clash_b:.2f} mm3")
    print(f"  C sankirta {clash_c:.2f} mm3")
    ok &= clash_a < 1.0 and clash_b < 1.0 and clash_c < 1.0

    for name, part in (("A korpusas", a_body), ("A dangtis", a_lid), ("B korpusas", b_body),
                       ("B dangtis", b_lid), ("C korpusas", c_body), ("C dureles", c_door)):
        tm = to_trimesh(part)
        size = tm.bounds[1] - tm.bounds[0]
        grams = abs(tm.volume) / 1000 * 1.24
        print(f"  {name:12s} {size[0]:5.1f} x {size[1]:5.1f} x {size[2]:4.1f} mm   {grams:4.1f} g")
        ok &= tm.volume > 0 and len(tm.faces) > 100
    return ok


def export_stl(name, tm):
    path = os.path.join(OUT, name)
    parked = on_bed(tm)
    parked.export(path)
    parked.export(os.path.splitext(path)[0] + ".3mf")
    return path


# --- vaizdas -----------------------------------------------------------------

def _norm(v):
    n = np.linalg.norm(v)
    return v / n if n else v


def look_at(eye, target):
    forward = _norm(target - eye)
    right = _norm(np.cross(forward, np.array([0.0, 0.0, 1.0])))
    up = np.cross(right, forward)
    return right, up, forward


def shade_faces(normals, base):
    n = normals / (np.linalg.norm(normals, axis=1, keepdims=True) + 1e-9)
    l1 = _norm(np.array([0.25, -0.55, 0.80]))
    l2 = _norm(np.array([-0.70, 0.35, 0.40]))
    lambert = np.clip(n @ l1, 0, 1)
    fill = np.clip(n @ l2, 0, 1)
    sky = np.clip(n[:, 2] * 0.5 + 0.5, 0, 1)
    light = 0.42 + 0.50 * lambert + 0.16 * fill
    light = light * (0.88 + 0.12 * sky)
    col = base * light[:, None]
    spec = (np.clip(lambert, 0, 1) ** 18) * 0.22
    return np.clip(col + spec[:, None], 0, 1), n


def raster(meshes, w, h, elev=24, azim=-52, margin=0.78):
    """meshes: list of (trimesh, rgb ndarray shape 3)."""
    all_v = np.vstack([tm.vertices for tm, _ in meshes])
    center = (all_v.min(0) + all_v.max(0)) / 2
    radius = np.linalg.norm(all_v - center, axis=1).max()
    elev_r, az_r = math.radians(elev), math.radians(azim)
    eye = center + radius * 3.15 * np.array([
        math.cos(elev_r) * math.cos(az_r),
        math.cos(elev_r) * math.sin(az_r),
        math.sin(elev_r),
    ])
    right, up, forward = look_at(eye, center)

    shaded = []
    for tm, base in meshes:
        col, n = shade_faces(tm.face_normals, np.asarray(base, float))
        shaded.append((tm, col, n))

    # Projekcija, kad objektas uzimtu kadra.
    proj = []
    for tm, col, n in shaded:
        p = tm.vertices - eye
        cam = np.column_stack([p @ right, p @ up, p @ forward])
        proj.append(cam)
    P = np.vstack(proj)
    z = np.clip(P[:, 2], 1e-3, None)
    xy = P[:, :2] / z[:, None]
    span = max(xy[:, 0].max() - xy[:, 0].min(), xy[:, 1].max() - xy[:, 1].min())
    focal = (min(w, h) * margin) / span
    origin = np.array([w / 2, h / 2])

    img = np.zeros((h, w, 3), np.float32)
    grad = np.linspace(0.93, 0.80, h, dtype=np.float32)[:, None]
    img[:] = np.dstack([grad * 0.96, grad * 0.97, grad])
    zbuf = np.full((h, w), np.inf, np.float32)

    # Minkstas seselis po objektu.
    pix_all = np.column_stack([
        origin[0] + xy[:, 0] * focal,
        origin[1] - xy[:, 1] * focal,
    ])
    gx = (pix_all[:, 0].min() + pix_all[:, 0].max()) / 2
    gy = pix_all[:, 1].max() + 8
    sx = max((pix_all[:, 0].max() - pix_all[:, 0].min()) * 0.34, 12)
    sy = max(sx * 0.16, 8)
    yy, xx = np.mgrid[0:h, 0:w]
    blob = np.exp(-(((xx - gx) / sx) ** 2 + ((yy - gy) / sy) ** 2))
    img *= (1 - 0.28 * blob[..., None])

    for (tm, col, n), cam in zip(shaded, proj):
        front = n @ forward < -0.05
        faces = tm.faces[front]
        colors = col[front]
        pix = cam[:, :2] / np.clip(cam[:, 2], 1e-3, None)[:, None] * focal
        pix[:, 0] = origin[0] + pix[:, 0]
        pix[:, 1] = origin[1] - pix[:, 1]
        depth = cam[:, 2]
        for f, c in zip(faces, colors):
            tri = pix[f]
            ztri = depth[f]
            minx = max(int(tri[:, 0].min()), 0)
            maxx = min(int(tri[:, 0].max()) + 1, w - 1)
            miny = max(int(tri[:, 1].min()), 0)
            maxy = min(int(tri[:, 1].max()) + 1, h - 1)
            if minx >= maxx or miny >= maxy:
                continue
            xs = np.arange(minx, maxx + 1) + 0.5
            ys = np.arange(miny, maxy + 1) + 0.5
            xg, yg = np.meshgrid(xs, ys)
            x0, y0 = tri[0]
            x1, y1 = tri[1]
            x2, y2 = tri[2]
            den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(den) < 1e-8:
                continue
            w0 = ((y1 - y2) * (xg - x2) + (x2 - x1) * (yg - y2)) / den
            w1 = ((y2 - y0) * (xg - x2) + (x0 - x2) * (yg - y2)) / den
            w2 = 1 - w0 - w1
            mask = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
            if not np.any(mask):
                continue
            zd = w0 * ztri[0] + w1 * ztri[1] + w2 * ztri[2]
            view = zbuf[miny:maxy + 1, minx:maxx + 1]
            hit = mask & (zd < view)
            if not np.any(hit):
                continue
            view[hit] = zd[hit]
            img[miny:maxy + 1, minx:maxx + 1][hit] = c
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


BLOCK_COLOR = np.array([0.72, 0.50, 0.22])
BODY_COLOR = np.array([0.78, 0.79, 0.80])
LID_COLOR = np.array([0.88, 0.89, 0.90])
SHELL_COLOR = np.array([0.84, 0.85, 0.83])
PIN_COLOR = np.array([0.18, 0.19, 0.20])


def goggle_slab():
    slab = trimesh.creation.box((78, 52, 7))
    slab.apply_translation([OX / 2, 0, -3.85])
    return slab


def block_ghost():
    b = trimesh.creation.box((35.0, 22.0, 20.0))
    # Blokas guli prie kairės sienos, ant dugno, su vienodu tarpu.
    b.apply_translation([
        W + 0.30 + 17.5,
        BLOCK_Y0 + 0.30 + 11.0,
        FLOOR + 0.30 + 10.0,
    ])
    return b


def pin_along_x(x0, x1, y, z, r=0.85):
    pin = trimesh.creation.cylinder(radius=r, height=abs(x1 - x0), sections=32)
    pin.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    pin.apply_translation([(x0 + x1) / 2, y, z])
    return pin


def _cyl_along(p0, p1, r, sections=28):
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    d = p1 - p0
    h = np.linalg.norm(d)
    tm = trimesh.creation.cylinder(radius=r, height=h, sections=sections)
    axis = d / h
    z = np.array([0.0, 0.0, 1.0])
    c = np.cross(z, axis)
    if np.linalg.norm(c) < 1e-8:
        if axis[2] < 0:
            tm.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]))
    else:
        tm.apply_transform(trimesh.transformations.rotation_matrix(
            math.acos(np.clip(axis[2], -1, 1)), c))
    tm.apply_translation((p0 + p1) / 2)
    return tm


def pod_props():
    """Kištukas, laidas ir antenos viela, kad matytųsi abi pusės."""
    plug = trimesh.creation.box((13.2, 12.4, 7.4))
    plug.apply_translation([
        POD_BAY0 - 1.2 - 6.6,
        BLOCK_YC,
        FLOOR + 1.15 + 3.7,
    ])
    cable = _cyl_along([POD_BAY0 - 14.2, 0.0, FLOOR + 4.9], [-7.5, 0.0, FLOOR + 4.0], 2.05)
    coax_a = _cyl_along(
        [POD_XR - 2.2, BLOCK_YC, 16.5],
        [POD_XR + 1.5, BLOCK_YC, 27.5],
        0.7,
    )
    coax_b = _cyl_along(
        [POD_XR + 1.5, BLOCK_YC, 27.5],
        [POD_XR + 14.0, BLOCK_YC, 31.0],
        0.7,
    )
    return plug, cable, coax_a + coax_b


def render_pod(body, lid):
    cx = POD_XR / 2
    slab = trimesh.creation.box((150, 68, 7))
    slab.apply_translation([cx, 0, -3.7])
    ghost = block_ghost()
    ghost.apply_translation([POD_BAY0 + 0.3 - (W + 0.30), 0, 0])
    plug, cable, coax = pod_props()
    opened = lid.copy()
    opened.apply_translation([-44, 0, 0])
    dark = np.array([0.16, 0.17, 0.18])
    body_c = np.array([0.46, 0.48, 0.50])
    lid_c = np.array([0.80, 0.82, 0.84])
    deck = np.array([0.73, 0.74, 0.72])
    cells = [
        ("Uždaryta", "USB priekyje, antena gale",
         [(slab, deck), (body, body_c), (lid, lid_c), (plug, dark), (cable, dark), (coax, dark)],
         34, -124),
        ("Iš galo", "antena išeina pro viršų",
         [(slab, deck), (body, body_c), (lid, lid_c), (plug, dark), (cable, dark), (coax, dark)],
         28, 52),
        ("Iš šono", "nuožulnūs šonai, plokščias viršus",
         [(slab, deck), (body, body_c), (lid, lid_c), (plug, dark), (cable, dark), (coax, dark)],
         11, -92),
        ("Atidaryta", "dangtis nuslenka link USB laido",
         [(slab, deck), (body, body_c), (ghost, BLOCK_COLOR), (opened, lid_c), (plug, dark), (cable, dark)],
         32, -128),
    ]
    pw, ph = 900, 700
    panels = []
    for title, subtitle, meshes, elev, azim in cells:
        print(f"  piešiu {title}")
        rgb = raster(meshes, pw, ph, elev=elev, azim=azim, margin=0.82)
        panels.append((title, subtitle, rgb))
    label_h = 78
    sheet = Image.new("RGB", (pw * 2, (ph + label_h) * 2), (244, 244, 242))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 28)
    font_s = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 18)
    for i, (title, subtitle, rgb) in enumerate(panels):
        col, row = i % 2, i // 2
        x = col * pw
        y = row * (ph + label_h)
        draw.text((x + 22, y + 14), title, fill=(28, 30, 32), font=font)
        draw.text((x + 22, y + 46), subtitle, fill=(90, 94, 98), font=font_s)
        sheet.paste(Image.fromarray(rgb), (x, y + label_h))
    path = os.path.join(OUT, "deze_A.png")
    sheet.save(path, quality=95)
    # Aukštas vaizdas iš viršaus, kad matytųsi planas.
    print("  piešiu viršų")
    top = raster(cells[0][2], 1100, 780, elev=62, azim=-108, margin=0.84)
    top_img = Image.fromarray(top)
    top_path = os.path.join(OUT, "deze_A_virsus.png")
    top_img.save(top_path, quality=95)
    return path, top_path


def render_sheet(a_body, a_lid, b_body, b_lid, c_body, c_door, c_pin):
    slab = goggle_slab()
    ghost = block_ghost()
    a_open = a_lid.copy()
    a_open.apply_translation([-18, 0, 0])
    b_open = b_lid.copy()
    hinge = [OX / 2, AXIS_Y, AXIS_Z]
    b_open.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(108), [1, 0, 0], hinge))
    c_open = c_door.copy()
    c_open.apply_translation([0, 0, 16])

    b_pin = pin_along_x(1.4, OX - 1.4, AXIS_Y, AXIS_Z, 0.85)
    c_pin_mesh = pin_along_x(OX - 9.6, OX + 2.4, c_pin[0], c_pin[1], 0.85)

    cells = [
        ("A  ·  Bėgelis", "dangtis nuslenka į priekį",
         [(slab, SHELL_COLOR), (a_body, BODY_COLOR), (a_lid, LID_COLOR)]),
        ("B  ·  Vyriai", "atsidaro, ašis lieka dėžutėje",
         [(slab, SHELL_COLOR), (b_body, BODY_COLOR), (b_lid, LID_COLOR), (b_pin, PIN_COLOR)]),
        ("C  ·  Stalčius", "dureles laiko kaištis gale",
         [(slab, SHELL_COLOR), (c_body, BODY_COLOR), (c_door, LID_COLOR), (c_pin_mesh, PIN_COLOR)]),
        ("A atidaryta", "pastumk dangtį ir iškelk bloką",
         [(slab, SHELL_COLOR), (a_body, BODY_COLOR), (ghost, BLOCK_COLOR), (a_open, LID_COLOR)]),
        ("B atidaryta", "atlenk dangtį, ašies neišimk",
         [(slab, SHELL_COLOR), (b_body, BODY_COLOR), (ghost, BLOCK_COLOR), (b_open, LID_COLOR), (b_pin, PIN_COLOR)]),
        ("C atidaryta", "ištrauk kaištį ir pakelk dureles",
         [(slab, SHELL_COLOR), (c_body, BODY_COLOR), (ghost, BLOCK_COLOR), (c_open, LID_COLOR)]),
    ]

    pw, ph = 960, 760
    panels = []
    for title, subtitle, meshes in cells:
        print(f"  piešiu {title}")
        rgb = raster(meshes, pw, ph, elev=27, azim=-128)
        panels.append((title, subtitle, rgb))

    label_h = 78
    sheet = Image.new("RGB", (pw * 3, (ph + label_h) * 2), (244, 244, 242))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 28)
    font_s = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 18)
    for i, (title, subtitle, rgb) in enumerate(panels):
        col, row = i % 3, i // 3
        x = col * pw
        y = row * (ph + label_h)
        draw.text((x + 22, y + 14), title, fill=(28, 30, 32), font=font)
        draw.text((x + 22, y + 46), subtitle, fill=(90, 94, 98), font=font_s)
        sheet.paste(Image.fromarray(rgb), (x, y + label_h))
    path = os.path.join(OUT, "deze_variantai.png")
    sheet.save(path, quality=95)
    return path


# --- L variantas: dvi sesiakampes lekstes --------------------------------------
# Padas siauras, vidurys isplateja ir iskart susiaureja iki dangcio.
# Nera vertikalaus boksto. Isdestytas sesiakampis, kad nebutu platus diskas.

_SQRT3 = math.sqrt(3)
L_CX = 38.6
L_RX = 29.4                 # +1 mm kiekviename gale, kad blokas tilptų
L_Y_FLAT = 16.4             # po 2 mm siauriau iš šono
L_RY = L_Y_FLAT / (_SQRT3 / 2)
L_RX_B = 32.6
L_Y_BELLY = 19.6
L_RY_B = L_Y_BELLY / (_SQRT3 / 2)
# Bėgelis stovi sienos viduje. Kojelės jungtis 1,2 mm, kad 0,4 mm galvutė ją atspausdintų.
L_RAIL_Y = 11.85
L_Y_IN = Y_IN - 2.0
L_BLOCK_L = BLOCK_L - 2.0
L_OZ = OZ - 3.0
L_BAY0 = L_CX - L_BLOCK_L / 2
L_BAY1 = L_CX + L_BLOCK_L / 2
L_SEAM = 0.28
L_LID_T = 1.8
L_RX_LID = L_RX - 0.35
L_RY_LID = (L_Y_FLAT - 0.42) / (_SQRT3 / 2)
# Burna ant priekinio šlaito, kad kojelė įvažiuotų. Galas lieka uždaras ir stabdo dangtį.
L_RAIL_OPEN = L_CX - L_RX / 2 - 4.6
L_RAIL_STOP = L_CX + L_RX / 2 - 0.6
L_FOOT_X0 = L_CX - L_RX / 2 + 1.1
# Kojelė ir statramstis persidengia, statramstis įleistas į dangčio plokštę.
L_FOOT_T0, L_FOOT_T1 = 0.35, 2.35
L_WEB_T0, L_WEB_T1 = 1.55, 3.20
L_CH_T0, L_CH_T1 = 0.10, 2.62
L_SLOT_T0, L_SLOT_T1 = 1.28, 3.50


def hexagon(cx, cy, rx, ry):
    pts = []
    for i in range(6):
        a = math.radians(60 * i)
        pts.append((cx + rx * math.cos(a), cy + ry * math.sin(a)))
    return pts


def hex_solid(cx, rx, ry, z0, z1):
    return extrude(hexagon(cx, 0.0, rx, ry), z0, z1)


def lekste_shell():
    """Dvi lėkštės: nuo pado platėja iki aštraus vidurio ir iškart grįžta prie dangčio."""
    foot = hex_solid(L_CX, L_RX, L_RY, 0.0, 1.35)
    belly = hex_solid(L_CX, L_RX_B, L_RY_B, 8.85, 9.25)
    lip = hex_solid(L_CX, L_RX, L_RY, L_OZ - 3.55, L_OZ - 3.15)
    neck = hex_solid(L_CX, L_RX, L_RY, L_OZ - 3.45, L_OZ)
    return neck + Manifold.batch_hull([foot, belly, lip])


def lekste_rails(body):
    x_stop = L_RAIL_STOP
    for sign in (1, -1):
        y_in = sign * L_RAIL_Y

        def y_at(t0, t1, y_in=y_in, sign=sign):
            a = y_in + sign * t0
            b = y_in + sign * t1
            return (min(a, b), max(a, b))

        y0, y1 = y_at(L_CH_T0, L_CH_T1)
        ch = box(L_RAIL_OPEN, y0, L_OZ - 3.28, x_stop, y1, L_OZ - 1.50)
        y0, y1 = y_at(L_SLOT_T0, L_SLOT_T1)
        web = box(L_RAIL_OPEN, y0, L_OZ - 3.20, x_stop, y1, L_OZ + 3)
        body = body - (ch + web)
    return body


def lekste_foot(sign, x0, x1):
    y_in = sign * L_RAIL_Y

    def y_at(t0, t1):
        a = y_in + sign * t0
        b = y_in + sign * t1
        return (min(a, b), max(a, b))

    y0, y1 = y_at(L_FOOT_T0, L_FOOT_T1)
    bar = box(x0, y0, L_OZ - 3.02, x1, y1, L_OZ - 1.78)
    y0, y1 = y_at(L_WEB_T0, L_WEB_T1)
    web = box(x0, y0, L_OZ - 2.55, x1, y1, L_OZ + L_SEAM + 1.05)
    return bar + web


def lekste_lid():
    z0 = L_OZ + L_SEAM
    z1 = z0 + L_LID_T
    plate = hex_solid(L_CX, L_RX_LID, L_RY_LID, z0, z1)
    tab = hex_solid(L_CX - L_RX_LID + 1.5, 3.2, 3.2, z0, z1)
    plate = plate + tab
    plate = plate + lekste_foot(1, L_FOOT_X0, L_RAIL_STOP - 0.9)
    plate = plate + lekste_foot(-1, L_FOOT_X0, L_RAIL_STOP - 0.9)
    plate = plate - u_slot_x(
        L_CX + L_RX_LID - 2.1, L_CX + L_RX_LID + 2.4,
        0.0, z0 - 0.3, z1 + 1, 2.4,
    )
    return plate


def variant_lekste():
    body = lekste_shell()
    cradle = box(
        L_BAY0 - 1.6, -(L_Y_IN + 1.35), FLOOR - 0.2,
        L_BAY1 + 1.5, L_Y_IN + 1.35, L_OZ - 0.1,
    )
    body = body + cradle
    body = body - extrude(
        rounded_rect(L_BAY0, L_BAY1, -L_Y_IN, L_Y_IN, 0.6),
        FLOOR, L_OZ + 3,
    )
    # USB lovys priekyje. Platesnė dalis kištukui, siauras kanalas laidui pro smaigalį.
    body = body - box(L_BAY0 - 12.5, -8.0, FLOOR, L_BAY0 + 0.4, 8.0, 15.2)
    body = body - u_slot_x(L_CX - L_RX_B - 1.2, L_BAY0 - 5.5, 0.0, 5.4, 16.0, 7.6)
    body = body + box(
        L_BAY0 - 1.45, -L_Y_IN + 0.45, FLOOR,
        L_BAY0 - 0.08, L_Y_IN - LANE - 0.15, FLOOR + 0.85,
    )
    body = body + box(
        L_BAY0 + 1.3, L_Y_IN - LANE - RIB_W, FLOOR - 0.05,
        L_BAY1 - 1.4, L_Y_IN - LANE, FLOOR + RIB_H,
    )
    ant_out = L_CX + L_RX_B + 1.4
    body = body - u_slot_x(L_BAY1 - 0.6, ant_out, 0.0, 12.2, L_OZ + 2, 2.4)
    # Tuštuma po dangciu aplink lopsi, kad diskas nebutu plytos svorio.
    keep = box(
        L_BAY0 - 2.3, -(L_Y_IN + 2.1), 0,
        L_BAY1 + 2.2, L_Y_IN + 2.1, L_OZ + 4,
    )
    skirt = hex_solid(L_CX, L_RX - 3.4, L_RY - 3.4, 2.25, 6.8)
    body = body - (skirt - keep)
    light = hex_solid(L_CX, L_RX - 5.2, (L_RAIL_Y - 0.55) / (_SQRT3 / 2), 5.2, L_OZ + 2)
    body = body - (light - keep)
    body = lekste_rails(body)
    # Plona siūlė ten, kur susitinka dvi lėkštės.
    band = hex_solid(L_CX, L_RX_B + 0.5, L_RY_B + 0.5, 8.82, 9.28)
    inner = hex_solid(L_CX, L_RX_B - 0.4, L_RY_B - 0.4, 8.65, 9.45)
    body = body - (band - inner)
    return body, lekste_lid()


def render_lekste(body, lid):
    slab = trimesh.creation.box((168, 96, 7))
    slab.apply_translation([L_CX, 0, -3.7])
    ghost = trimesh.creation.box((L_BLOCK_L - 0.6, L_Y_IN * 2 - LANE - 1.4, L_OZ - FLOOR - 0.6))
    ghost.apply_translation([
        (L_BAY0 + L_BAY1) / 2,
        -LANE / 2,
        (FLOOR + L_OZ - 0.3) / 2,
    ])
    plug = trimesh.creation.box((13.0, 12.2, 7.2))
    plug.apply_translation([L_BAY0 - 7.4, BLOCK_YC, FLOOR + 1.2 + 3.6])
    cable = _cyl_along([L_BAY0 - 11.0, 0.0, 8.4], [L_CX - L_RX_B - 7.0, 0.0, 7.6], 1.9)
    coax = _cyl_along(
        [L_CX + L_RX + 0.5, 0.0, 18.0],
        [L_CX + L_RX_B + 6.0, 0.0, 28.0],
        0.65,
    )
    opened = lid.copy()
    opened.apply_translation([-40, 0, 0])
    dark = np.array([0.12, 0.13, 0.14])
    body_c = np.array([0.73, 0.74, 0.75])
    lid_c = np.array([0.90, 0.91, 0.92])
    deck = np.array([0.80, 0.81, 0.79])
    base = [(slab, deck), (body, body_c), (lid, lid_c), (plug, dark), (cable, dark), (coax, dark)]
    cells = [
        ("Uždaryta", "šešiakampė lėkštė, USB priekyje", base, 36, -128),
        ("Iš viršaus", "trys šešiakampiai žiedai", base, 64, -96),
        ("Iš šono", "dvi lėkštės, be tiesaus šono", base, 8, -90),
        ("Atidaryta", "dangtis nuslenka link USB",
         [(slab, deck), (body, body_c), (ghost, BLOCK_COLOR), (opened, lid_c), (plug, dark), (cable, dark)],
         34, -132),
    ]
    pw, ph = 900, 700
    panels = []
    for title, subtitle, meshes, elev, azim in cells:
        print(f"  piešiu {title}")
        rgb = raster(meshes, pw, ph, elev=elev, azim=azim, margin=0.84)
        panels.append((title, subtitle, rgb))
    label_h = 78
    sheet = Image.new("RGB", (pw * 2, (ph + label_h) * 2), (244, 244, 242))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 28)
    font_s = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 18)
    for i, (title, subtitle, rgb) in enumerate(panels):
        col, row = i % 2, i // 2
        x = col * pw
        y = row * (ph + label_h)
        draw.text((x + 22, y + 14), title, fill=(28, 30, 32), font=font)
        draw.text((x + 22, y + 46), subtitle, fill=(90, 94, 98), font=font_s)
        sheet.paste(Image.fromarray(rgb), (x, y + label_h))
    path = os.path.join(OUT, "deze_L_lekste.png")
    sheet.save(path, quality=95)
    print("  piešiu planą")
    top = raster(base, 1100, 780, elev=68, azim=-90, margin=0.86)
    top_path = os.path.join(OUT, "deze_L_planas.png")
    Image.fromarray(top).save(top_path, quality=95)
    return path, top_path


def main():
    os.makedirs(OUT, exist_ok=True)
    print(f"A išorė apie {POD_XR:.1f} x {2 * POD_YB:.1f} mm, aukštis {OZ + POD_SEAM + POD_LID_T:.1f} mm")
    print(f"Bloko guolis {BLOCK_L:.1f} x {BLOCK_W:.1f} x {BLOCK_H:.1f} mm, vielų juosta {LANE:.1f} mm")
    print(f"USB lovys prieš bloką, antena ties x={POD_XR:.1f}")

    a_body_m, a_lid_m = variant_a()
    b_body_m, b_lid_m = variant_b()
    c_body_m, c_door_m, c_pin_x, c_pin_z = variant_c()

    if not validate(a_body_m, a_lid_m, b_body_m, b_lid_m, c_body_m, c_door_m):
        print("Yra klaidų, STL vis tiek rašau, kad matytųsi forma.")

    a_body, a_lid = to_trimesh(a_body_m), to_trimesh(a_lid_m)
    b_body, b_lid = to_trimesh(b_body_m), to_trimesh(b_lid_m)
    c_body, c_door = to_trimesh(c_body_m), to_trimesh(c_door_m)

    # Spausdinimo padėtis: plokščias paviršius ant stalo, be atramų.
    export_stl("deze_A_korpus.stl", a_body)
    export_stl("deze_A_dangtis.stl", rot(a_lid, [1, 0, 0], math.pi))
    print("A STL įrašyti, piešiu")
    path, top = render_pod(a_body, a_lid)
    print(path)
    print(top)
    export_stl("deze_B_korpus.stl", b_body)
    export_stl("deze_B_dangtis.stl", rot(b_lid, [1, 0, 0], math.pi))
    export_stl("deze_C_korpus.stl", rot(c_body, [0, 1, 0], -math.pi / 2))
    export_stl("deze_C_dureles.stl", rot(c_door, [0, 1, 0], -math.pi / 2))
    export_stl("deze_C_kaistis.stl", to_trimesh(service_pin()))

    l_body_m, l_lid_m = variant_lekste()
    clash_l = (l_body_m ^ l_lid_m).volume()
    print(f"  L sankirta {clash_l:.2f} mm3")
    l_body, l_lid = to_trimesh(l_body_m), to_trimesh(l_lid_m)
    export_stl("deze_L_korpus.stl", l_body)
    export_stl("deze_L_dangtis.stl", rot(l_lid, [1, 0, 0], math.pi))
    for name, part in (("L korpusas", l_body), ("L dangtis", l_lid)):
        size = part.bounds[1] - part.bounds[0]
        grams = abs(part.volume) / 1000 * 1.24
        print(f"  {name:12s} {size[0]:5.1f} x {size[1]:5.1f} x {size[2]:4.1f} mm   {grams:4.1f} g")
    path, top = render_lekste(l_body, l_lid)
    print(path)
    print(top)


if __name__ == "__main__":
    main()
