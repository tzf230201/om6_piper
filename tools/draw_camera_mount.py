"""Offline CAD-derived drawing. No robot/SDK imports and no source-asset edits.

Run with the bundled Python (numpy, Pillow, reportlab). Assembly dimensions use
the active fitted viewer mesh; original part envelopes are shown separately.
All projections are orthographic and every coordinate is in millimetres.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math

import numpy as np
from PIL import Image
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont


REPO = Path(__file__).resolve().parents[1]
WORKSPACE = REPO.parent
CAD = REPO / 'piper_readonly/assets/wrist_cad'
PROFILE = WORKSPACE / 'output/cad_optical_20261005_v1/fgng_config.cad_nominal.json'
GRIPPER = WORKSPACE / 'piper_readonly/piper_model_source/src/piper_description/meshes/gripper_base.STL'
INK = '#171A20'
MUTED = '#65696F'
DATUM = '#474B51'
OPTICAL = '#171A20'
PALE = '#DCDDDF'
WIDTH, HEIGHT = 420, 297


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_stl(path):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    assert len(raw) == 84 + 50 * count, path
    dtype = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(raw, dtype=dtype, count=count, offset=84)['v'].astype(float) * 1000


def pose(xyz, rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = np.asarray(xyz) * 1000
    return result


def place(triangles, transform):
    return triangles @ transform[:3, :3].T + transform[:3, 3]


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def geometry():
    manifest = json.loads((CAD / 'manifest.json').read_text())
    assert digest(REPO / 'cad/source/arm_with_wirst_mount_d405.stp') == manifest['source_sha256']
    fitted = json.loads((CAD / 'fitted_mount.json').read_text())
    config = json.loads(PROFILE.read_text())
    assert fitted['enabled'] and fitted['viewer_only']
    assert fitted['source_sha256'] == manifest['source_sha256']
    originals = {}
    for part in manifest['parts']:
        assert digest(CAD / part['file']) == fitted['original_mesh_sha256'][part['file']]
        originals[part['name']] = load_stl(CAD / part['file'])
    assert digest(CAD / fitted['bottom_file']) == fitted['derived_mesh_sha256']
    anchor = pose(**fitted['anchor_origin'])
    camera_part = next(p for p in manifest['parts'] if p['name'] == 'RealSense_D405')
    camera_body = anchor @ pose(**camera_part['origin'])
    optical = camera_body @ pose(**config['body_from_depth_optical'])
    bottom = anchor @ pose(**fitted['bottom_origin'])
    meshes = [
        ('gripper', load_stl(GRIPPER), [222, 222, 222]),
        ('lower', place(load_stl(CAD / fitted['bottom_file']), bottom), [163, 163, 163]),
        ('upper', place(originals['realsense_d405_mounting_top'], anchor), [181, 181, 181]),
        ('camera', place(originals['RealSense_D405'], camera_body), [211, 211, 211]),
    ]
    expected = np.array([[0, 1, 0, -83.75], [-1, 0, 0, 9], [0, 0, 1, 26.7], [0, 0, 0, 1.]])
    np.testing.assert_allclose(optical, expected, atol=1e-8)
    # Verify the actual fitted STL bore circles, without trusting labels alone.
    vertices = load_stl(CAD / fitted['bottom_file']).reshape(-1, 3)
    centers = []
    for z in (-12., -24.):
        radius = np.linalg.norm(vertices[:, [0, 2]] - [47.5, z], axis=1)
        rim = vertices[(abs(vertices[:, 1] + 1) < 1e-4) & (abs(radius - 1.5) < 1e-4)]
        assert len(rim) >= 100
        xy = rim[:, [0, 2]]
        a = np.column_stack((2 * xy, np.ones(len(xy))))
        x, zz, _ = np.linalg.lstsq(a, (xy * xy).sum(axis=1), rcond=None)[0]
        centers.append((bottom @ [x, 5, zz, 1])[:3])
    np.testing.assert_allclose(centers, [[-36, -6, 44], [-36, 6, 44]], atol=1e-4)
    return meshes, originals, optical, bottom, anchor


def render(meshes, u, v, path, ppm=9):
    """Orthographic software z-buffer: no camera intrinsics or pose offsets."""
    u, v = unit(u), unit(v)
    assert abs(u @ v) < 1e-8
    depth_axis = np.cross(u, v)
    basis = np.column_stack((u, v, depth_axis))
    key = hashlib.sha256(b'orthographic-zbuffer-v2-monochrome')
    key.update(basis.tobytes()); key.update(str(ppm).encode())
    for name, triangles, color in meshes:
        key.update(name.encode()); key.update(triangles.tobytes()); key.update(bytes(color))
    cache = path.with_suffix('.json')
    if path.is_file() and cache.is_file():
        saved = json.loads(cache.read_text())
        if saved['key'] == key.hexdigest():
            return np.array(saved['lo']), np.array(saved['hi'])
    projected = [(name, t @ basis, color) for name, t, color in meshes]
    all_vertices = np.concatenate([t.reshape(-1, 3) for _, t, _ in projected])
    lo = all_vertices[:, :2].min(axis=0) - 1.5
    hi = all_vertices[:, :2].max(axis=0) + 1.5
    w, h = np.ceil((hi - lo) * ppm).astype(int) + 1
    zbuf = np.full((h, w), -np.inf, dtype=np.float32)
    ids = np.zeros((h, w), dtype=np.uint8)
    image = np.full((h, w, 3), 255, dtype=np.uint8)
    light = unit([-.3, .6, 1])
    for part_id, (_, triangles, color) in enumerate(projected, 1):
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        norms = np.linalg.norm(normals, axis=1)
        visible = (normals[:, 2] > 1e-12) & (norms > 1e-12)
        for tri, normal in zip(triangles[visible], (normals[visible] / norms[visible, None])):
            xy = (tri[:, :2] - lo) * ppm
            x0, y0 = np.maximum(np.floor(xy.min(axis=0)).astype(int), 0)
            x1, y1 = np.minimum(np.ceil(xy.max(axis=0)).astype(int), [w - 1, h - 1])
            if x0 > x1 or y0 > y1:
                continue
            a, b, d = xy
            den = (b[1] - d[1]) * (a[0] - d[0]) + (d[0] - b[0]) * (a[1] - d[1])
            if abs(den) < 1e-10:
                continue
            yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
            wa = ((b[1] - d[1]) * (xx - d[0]) + (d[0] - b[0]) * (yy - d[1])) / den
            wb = ((d[1] - a[1]) * (xx - d[0]) + (a[0] - d[0]) * (yy - d[1])) / den
            wc = 1 - wa - wb
            z = wa * tri[0, 2] + wb * tri[1, 2] + wc * tri[2, 2]
            crop = zbuf[y0:y1 + 1, x0:x1 + 1]
            mask = (wa >= -1e-6) & (wb >= -1e-6) & (wc >= -1e-6) & (z > crop)
            crop[mask] = z[mask]
            ids[y0:y1 + 1, x0:x1 + 1][mask] = part_id
            shade = .73 + .27 * max(0., normal @ light)
            image[y0:y1 + 1, x0:x1 + 1][mask] = np.clip(np.array(color) * shade, 0, 255).astype('u1')
    # Surface discontinuities and part boundaries, not a triangulation wireframe.
    edge = np.zeros_like(ids, dtype=bool)
    for dy, dx in ((1, 0), (0, 1)):
        other_id = np.roll(ids, (dy, dx), axis=(0, 1))
        other_z = np.roll(zbuf, (dy, dx), axis=(0, 1))
        both = (ids > 0) & (other_id > 0)
        jump = np.zeros_like(ids, dtype=bool)
        jump[both] = abs(zbuf[both] - other_z[both]) > .5
        edge |= ((ids != other_id) & (ids > 0)) | jump
    image[edge] = [70, 70, 70]
    Image.fromarray(image[::-1]).save(path)
    cache.write_text(json.dumps(dict(key=key.hexdigest(), lo=lo.tolist(), hi=hi.tolist())))
    return lo, hi


class Sheet:
    def __init__(self, pdf):
        self.c = pdf

    def text(self, x, y, text, size=8, bold=False, color=INK, align='left'):
        self.c.setFillColor(colors.HexColor(color))
        self.c.setFont('DocSans-Bold' if bold else 'DocSans', size)
        {'left': self.c.drawString, 'center': self.c.drawCentredString, 'right': self.c.drawRightString}[align](x * mm, y * mm, text)

    def label(self, x, y, text, size=8, tracking=1, color=INK):
        """Small, tracked section labels; no decorative display typeface."""
        self.c.saveState()
        obj = self.c.beginText(x * mm, y * mm)
        obj.setFont('DocSans-Bold', size)
        obj.setFillColor(colors.HexColor(color))
        obj.setCharSpace(tracking)
        obj.textOut(text)
        self.c.drawText(obj)
        self.c.restoreState()

    def line(self, a, b, color=INK, width=.22, dash=None):
        self.c.setStrokeColor(colors.HexColor(color)); self.c.setLineWidth(width * mm)
        self.c.setDash(*([d * mm for d in dash] if dash else []))
        self.c.line(a[0] * mm, a[1] * mm, b[0] * mm, b[1] * mm)
        self.c.setDash()

    def box(self, x, y, w, h, fill=None, stroke=PALE):
        self.c.setStrokeColor(colors.HexColor(stroke)); self.c.setLineWidth(.2 * mm)
        if fill: self.c.setFillColor(colors.HexColor(fill))
        self.c.rect(x * mm, y * mm, w * mm, h * mm, stroke=1, fill=bool(fill))

    def arrow(self, a, b, color=INK, width=.25, head=1.4):
        self.line(a, b, color, width)
        a, b = np.asarray(a), np.asarray(b)
        d = unit(b - a); n = np.array([-d[1], d[0]])
        self.line(b, b - head * d + .35 * head * n, color, width)
        self.line(b, b - head * d - .35 * head * n, color, width)

    def dimh(self, x1, x2, y1, y2, y, label, color=MUTED):
        for x, start in ((x1, y1), (x2, y2)):
            self.line((x, start), (x, y + math.copysign(1.4, y - start)), color, .13)
        self.arrow((x1, y), (x2, y), color, .16, 1.2)
        self.arrow((x2, y), (x1, y), color, .16, 1.2)
        self.text((x1 + x2) / 2, y + 1.8, label, 7.5, color=color, align='center')

    def dimv(self, y1, y2, x1, x2, x, label, color=MUTED):
        for y, start in ((y1, x1), (y2, x2)):
            self.line((start, y), (x + math.copysign(1.4, x - start), y), color, .13)
        self.arrow((x, y1), (x, y2), color, .16, 1.2)
        self.arrow((x, y2), (x, y1), color, .16, 1.2)
        self.c.saveState(); self.c.translate((x - 1.8) * mm, ((y1 + y2) / 2) * mm); self.c.rotate(90)
        self.text(0, 0, label, 7.5, color=color, align='center'); self.c.restoreState()

    def cross(self, p, label=None, color=OPTICAL):
        x, y = p
        self.line((x - 2, y), (x + 2, y), color, .25)
        self.line((x, y - 2), (x, y + 2), color, .25)
        self.c.setStrokeColor(colors.HexColor(color)); self.c.circle(x * mm, y * mm, 1.1 * mm, stroke=1, fill=0)
        if label: self.text(x + 2.8, y + 2, label, 7.5, True, color)

    def header(self, title, subtitle, page):
        self.label(14, 284, 'OM6  /  MECHANICAL ENGINEERING', 7, 1.25, MUTED)
        self.label(14, 274, title, 20, 1.3)
        self.text(14, 266, subtitle, 8.5, color=MUTED)
        self.text(405, 280, f'{page:02d} / 02', 12, align='right')
        self.text(405, 269, 'PIPER  +  REALSENSE D405', 8, color=MUTED, align='right')
        self.line((14, 259), (406, 259), INK, .35)
        self.line((14, 35), (406, 35), INK, .25)
        self.label(14, 27, 'ENGINEERING REFERENCE', 7, .8)
        self.text(14, 20, 'Nominal CAD. Verify the physical interface before fabrication.', 8, color=MUTED)
        self.text(14, 14, 'Not released for manufacture. Unspecified tolerances, material, fasteners and torque.', 7, color=MUTED)
        self.label(278, 27, 'DRAWING', 6.5, .6, MUTED)
        self.label(337, 27, 'REV', 6.5, .6, MUTED)
        self.label(363, 27, 'DATE', 6.5, .6, MUTED)
        self.text(278, 20, 'OM6-CAM-001', 9, True)
        self.text(337, 20, 'B', 9, True)
        self.text(363, 20, '2026-10-10', 8)
        self.text(405, 12, 'A3  /  mm  /  PRINT AT 100%', 7, color=MUTED, align='right')

    def note_lines(self, x, y, lines, size=8, step=4.8, color=MUTED):
        for line in lines:
            self.text(x, y, line, size, color=color); y -= step


def view(sheet, meshes, u, v, page_center, scale, name, scratch):
    path = scratch / (name + '.png')
    lo, hi = render(meshes, u, v, path)
    middle = (lo + hi) / 2
    start = np.asarray(page_center) - (hi - lo) * scale / 2
    sheet.c.drawImage(str(path), start[0] * mm, start[1] * mm,
                      (hi[0] - lo[0]) * scale * mm, (hi[1] - lo[1]) * scale * mm)
    def project(point):
        return np.asarray(page_center) + (np.array([unit(u) @ point, unit(v) @ point]) - middle) * scale
    return project


def create(output):
    output.mkdir(parents=True, exist_ok=True)
    scratch = REPO / '.local/drawing_preview/d405_mount_no_offset'
    scratch.mkdir(parents=True, exist_ok=True)
    inputs = [CAD / name for name in ('manifest.json', 'fitted_mount.json', 'attachment.json',
               'realsense_d405_mounting_bot.stl', 'realsense_d405_mounting_bot_fitted.stl',
               'realsense_d405_mounting_top.stl', 'RealSense_D405.stl')] + [PROFILE, GRIPPER,
                 REPO / 'cad/source/arm_with_wirst_mount_d405.stp']
    hashes = {str(p.relative_to(WORKSPACE)): digest(p) for p in inputs}
    for name, file in [('DocSans', 'arial.ttf'), ('DocSans-Bold', 'arialbd.ttf'), ('DocMono', 'consola.ttf')]:
        pdfmetrics.registerFont(TTFont(name, str(Path('C:/Windows/Fonts') / file)))
    meshes, originals, optical, bottom, anchor = geometry()
    filename = output / 'd405_mount_no_offset.pdf'
    c = canvas.Canvas(str(filename), pagesize=(WIDTH * mm, HEIGHT * mm), pageCompression=1)
    c.setTitle('OM6-CAM-001 Rev B | Wrist Camera Mount')
    c.setAuthor('OM6 PiPER')
    c.setSubject('Mechanical interface and nominal depth-camera transform; zero world offset')
    c.setCreator('OM6 engineering drawing tools')
    s = Sheet(c)
    s.header('WRIST CAMERA MOUNT', 'Assembly and optical datum. Reference frame G: gripper_base.', 1)
    s.line((296, 40), (296, 258), PALE)
    for x, y, title, detail in ((15, 250, '01 / FRONT', 'VIEW -Z_G   /   1:2'),
                                (156, 250, '02 / SIDE', 'VIEW +Y_G   /   1:2'),
                                (15, 143, '03 / TOP', 'VIEW +X_G   /   1:2'),
                                (156, 143, '04 / ASSEMBLY', 'ORTHOGRAPHIC   /   NTS')):
        s.label(x, y, title, 8, .8); s.text(x, y - 5, detail, 7, color=MUTED)
    P = optical[:3, 3]
    A = view(s, meshes, [0, 1, 0], [-1, 0, 0], [83, 195], .5, 'assembly_front', scratch)
    B = view(s, meshes, [0, 0, 1], [-1, 0, 0], [220, 195], .5, 'assembly_side', scratch)
    C = view(s, meshes, [0, -1, 0], [0, 0, 1], [83, 93], .5, 'assembly_top', scratch)
    u = unit([0, .82, -.57]); v = unit([-1, -.30, -.4315789474]); v -= (v @ u) * u; v = unit(v)
    D = view(s, meshes, u, v, [220, 91], .58, 'assembly_iso', scratch)
    for project in (A, B, C):
        s.cross(project([0, 0, 0]), 'G', DATUM)
        s.cross(project(P), 'C', OPTICAL)
    a0, ac = A([0, 0, 0]), A(P)
    s.dimv(a0[1], ac[1], a0[0], ac[0], 26, '83.75  (-X_G)', OPTICAL)
    s.dimh(ac[0], a0[0], ac[1], ac[1], 236, '9.00  (+Y_G)', OPTICAL)
    b0, bc = B([0, 0, 0]), B(P)
    s.dimh(b0[0], bc[0], b0[1], bc[1], 152, '26.70  (+Z_G)', OPTICAL)
    s.arrow(bc, B(P + [0, 0, 34]), OPTICAL, .3, 2)
    s.text(B(P + [0, 0, 34])[0] + 1, bc[1] - 4, '+Z_C = +Z_G', 7, color=OPTICAL)
    c1, c2 = C([-51.75, -52.5, 0]), C([-51.75, 52.5, 0])
    s.dimh(c2[0], c1[0], c2[1], c1[1], 126, '105.00', MUTED)
    s.note_lines(16, 56, ['G  Gripper frame origin; not the tool center point.',
                         'C  Nominal depth optical center; not the RGB origin.'], 7.5, 4.7)
    s.note_lines(157, 45, ['Jaws, fasteners and cable omitted.'], 7.3)
    s.label(303, 250, 'CAMERA DATUM', 8.5, .9)
    s.text(303, 242, 'Depth optical frame C relative to G', 8, color=MUTED)
    s.line((303, 236), (405, 236), INK, .25)
    for y, axis, value in ((228, 'X_G', '-83.75'), (218, 'Y_G', '+9.00'), (208, 'Z_G', '+26.70')):
        s.text(304, y, axis, 8)
        s.text(391, y, value, 13, align='right')
        s.text(397, y, 'mm', 7, color=MUTED)
    s.line((303, 204), (405, 204), PALE, .2)
    s.text(303, 197, 'RPY [deg]     0 / 0 / -90', 8)
    s.text(303, 190, 'X_C = -Y_G;  Y_C = +X_G;  Z_C = +Z_G', 7.5)
    s.label(303, 179, 'CAMERA TO GRIPPER', 7.5, .7)
    s.text(303, 173, 'T_G<-C  |  translation in mm', 7, color=MUTED)
    c.setFont('DocMono', 9)
    c.setFillColor(colors.HexColor(INK))
    for i, row in enumerate(['[ 0   1   0   -83.75 ]', '[-1   0   0     9.00 ]',
                             '[ 0   0   1    26.70 ]', '[ 0   0   0     1    ]']):
        c.drawString(307 * mm, (166 - i * 5) * mm, row)
    s.line((303, 145), (405, 145), PALE, .2)
    s.text(303, 138, 'World translation [mm]     0 / 0 / 0', 8.5, True)
    s.note_lines(303, 130, ['Body origin in G: [-83.75, 0, 30.50] mm.',
                          'Depth origin in CAD body: [9, 0, -3.8] mm.'], 7.5, 5)
    s.label(303, 113, 'BASE TRANSFORM', 7.5, .7)
    s.note_lines(303, 106, ['T_B<-C(q) = T_B<-G(q) x T_G<-C',
                          'Joint-dependent via forward kinematics.'], 8, 5)
    s.label(303, 86, 'MODEL BASIS', 7.5, .7)
    s.note_lines(303, 78, ['Nominal CAD with a fitted lower bracket.',
                         'No measured hand-eye calibration.',
                         'Dimensions rounded to 0.01 mm.',
                         '12.00 mm fitted pitch; see Sheet 02.'], 8, 5)
    c.showPage()

    s.header('MOUNT INTERFACE', 'Component dimensions and model revisions. All coordinates refer to frame G unless noted.', 2)
    s.label(15, 250, '01 / LOWER BRACKET', 8.5, .8)
    s.text(15, 244, 'FITTED MESH   /   VIEW +Z_G   /   2:1', 7, color=MUTED)
    lower = [meshes[1]]
    L = view(s, lower, [0, -1, 0], [-1, 0, 0], [68, 205], 2, 'lower_front', scratch)
    l1, l2 = L([-31.5, -18, 44]), L([-31.5, 18, 44])
    s.dimh(l2[0], l1[0], l2[1], l1[1], 167, '36.00')
    ltop, lbottom = L([-56.75, 18, 44]), L([-31.5, 18, 44])
    s.dimv(lbottom[1], ltop[1], lbottom[0], ltop[0], 20, '25.25')
    h1, h2 = L([-36, -6, 44]), L([-36, 6, 44])
    for point in (h1, h2): s.cross(point, color=DATUM)
    s.dimh(h2[0], h1[0], h2[1], h1[1], 184, '12.00 C-C', INK)
    s.arrow((103, 187), h1, INK, .2)
    s.text(103, 189, '2X DIA 3.00', 8, color=INK)
    s.label(15, 153, 'SIDE   /   2:1', 7.5, .6)
    LS = view(s, lower, [0, 0, 1], [-1, 0, 0], [65, 115], 2, 'lower_side', scratch)
    ls1, ls2 = LS([-31.5, 0, 38]), LS([-31.5, 0, 50.5])
    s.dimh(ls1[0], ls2[0], ls1[1], ls2[1], 79, '12.50')
    s.note_lines(15, 65, ['Bore axes parallel to Z_G.',
                         'Centers: X_G = -36.00; Y_G = +/-6.00.',
                         'Diameter and pitch checked against fitted STL.'], 8, 5)
    s.line((143, 43), (143, 258), PALE)
    s.label(151, 250, '02 / UPPER BRACKET', 8.5, .8)
    s.text(151, 244, 'SOURCE GEOMETRY   /   Y_G-Z_G PLANE   /   1:1', 7, color=MUTED)
    upper = [meshes[2]]
    U = view(s, upper, [0, -1, 0], [0, 0, 1], [214, 207], 1, 'upper_plan', scratch)
    u1, u2 = U([-51.75, -52.5, 1.5]), U([-51.75, 52.5, 1.5])
    s.dimh(u2[0], u1[0], u2[1], u1[1], 176, '105.00')
    u3, u4 = U([-51.75, 52.5, 1.5]), U([-51.75, 52.5, 50.5])
    s.dimv(u3[1], u4[1], u3[0], u4[0], 151, '49.00')
    s.label(151, 161, 'SIDE   /   1:1', 7.5, .6)
    US = view(s, upper, [0, 0, 1], [-1, 0, 0], [211, 118], 1, 'upper_side', scratch)
    us1, us2 = US([-104.75, 0, 50.5]), US([-46.7523, 0, 50.5])
    s.dimv(us2[1], us1[1], us2[0], us1[0], 252, '58.00')
    s.note_lines(151, 73, ['Hinge axis parallel to Y_G.',
                          'Axis point: [-51.75, 0, 44.25] mm.',
                          '105 / 49 / 58 are overall mesh dimensions,',
                          'not hole spacing or plate thickness.'], 8, 5)
    s.line((281, 43), (281, 258), PALE)
    s.label(288, 250, '03 / DIMENSION REGISTER', 8.5, .8)
    s.text(288, 243, 'Overall mesh dimensions [mm]', 8, color=MUTED)
    table = [('COMPONENT', 'dX_G', 'dY_G', 'dZ_G'),
             ('Lower bracket', '25.25', '36.00', '12.50'),
             ('Upper bracket', '58.00', '105.00', '49.00'),
             ('D405 body', '42.00', '42.09', '23.00')]
    for i, row in enumerate(table):
        yy = 234 - i * 8
        if i == 0: s.line((287, yy - 3), (405, yy - 3), INK, .22)
        for x, word in zip((289, 344, 367, 390), row): s.text(x, yy, word, 7.5, i == 0)
    s.label(288, 195, 'SOURCE / FITTED MESH', 7.5, .7)
    comparison = [('FEATURE [mm]', 'SOURCE', 'FITTED'), ('Hole pitch', '12.50', '12.00'),
                  ('Hole diameter', '3.00', '3.00'), ('Recess roof X*', '44.00', '43.40')]
    for i, row in enumerate(comparison):
        yy = 185 - i * 7
        for x, word in zip((289, 356, 383), row): s.text(x, yy, word, 7.5, i == 0)
        if i == 0: s.line((287, yy - 3), (405, yy - 3), INK, .22)
    s.note_lines(288, 154, ['*Lower-part CAD frame.',
                          'Mesh-only fit to the official gripper model.',
                          'Source STEP / IAM geometry is unchanged.'], 7.5, 5)
    s.label(288, 130, 'REFERENCE FILES', 7.5, .7)
    s.note_lines(288, 122, ['arm_with_wirst_mount_d405.stp',
                          'gripper_base.STL',
                          'fitted_mount.json / manifest.json'], 8, 5)
    url = 'https://grabcad.com/library/realsense-wrist-mount-for-piper-arm-1'
    s.text(288, 101, 'Design reference / GrabCAD', 8, color=INK)
    c.linkURL(url, (288 * mm, 99 * mm, 398 * mm, 105 * mm), relative=0)
    s.text(288, 95, 'User-supplied link; revision and license unverified.', 7, color=MUTED)
    s.label(288, 82, 'FABRICATION STATUS', 7.5, .7)
    s.note_lines(288, 74, ['Material, tolerances and fastener specification: TBD.',
                         'Physical fit and structural capacity: not verified.',
                         'Nominal CAD dimensions; no inspection record.'], 7.5, 5)
    s.text(288, 52, 'World translation [mm]     0 / 0 / 0', 8, True)
    c.showPage(); c.save()
    assert hashes == {str(p.relative_to(WORKSPACE)): digest(p) for p in inputs}
    report = dict(drawing='OM6-CAM-001', revision='B', date='2026-10-10', language='en',
                  revision_note='English editorial and monochrome layout; geometry and dimensions unchanged.',
                  frame='gripper_base', world_offset_mm=[0, 0, 0], hardware_access=False,
                  hand_eye_calibrated=False, manufacturing_release=False,
                  gripper_from_depth_optical_mm=optical.tolist(), input_sha256=hashes,
                  component_bounds_gripper_mm={n: [t.reshape(-1, 3).min(0).tolist(),
                                                    t.reshape(-1, 3).max(0).tolist()] for n, t, _ in meshes},
                  original_local_envelopes_mm={n: np.ptp(t.reshape(-1, 3), axis=0).tolist() for n, t in originals.items()},
                  source_assets_unchanged=True, fitted_bore_centers_gripper_mm=[[-36, -6, 44], [-36, 6, 44]],
                  pdf_sha256=digest(filename))
    (output / 'drawing_provenance.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(filename)
    print(json.dumps({'pages': 2, 'world_offset_mm': [0, 0, 0], 'inputs_unchanged': True,
                      'camera_position_gripper_mm': optical[:3, 3].tolist()}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=REPO / 'output/pdf/d405_mount_no_offset')
    create(parser.parse_args().output.resolve())
