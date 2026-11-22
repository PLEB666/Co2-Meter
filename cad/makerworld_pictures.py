"""Pictures of the case for its MakerWorld page, rendered from the model.

Run from the project folder, after cad/case_v5.py (it reads the parts from cad/out/v5/viewer.html):
    cad\\.venv\\Scripts\\python.exe cad\\makerworld_pictures.py            all pictures
    cad\\.venv\\Scripts\\python.exe cad\\makerworld_pictures.py cover      only some

Writes docs/makerworld/NN_name.png, 1600 x 1200 (4:3, as MakerWorld shows pictures). Each picture is
rendered at twice that size by headless Chrome (three.js, software WebGL) and scaled down, which
smooths the edges. Needs an internet connection for three.js (jsdelivr), as the viewer does.

The display shows the firmware's main screen with example values, drawn with the Adafruit GFX font
the firmware uses (from the Arduino libraries folder). Everything else, the countersunk lid screws
included, comes from the model.
"""

import json
import os
import re
import subprocess
import sys
import tempfile
from base64 import b64encode
from io import BytesIO
from pathlib import Path

from PIL import Image, ImageDraw

HERE = Path(__file__).parent
VIEWER = HERE / "out" / "v5" / "viewer.html"
OUT = HERE.parent / "docs" / "makerworld"
GLCD = Path.home() / "Documents" / "Arduino" / "libraries" / "Adafruit_GFX_Library" / "glcdfont.c"
W, H, SS = 1600, 1200, 2               # picture size and supersampling

CASE_COLOUR = "#dedbd3"                # PLA colour in the realistic pictures; change to your filament's

# Example values on the display, and the LED colour that goes with them (CO2 < 600 ppm: GOOD, dark green)
SCREEN = {"co2": 512, "temp": 21.8, "hum": 46, "voc": 103}
LED_GREEN = "#2bff52"

# ---------------------------------------------------------------- the display's screen

def glcd_font():
    text = GLCD.read_text(encoding="utf-8", errors="replace")
    body = text[text.index("font[]"):]
    body = body[body.index("{") + 1:body.index("};")]
    body = re.sub(r"//[^\n]*", "", body)
    return [int(h, 16) for h in re.findall(r"0x[0-9A-Fa-f]{2}", body)]


def main_screen():
    """The firmware's drawMainScreen() with the SCREEN values, 128 x 128, white on black."""
    font = glcd_font()
    img = Image.new("L", (128, 128), 0)
    px = img.load()

    def char(c, size, x, y):
        for i in range(5):
            line = font[ord(c) * 5 + i]
            for j in range(8):
                if line & 1:
                    for dx in range(size):
                        for dy in range(size):
                            if 0 <= x + i * size + dx < 128 and 0 <= y + j * size + dy < 128:
                                px[x + i * size + dx, y + j * size + dy] = 255
                line >>= 1

    def at(t, size, x, y):
        for k, c in enumerate(t):
            char(c, size, x + k * 6 * size, y)
        return x + len(t) * 6 * size              # cursor after the text

    def right(t, size, y):
        at(t, size, 128 - len(t) * 6 * size, y)

    def centred(t, size, y):
        at(t, size, (128 - len(t) * 6 * size) // 2, y)

    at("CO2 ppm", 1, 0, 1)
    right("GOOD", 1, 1)
    centred(str(SCREEN["co2"]), 5, 28)
    for x in range(128):
        px[x, 80] = 255
    ux = at(f"{SCREEN['temp']:.1f}", 2, 0, 88) + 3
    for x, y in ((ux, 88), (ux, 90), (ux - 1, 89), (ux + 1, 89)):   # degree sign: drawCircle(ux, 89, 1)
        px[x, y] = 255
    at("C", 1, ux + 3, 88)
    right(f"{SCREEN['hum']}%", 2, 88)
    at(f"VOC {SCREEN['voc']}", 1, 0, 114)
    right("GOOD", 1, 114)

    # 4 x 4 texture pixels per OLED pixel, with a dark gap between the pixels
    big = Image.new("L", (512, 512), 0)
    draw = ImageDraw.Draw(big)
    for y in range(128):
        for x in range(128):
            if px[x, y]:
                draw.rectangle((x * 4, y * 4, x * 4 + 2, y * 4 + 2), fill=255)
    buf = BytesIO()
    big.save(buf, "PNG")
    return "data:image/png;base64," + b64encode(buf.getvalue()).decode()


# ---------------------------------------------------------------- the pictures

CLOSED = ["base", "lid", "oled", "button", "button_nut", "led", "rubber_feet", "veml7700", "esp32", "lid_screws"]
INSIDE = ["base", "oled", "button", "button_nut", "led", "rubber_feet", "veml7700", "esp32", "esp32_plugs",
          "scd41", "sht45", "bmp580", "sgp40", "plugs", "cables", "screws"]

SHOTS = {
    "01_cover": {
        "show": CLOSED, "palette": "real", "dir": [1, -0.78, 0.55], "fov": 24,
        "region": [-0.84, 0.84, -0.78, 0.82], "screen": True,
    },
    "01_cover_3x4": {   # the app's portrait cover (Bambu Handy), 1200 x 1600; seen a little more from above
        "size": [1200, 1600],
        "show": CLOSED, "palette": "real", "dir": [1, -0.78, 0.75], "fov": 24,
        "region": [-0.86, 0.86, -0.62, 0.66], "screen": True,
    },
    "02_front": {
        "show": CLOSED, "palette": "real", "dir": [1, -0.12, 0.10], "fov": 20,
        "region": [-0.90, 0.58, -0.66, 0.62], "screen": True,
        "title": "Front", "sub": "1.5\" OLED, light sensor window, button and LED",
        "labels": [
            {"text": "Display", "sub": "1.5\" OLED, 128 x 128", "at": [83.4, -9.4, 4.4], "pos": [0.24, 0.925]},
            {"text": "Light sensor", "sub": "auto brightness", "at": [84.3, 30.0, 18.5], "pos": [0.58, 0.925]},
            {"text": "Button", "sub": "screens, sleep mode", "at": [92.0, 38.5, 25.7], "pos": [0.895, 0.30]},
            {"text": "LED", "sub": "CO₂ traffic light", "at": [85.8, 38.5, 11.7], "pos": [0.895, 0.62]},
        ],
    },
    "03_back": {
        "show": CLOSED, "palette": "real", "dir": [-1, 0.75, 0.5], "fov": 24,
        "region": [-0.84, 0.84, -0.78, 0.74],
        "title": "Back", "sub": "USB-C power and the vents",
        "labels": [
            {"text": "USB-C", "sub": "power, firmware upload", "at": [-2.0, 14.2, 5.6], "pos": [0.20, 0.80]},
            {"text": "Vents", "sub": "sensor bays and ESP32", "at": [-2.0, -12.5, 28.0], "pos": [0.83, 0.22]},
        ],
    },
    "04_inside": {
        "show": INSIDE, "palette": "real", "dir": [0.62, -0.62, 1.45], "fov": 24,
        "region": [-0.84, 0.84, -0.78, 0.74], "screen": True,
        "title": "Inside", "sub": "lid off: the sensors and the display on one STEMMA QT cable chain, no soldering",
    },
    "05_layout": {
        "show": INSIDE, "palette": "ident", "ortho": True, "dir": [0, 0, 1], "up": [0, 1, 0],
        "region": [-0.56, 0.56, -0.78, 0.58], "noGround": True,
        "title": "Parts layout", "sub": "seen from above, lid and speaker removed; red: the STEMMA QT cables",
        "labels": [
            {"text": "ESP32-C6", "sub": "Waveshare, upside down", "part": "esp32", "pos": [0.10, 0.45]},
            {"text": "USB-C", "sub": "in the back wall", "at": [-2.0, 14.2, 10.0], "pos": [0.10, 0.64]},
            {"text": "SGP40", "sub": "VOC index", "part": "sgp40", "pos": [0.30, 0.165]},
            {"text": "BMP580", "sub": "air pressure", "part": "bmp580", "pos": [0.56, 0.165]},
            {"text": "Button + LED", "part": "button", "pos": [0.90, 0.30]},
            {"text": "VEML7700", "sub": "light", "part": "veml7700", "pos": [0.90, 0.45]},
            {"text": "Display", "sub": "1.5\" OLED", "part": "oled", "pos": [0.90, 0.64]},
            {"text": "SHT45", "sub": "temperature, humidity", "part": "sht45", "pos": [0.57, 0.945]},
            {"text": "SCD-41", "sub": "CO₂", "part": "scd41", "pos": [0.30, 0.945]},
        ],
    },
    "06_cut": {
        "show": INSIDE + ["lid", "speaker", "lid_screws"], "palette": "ident", "cut": 30.0,
        "dir": [1, -0.22, 0.22], "fov": 18, "region": [-0.50, 0.50, -0.80, 0.70],
        "title": "Cut across the case", "sub": "double walls with an air gap keep the ESP32's heat away from the sensors",
        "labels": [
            {"text": "Sensor chamber", "sub": "SCD-41 and SHT45", "at": [30.0, -17.0, 26.0], "pos": [0.11, 0.30]},
            {"text": "Double wall", "sub": "1.6 mm air gap", "at": [30.0, -2.0, 30.0], "pos": [0.12, 0.62]},
            {"text": "ESP32-C6", "sub": "upside down", "at": [30.0, 14.0, 13.0], "pos": [0.14, 0.86]},
            {"text": "Speaker", "sub": "under the lid", "at": [30.0, 20.0, 36.0], "pos": [0.86, 0.24]},
            {"text": "Double wall", "sub": "1.6 mm air gap", "at": [30.0, 31.0, 22.0], "pos": [0.88, 0.50]},
            {"text": "SGP40 + BMP580", "sub": "their own bay", "at": [30.0, 43.0, 12.0], "pos": [0.86, 0.80]},
        ],
    },
    "07_size": {
        "show": CLOSED, "palette": "real", "dir": [1, -0.85, 0.62], "fov": 24,
        "region": [-0.70, 0.74, -0.70, 0.66], "screen": True,
        "title": "Size", "sub": "86.3 x 77 x 41.4 mm, plus 3.5 mm rubber feet",
        "dims": [
            {"a": [-2.0, -38.0, -3.5], "b": [84.3, -38.0, -3.5], "text": "86.3 mm",
             "ext": [[[-2.0, -24.0, -3.5], [-2.0, -40.0, -3.5]], [[84.3, -24.0, -3.5], [84.3, -40.0, -3.5]]]},
            {"a": [98.3, -24.0, -3.5], "b": [98.3, 53.0, -3.5], "text": "77.0 mm",
             "ext": [[[84.3, -24.0, -3.5], [100.3, -24.0, -3.5]], [[84.3, 53.0, -3.5], [100.3, 53.0, -3.5]]]},
            {"a": [-2.0, -38.0, 0.0], "b": [-2.0, -38.0, 41.43], "text": "41.4 mm",
             "ext": [[[-2.0, -24.0, 41.43], [-2.0, -40.0, 41.43]], [[-2.0, -24.0, 0.0], [-2.0, -40.0, 0.0]]]},
        ],
    },
}

PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8">
<style>
  html, body { margin: 0; height: 100%; overflow: hidden; font-family: "Segoe UI", system-ui, sans-serif;
               background: radial-gradient(ellipse 80% 75% at 50% 45%, #fdfdfe 0%, #eef1f4 55%, #dde2e8 100%); }
  #view, svg { position: fixed; inset: 0; width: 100%; height: 100%; }
  #err { position: fixed; top: 40%; left: 5%; right: 5%; color: #cf222e; font-size: 30px; }
</style>
<script type="importmap">
{ "imports": {
    "three": "https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js",
    "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.170.0/examples/jsm/" } }
</script>
</head><body>
<div id="view"></div><svg id="ov" xmlns="http://www.w3.org/2000/svg"></svg><div id="err"></div>
<script>window.addEventListener('error', e => { document.getElementById('err').textContent = 'Error: ' + e.message; });</script>
<script type="module">
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';

const DATA = /*DATA*/null;
const SHOTS = /*SHOTS*/null;
const SCREEN = '/*SCREEN*/';
const STYLE = /*STYLE*/null;
const shot = SHOTS[location.hash.slice(1)];
const W = innerWidth, H = innerHeight;

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(devicePixelRatio);
renderer.setSize(W, H);
renderer.toneMapping = THREE.NeutralToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.localClippingEnabled = true;
document.getElementById('view').appendChild(renderer.domElement);

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.85;

const clip = shot.cut != null ? [new THREE.Plane(new THREE.Vector3(-1, 0, 0), shot.cut)] : [];
const REAL = {
  base: { c: STYLE.case, r: 0.62 }, lid: { c: STYLE.case, r: 0.62 },
  scd41: { c: '#1e2124', r: 0.45 }, sht45: { c: '#1e2124', r: 0.45 }, bmp580: { c: '#1e2124', r: 0.45 },
  sgp40: { c: '#1e2124', r: 0.45 }, veml7700: { c: '#1e2124', r: 0.45 }, esp32: { c: '#1b2230', r: 0.45 },
  esp32_plugs: { c: '#2b2f36', r: 0.5 }, plugs: { c: '#f1efe8', r: 0.5 }, cables: { c: '#b3261e', r: 0.5 },
  oled: { c: '#07080a', r: 0.15 }, button: { c: '#c62828', r: 0.4 }, button_nut: { c: '#c3c8ce', m: 0.85, r: 0.3 },
  led: { c: '#d6ffe0', e: STYLE.led, ei: 1.4, r: 0.3 }, speaker: { c: '#26292d', r: 0.5 },
  screws: { c: '#aab0b7', m: 0.85, r: 0.35 }, rubber_feet: { c: '#202326', r: 0.9 }, lid_screws: { c: '#b4bac1', m: 0.85, r: 0.3 },
};
const meshes = {};
const loader = new STLLoader();

function material(name, colour) {
  const s = shot.palette === 'real' ? (REAL[name] || { c: colour }) : { c: colour, r: 0.6 };
  const m = new THREE.MeshStandardMaterial({ color: s.c, roughness: s.r ?? 0.6, metalness: s.m ?? 0,
    side: clip.length ? THREE.FrontSide : THREE.DoubleSide, clippingPlanes: clip, clipShadows: true });
  if (s.e) { m.emissive = new THREE.Color(s.e); m.emissiveIntensity = s.ei; }
  return m;
}

function addMesh(name, geom, colour, lid) {
  const mesh = new THREE.Mesh(geom, material(name, colour));
  mesh.castShadow = mesh.receiveShadow = true;
  if (clip.length) {   // the inside of a cut solid shows its back faces: give them a flat cut colour
    const cap = new THREE.Color(shot.palette === 'real' ? REAL[name]?.c || colour : colour).multiplyScalar(0.72);
    const capMesh = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({ color: cap, side: THREE.BackSide, clippingPlanes: clip }));
    capMesh.castShadow = false;
    mesh.add(capMesh);
  }
  if (shot.palette === 'ident') {
    mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(geom, 25),
      new THREE.LineBasicMaterial({ color: 0x1f2328, transparent: true, opacity: 0.28, clippingPlanes: clip })));
  }
  mesh.userData.lid = lid;
  scene.add(mesh);
  meshes[name] = mesh;
}

for (const p of DATA.parts) {
  if (!shot.show.includes(p.name)) continue;
  const bytes = Uint8Array.from(atob(p.stl), c => c.charCodeAt(0));
  addMesh(p.name, toCreasedNormals(loader.parse(bytes.buffer), 0.6), p.color, p.lid);
}

// display screen: the firmware's main screen in front of the glass
if (shot.screen && meshes.oled) {
  const lit = DATA.parts.find(p => p.name === 'oled_lit');
  const g = loader.parse(Uint8Array.from(atob(lit.stl), c => c.charCodeAt(0)).buffer);
  g.computeBoundingBox();
  const b = g.boundingBox, x = b.max.x + 0.03;
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute([x, b.min.y, b.min.z, x, b.max.y, b.min.z, x, b.max.y, b.max.z, x, b.min.y, b.max.z], 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute([0, 0, 1, 0, 1, 1, 0, 1], 2));
  geo.setIndex([0, 1, 2, 0, 2, 3]);
  const img = new Image(); img.src = SCREEN; await img.decode();
  const tex = new THREE.Texture(img);
  tex.colorSpace = THREE.SRGBColorSpace; tex.anisotropy = 8; tex.needsUpdate = true;
  scene.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ map: tex, color: '#eef3ff', toneMapped: false })));
}

// the camera fits what is shown into the picture's region (NDC, x0 x1 y0 y1)
const visible = Object.values(meshes);
const pts = [];
for (const m of visible) {
  m.updateMatrixWorld();
  const pos = m.geometry.attributes.position, step = Math.max(1, Math.floor(pos.count / 6000));
  for (let i = 0; i < pos.count; i += step) {
    const v = new THREE.Vector3().fromBufferAttribute(pos, i).applyMatrix4(m.matrixWorld);
    if (clip.length && clip[0].distanceToPoint(v) < 0) continue;
    pts.push(v);
  }
}
const box = new THREE.Box3().setFromPoints(pts);
const floorZ = box.min.z;
const aspect = W / H, dir = new THREE.Vector3(...shot.dir).normalize();
const camera = shot.ortho ? new THREE.OrthographicCamera(-1, 1, 1, -1, 1, 5000) : new THREE.PerspectiveCamera(shot.fov, aspect, 1, 5000);
camera.up.set(...(shot.up || [0, 0, 1]));
const [rx0, rx1, ry0, ry1] = shot.region;
let target = box.getCenter(new THREE.Vector3());
let dist = box.getSize(new THREE.Vector3()).length() / (2 * Math.tan((shot.fov || 30) * Math.PI / 360));
let half = box.getSize(new THREE.Vector3()).length() / 2;
for (let it = 0; it < 14; it++) {
  camera.position.copy(target).addScaledVector(dir, shot.ortho ? 1000 : dist);
  camera.lookAt(target);
  if (shot.ortho) Object.assign(camera, { left: -half * aspect, right: half * aspect, top: half, bottom: -half });
  camera.updateMatrixWorld(); camera.updateProjectionMatrix();
  let x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
  for (const p of pts) {
    const q = p.clone().project(camera);
    x0 = Math.min(x0, q.x); x1 = Math.max(x1, q.x); y0 = Math.min(y0, q.y); y1 = Math.max(y1, q.y);
  }
  const s = Math.max((x1 - x0) / (rx1 - rx0), (y1 - y0) / (ry1 - ry0));
  const cx = (x0 + x1) / 2 - (rx0 + rx1) / 2, cy = (y0 + y1) / 2 - (ry0 + ry1) / 2;
  const hh = shot.ortho ? half : dist * Math.tan(shot.fov * Math.PI / 360), hw = hh * aspect;
  const right = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
  const upv = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 1);
  target.addScaledVector(right, cx * hw).addScaledVector(upv, cy * hh);
  if (shot.ortho) half *= s; else dist *= s;
}

// lights: the room environment, plus a sun from the upper left behind the camera, with soft shadows
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
const right = new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld, 0);
sun.position.copy(target).addScaledVector(dir, 150).addScaledVector(right, -110).add(new THREE.Vector3(0, 0, 220));
sun.target.position.copy(target);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
Object.assign(sun.shadow.camera, { left: -180, right: 180, top: 180, bottom: -180, near: 1, far: 900 });
sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.05; sun.shadow.radius = 3;
scene.add(sun, sun.target);

if (!shot.noGround) {
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(4000, 4000), new THREE.ShadowMaterial({ opacity: 0.16 }));
  ground.position.z = floorZ - 0.02;
  ground.receiveShadow = true;
  scene.add(ground);
}

renderer.render(scene, camera);

// ---- overlay: title, labels with leader lines, dimension lines
const svg = document.getElementById('ov');
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs, parent = svg) {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  parent.appendChild(e);
  return e;
}
function screen(v) {
  const q = new THREE.Vector3(...v).project(camera);
  return [(q.x + 1) / 2 * W, (1 - q.y) / 2 * H];
}
if (shot.title) {
  el('text', { x: 56, y: 82, 'font-size': 40, 'font-weight': 650, fill: '#1f2328' }).textContent = shot.title;
  if (shot.sub) el('text', { x: 58, y: 122, 'font-size': 25, fill: '#57606a' }).textContent = shot.sub;
}
const defs = el('defs', {});
const marker = el('marker', { id: 'arrow', viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 9, markerHeight: 9, orient: 'auto-start-reverse' }, defs);
el('path', { d: 'M0,1 L10,5 L0,9 z', fill: '#1f2328' }, marker);
for (const d of shot.dims || []) {
  for (const [p, q] of d.ext) {
    const [ax, ay] = screen(p), [bx, by] = screen(q);
    el('line', { x1: ax, y1: ay, x2: bx, y2: by, stroke: '#57606a', 'stroke-width': 1.6, 'stroke-dasharray': '6 5' });
  }
  const [ax, ay] = screen(d.a), [bx, by] = screen(d.b);
  el('line', { x1: ax, y1: ay, x2: bx, y2: by, stroke: '#1f2328', 'stroke-width': 2.4, 'marker-start': 'url(#arrow)', 'marker-end': 'url(#arrow)' });
  const t = el('text', { x: (ax + bx) / 2, y: (ay + by) / 2 + 10, 'font-size': 30, 'font-weight': 650, fill: '#1f2328',
    'text-anchor': 'middle', stroke: '#ffffff', 'stroke-width': 9, 'paint-order': 'stroke', 'stroke-linejoin': 'round' });
  t.textContent = d.text;
}
const lines = el('g', {}), boxes = el('g', {});
for (const l of shot.labels || []) {
  let anchor;
  if (l.part) {
    const b = new THREE.Box3().setFromObject(meshes[l.part]);
    if (clip.length) b.max.x = Math.min(b.max.x, shot.cut);
    anchor = b.getCenter(new THREE.Vector3()).toArray();
    anchor[2] += l.dz || 0;
  } else anchor = l.at;
  const [ax, ay] = screen(anchor), lx = l.pos[0] * W, ly = l.pos[1] * H;
  el('line', { x1: ax, y1: ay, x2: lx, y2: ly, stroke: '#1f2328', 'stroke-width': 2.2 }, lines);
  el('circle', { cx: ax, cy: ay, r: 6.5, fill: '#1f2328', stroke: '#ffffff', 'stroke-width': 2.5 }, lines);
  const g = el('g', {}, boxes);
  const t1 = el('text', { x: lx, y: l.sub ? ly - 4 : ly + 9, 'font-size': 26, 'font-weight': 650, fill: '#1f2328', 'text-anchor': 'middle' }, g);
  t1.textContent = l.text;
  let wText = t1.getBBox().width;
  if (l.sub) {
    const t2 = el('text', { x: lx, y: ly + 24, 'font-size': 20, fill: '#57606a', 'text-anchor': 'middle' }, g);
    t2.textContent = l.sub;
    wText = Math.max(wText, t2.getBBox().width);
  }
  const bw = wText + 34, bh = l.sub ? 72 : 46;
  const r = el('rect', { x: lx - bw / 2, y: ly - bh / 2, width: bw, height: bh, rx: 10, fill: '#ffffff',
    stroke: '#c9d1d9', 'stroke-width': 1.5 }, g);
  g.insertBefore(r, g.firstChild);
}
document.title = 'done';
</script>
</body></html>
"""


def chrome():
    env = os.environ.get
    for c in (Path(env("LOCALAPPDATA", "")) / "Google/Chrome/Application/chrome.exe",
              Path(env("ProgramFiles", "")) / "Google/Chrome/Application/chrome.exe",
              Path(env("ProgramFiles(x86)", "")) / "Google/Chrome/Application/chrome.exe",
              Path(env("ProgramFiles(x86)", "")) / "Microsoft/Edge/Application/msedge.exe",
              Path(env("ProgramFiles", "")) / "Microsoft/Edge/Application/msedge.exe"):
        if c.is_file():
            return c
    sys.exit("Neither Chrome nor Edge was found.")


def main():
    wanted = sys.argv[1:] or list(SHOTS)
    match = lambda s, n: s == n or s.startswith(n) or s.endswith("_" + n)   # "01", "cover" or "01_cover"
    unknown = [n for n in wanted if not any(match(s, n) for s in SHOTS)]
    if unknown:
        sys.exit(f"unknown picture(s) {unknown}; there are: {', '.join(SHOTS)}")
    names = [s for s in SHOTS if any(match(s, n) for n in wanted)]

    text = VIEWER.read_text(encoding="utf-8")
    data = re.search(r"const DATA = (\{.*?\});\n", text, re.S).group(1)
    style = {"case": CASE_COLOUR, "led": LED_GREEN}
    page = (PAGE.replace("/*DATA*/null", data).replace("/*SHOTS*/null", json.dumps(SHOTS))
            .replace("/*SCREEN*/", main_screen()).replace("/*STYLE*/null", json.dumps(style)))

    OUT.mkdir(parents=True, exist_ok=True)
    browser = chrome()
    with tempfile.TemporaryDirectory() as tmp:
        html = Path(tmp) / "render.html"
        html.write_text(page, encoding="utf-8")
        for name in names:
            w, h = SHOTS[name].get("size", (W, H))
            raw = Path(tmp) / f"{name}_raw.png"
            subprocess.run([str(browser), "--headless=new", f"--screenshot={raw}", f"--window-size={w},{h}",
                            f"--force-device-scale-factor={SS}", "--hide-scrollbars", "--use-angle=swiftshader",
                            "--enable-unsafe-swiftshader", "--virtual-time-budget=120000", "--no-first-run",
                            "--no-default-browser-check", f"--user-data-dir={Path(tmp) / 'profile'}",
                            html.as_uri() + "#" + name], capture_output=True, timeout=600)
            if not raw.is_file():
                print(f"{name}: no picture (Chrome failed)")
                continue
            img = Image.open(raw).convert("RGB").resize((w, h), Image.LANCZOS)
            out = OUT / f"{name}.png"
            img.save(out, optimize=True)
            print(f"{name}: {out.relative_to(HERE.parent)} ({out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
