import * as THREE from './three.module.min.js';
import { color, onChange } from './theme.js';
import { getJSON, store } from './transport.js';
import { Orbit } from './orbit.js';

async function loadOBJ(name, material, parent, onError) {
  try {
    const res = await fetch(`./assets/${name}.obj`);
    if (!res.ok) throw new Error(`${name}.obj: ${res.status}`);
    const txt = await res.text();
    const V = [], N = [], P = [], No = [];
    for (const ln of txt.split('\n')) {
      const tk = ln.split(/\s+/);
      if (tk[0] === 'v') V.push([+tk[1], +tk[2], +tk[3]]);
      else if (tk[0] === 'vn') N.push([+tk[1], +tk[2], +tk[3]]);
      else if (tk[0] === 'f') {
        const f = tk.slice(1).map(s => s.split('/').map(x => (x ? +x - 1 : -1)));
        for (let i = 1; i < f.length - 1; i++) for (const v of [f[0], f[i], f[i + 1]]) {
          P.push(...V[v[0]]);
          if (v[2] >= 0 && N[v[2]]) No.push(...N[v[2]]);
        }
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(P, 3));
    if (No.length === P.length) g.setAttribute('normal', new THREE.Float32BufferAttribute(No, 3));
    else g.computeVertexNormals();
    const m = new THREE.Mesh(g, material);
    m.castShadow = true;
    parent.add(m);
  } catch (e) {
    onError(e);  // a missing mesh is reported, not silently skipped
  }
}

// The model's material names, mapped onto the token palette. This is the only
// thing the viewer still knows about the robot — the chain itself is data.

const MATERIAL_TOKEN = {
  black: '--scene-mat-black',
  jointgray: '--scene-mat-joint',
  linkgray: '--scene-mat-link',
  urblue: '--scene-mat-brand',
  carriage: '--scene-carriage',
  rail: '--scene-rail',
};

// MuJoCo hides groups 3+ in its own viewer: that is where collision geometry
// lives, and drawing it would bury the visual meshes.
const kCollisionGroup = 3;

// MuJoCo primitives → three.js geometry. `size` is half-extents, MuJoCo's
// convention; a plane is the ground, which the scene already draws.
function geometryFor(geom) {
  const [a, b, c] = geom.size;
  switch (geom.type) {
    case 'box': return new THREE.BoxGeometry(a * 2, b * 2, c * 2);
    case 'sphere': return new THREE.SphereGeometry(a, 16, 12);
    case 'cylinder': return new THREE.CylinderGeometry(a, a, b * 2, 20);
    case 'capsule': return new THREE.CapsuleGeometry(a, b * 2, 6, 16);
    default: return null;
  }
}

export function initScene(view) {
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  view.appendChild(renderer.domElement);

  new Orbit(camera, renderer.domElement, new THREE.Vector3(0.6, 0.5, -0.2), 2.6);

  const hemi = new THREE.HemisphereLight(0, 0, 0.85);
  scene.add(hemi);
  const key = new THREE.DirectionalLight(0, 1.6);  // colour comes from paint()
  key.position.set(2.5, 4.5, 3);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3, near: 0.5, far: 14 });
  scene.add(key);
  const fill = new THREE.DirectionalLight(0, 0.4);
  fill.translateX(-3);
  scene.add(fill);

  const floorMat = new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.95 });
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(12, 12), floorMat);
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);
  let grid = null;

  // MuJoCo is Z-up, the viewer Y-up.
  const zup = new THREE.Group();
  zup.rotation.x = -Math.PI / 2;
  scene.add(zup);


  const MAT = Object.fromEntries(
    Object.keys(MATERIAL_TOKEN).map(name => [
      name,
      new THREE.MeshStandardMaterial({ metalness: 0.35, roughness: 0.5 }),
    ]),
  );
  const fallbackMat = new THREE.MeshStandardMaterial({ metalness: 0.35, roughness: 0.55 });

  const meshError = e => store.patch({ error: String(e.message ?? e) });
  const joints = new Map();   // model joint name -> { group, axis, type, rest }
  let tcpMark = null;

  // Rebuild the robot from the model the physics runs (GET /model): body
  // offsets, joint axes and meshes all come from the MJCF, so the view cannot
  // drift from the simulation.
  // Everything drawable on a link comes from the model — meshes and primitives
  // alike, so a fixture added to the MJCF appears here with no viewer change.
  function drawGeoms(link, parent) {
    for (const geom of link.geoms ?? []) {
      if (geom.group >= kCollisionGroup) continue;  // collision shapes are not visuals
      const material = MAT[geom.material] ?? fallbackMat;
      if (geom.type === 'mesh') {
        loadOBJ(geom.mesh, material, parent, meshError);
        continue;
      }
      const geometry = geometryFor(geom);
      if (!geometry) continue;  // planes are the ground the scene already draws
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.set(geom.pos[0], geom.pos[1], geom.pos[2]);
      const [w, x, y, z] = geom.quat;
      mesh.quaternion.set(x, y, z, w).normalize();
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      parent.add(mesh);
    }
  }

  async function buildChain() {
    let model;
    try {
      model = await getJSON('/model');
    } catch (e) {
      meshError(e);
      return;
    }
    if (!model.links) return;

    const groups = new Map([[0, zup]]);  // body 0 is the world
    // The workpiece is reported dynamically, so its static geometry is skipped.
    const workpiece = (model.sites ?? []).find(site => site.name === model.workpiece_site);
    const skipBody = workpiece ? workpiece.body : -1;
    drawGeoms(model.links[0], zup);  // world-body furniture: floor beam, fixtures
    model.links.forEach((link, index) => {
      if (index === 0) return;  // already drawn as the scene root
      const parent = groups.get(link.parent) ?? zup;
      const fixed = new THREE.Group();
      fixed.position.set(link.pos[0], link.pos[1], link.pos[2]);
      const [w, x, y, z] = link.quat;
      fixed.quaternion.set(x, y, z, w).normalize();
      parent.add(fixed);

      const moving = new THREE.Group();
      fixed.add(moving);
      groups.set(index, moving);

      if (link.joint) {
        joints.set(link.joint, {
          group: moving,
          axis: new THREE.Vector3(...link.axis),
          type: link.joint_type,
          rest: moving.position.clone(),
        });
      }
      if (index !== skipBody) drawGeoms(link, moving);
    });

    // The tool point is a named site on a link — the model says where it is.
    const tcp = (model.sites ?? []).find(site => site.name === model.tcp_site);
    const host = tcp && groups.get(tcp.body);
    if (host && !tcpMark) {
      tcpMark = new THREE.Mesh(new THREE.SphereGeometry(0.012, 12, 12), tcpMat);
      tcpMark.position.set(tcp.pos[0], tcp.pos[1], tcp.pos[2]);
      host.add(tcpMark);
    }
    paint();
  }

  const tcpMat = new THREE.MeshStandardMaterial({});
  const partMat = new THREE.MeshStandardMaterial({});
  const partMesh = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.06), partMat);
  partMesh.castShadow = true;
  zup.add(partMesh);

  // Every colour comes from the token layer, so a theme switch repaints here.
  function paint() {
    scene.background = new THREE.Color(color('--scene-bg'));
    hemi.color.setHex(color('--scene-sky'));
    hemi.groundColor.setHex(color('--scene-ground'));
    key.color.setHex(color('--scene-key-light'));
    fill.color.setHex(color('--scene-fill-light'));
    floorMat.color.setHex(color('--scene-floor'));
    tcpMat.color.setHex(color('--scene-tcp'));
    tcpMat.emissive.setHex(color('--scene-tcp')).multiplyScalar(0.25);
    paintPart();
    for (const [name, token] of Object.entries(MATERIAL_TOKEN)) {
      MAT[name].color.setHex(color(token));
    }
    fallbackMat.color.setHex(color('--scene-mat-link'));
    if (grid) scene.remove(grid);
    grid = new THREE.GridHelper(12, 48, color('--scene-grid'), color('--scene-grid-sub'));
    scene.add(grid);
  }
  // Live pose + workpiece state. Declared before paint(), which reads them.
  let target = [];
  const shown = [];
  let part = null;
  let held = false;

  // Carried parts read differently from free ones, so a grasp is visible.
  function paintPart() {
    const name = held ? '--scene-part-held' : '--scene-part';
    partMat.color.setHex(color(name));
    partMat.emissive.setHex(color(name)).multiplyScalar(held ? 0.45 : 0.2);
  }

  paint();
  onChange(paint);
  buildChain();

  // Bind telemetry to the model's joints by NAME: the node tree carries the
  // model joint each axis drives, so nothing depends on ordering.
  let axes = [];  // [{ joint, scale }] parallel to the telemetry vectors
  store.subscribe(s => s.nodes.map(n => `${n.joint}:${n.unit}`).join('|'), (_, s) => {
    axes = s.nodes.map(n => ({ joint: n.joint, scale: n.unit === 'mm' ? 1000 : 1 }));
    shown.length = 0;
  });
  store.subscribe(s => s.pos, pos => { if (pos.length) target = pos; });
  store.subscribe(s => s.workpiece ?? s.part, p => { part = p; });
  store.subscribe(s => s.holding, h => { held = h; paintPart(); });

  function resize() {
    const w = view.clientWidth, h = view.clientHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  addEventListener('resize', resize);
  resize();

  (function loop() {
    for (let i = 0; i < target.length; i++) {
      shown[i] = (shown[i] ?? 0) + ((target[i] ?? 0) - (shown[i] ?? 0)) * 0.25;
      const axis = axes[i];
      const joint = axis && joints.get(axis.joint);
      if (!joint) continue;
      const value = shown[i] / axis.scale;  // descriptor units -> model units
      if (joint.type === 'slide') {
        joint.group.position.copy(joint.rest).addScaledVector(joint.axis, value);
      } else {
        joint.group.quaternion.setFromAxisAngle(joint.axis, value);
      }
    }
    partMesh.visible = part != null;
    if (part) partMesh.position.set(part[0], part[1], part[2]);
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
  })();
}
