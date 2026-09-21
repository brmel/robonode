import * as THREE from './three.module.min.js';

// Drag to orbit, wheel to zoom, right/shift-drag to pan.
export class Orbit {
  constructor(camera, dom, target, radius) {
    this.cam = camera; this.target = target.clone();
    this.r = radius; this.theta = 0.7; this.phi = 1.05; this.drag = null;
    dom.addEventListener('contextmenu', e => e.preventDefault());
    dom.addEventListener('pointerdown', e => { this.drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey }; });
    addEventListener('pointerup', () => { this.drag = null; });
    addEventListener('pointermove', e => this.move(e));
    dom.addEventListener('wheel', e => { e.preventDefault(); this.r = THREE.MathUtils.clamp(this.r * (1 + Math.sign(e.deltaY) * 0.1), 0.6, 12); this.update(); }, { passive: false });
    this.update();
  }
  move(e) {
    if (!this.drag) return;
    const dx = e.clientX - this.drag.x, dy = e.clientY - this.drag.y;
    this.drag.x = e.clientX; this.drag.y = e.clientY;
    if (this.drag.pan) {
      const s = this.r * 0.0016;
      const right = new THREE.Vector3().setFromMatrixColumn(this.cam.matrix, 0);
      const up = new THREE.Vector3().setFromMatrixColumn(this.cam.matrix, 1);
      this.target.addScaledVector(right, -dx * s).addScaledVector(up, dy * s);
    } else {
      this.theta -= dx * 0.006;
      this.phi = THREE.MathUtils.clamp(this.phi - dy * 0.006, 0.15, Math.PI / 2 - 0.02);
    }
    this.update();
  }
  update() {
    const sp = Math.sin(this.phi);
    this.cam.position.set(
      this.target.x + this.r * sp * Math.sin(this.theta),
      this.target.y + this.r * Math.cos(this.phi),
      this.target.z + this.r * sp * Math.cos(this.theta));
    this.cam.lookAt(this.target);
  }
}

// Minimal OBJ → BufferGeometry (v / vn / f, triangulated).
