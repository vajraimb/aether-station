import * as THREE from "three";

function hash(ix: number, iy: number): number {
  let n = ix * 374761393 + iy * 668265263;
  n = (n ^ (n >> 13)) * 1274126177;
  return ((n ^ (n >> 16)) >>> 0) / 4294967296;
}

function valueNoise(x: number, y: number): number {
  const x0 = Math.floor(x);
  const y0 = Math.floor(y);
  const fx = x - x0;
  const fy = y - y0;
  const u = fx * fx * (3 - 2 * fx);
  const v = fy * fy * (3 - 2 * fy);
  const a = hash(x0, y0);
  const b = hash(x0 + 1, y0);
  const c = hash(x0, y0 + 1);
  const d = hash(x0 + 1, y0 + 1);
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
}

function fbm(x: number, y: number): number {
  let s = 0;
  let a = 0.5;
  let f = 1;
  for (let i = 0; i < 5; i++) {
    s += a * valueNoise(x * f, y * f);
    a *= 0.5;
    f *= 2;
  }
  return s;
}

export function makeEarthTexture(): THREE.CanvasTexture {
  const w = 512;
  const h = 256;
  const c = document.createElement("canvas");
  c.width = w;
  c.height = h;
  const ctx = c.getContext("2d")!;
  const img = ctx.createImageData(w, h);
  for (let y = 0; y < h; y++) {
    const lat = 1 - (2 * y) / h;
    for (let x = 0; x < w; x++) {
      const lon = (x / w) * 8;
      const n = fbm(lon, lat * 4);
      const ice = Math.abs(lat) > 0.78 + n * 0.08;
      const land = n > 0.52;
      const i = (y * w + x) * 4;
      if (ice) {
        img.data[i] = 232;
        img.data[i + 1] = 238;
        img.data[i + 2] = 246;
      } else if (land) {
        img.data[i] = 90 + n * 50;
        img.data[i + 1] = 130 + n * 40;
        img.data[i + 2] = 72;
      } else {
        img.data[i] = 48 + n * 30;
        img.data[i + 1] = 118 + n * 45;
        img.data[i + 2] = 196 + n * 40;
      }
      img.data[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  tex.needsUpdate = true;
  return tex;
}

export function makeMoonTexture(): THREE.CanvasTexture {
  const w = 384;
  const h = 192;
  const c = document.createElement("canvas");
  c.width = w;
  c.height = h;
  const ctx = c.getContext("2d")!;
  const img = ctx.createImageData(w, h);
  for (let y = 0; y < h; y++) {
    const lat = 1 - (2 * y) / h;
    for (let x = 0; x < w; x++) {
      const n = fbm((x / w) * 10, lat * 5);
      const crater = n > 0.62 ? 18 : 0;
      const g = 118 + n * 50 - crater;
      const i = (y * w + x) * 4;
      img.data[i] = g;
      img.data[i + 1] = g - 2;
      img.data[i + 2] = g - 8;
      img.data[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  return tex;
}

export function makeStarPositions(n = 1600, radius = 2200): Float32Array {
  const arr = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const u = Math.random();
    const v = Math.random();
    const theta = 2 * Math.PI * u;
    const phi = Math.acos(2 * v - 1);
    const r = radius * (0.7 + Math.random() * 0.3);
    arr[i * 3] = r * Math.sin(phi) * Math.cos(theta);
    arr[i * 3 + 1] = r * Math.sin(phi) * Math.sin(theta);
    arr[i * 3 + 2] = r * Math.cos(phi);
  }
  return arr;
}
