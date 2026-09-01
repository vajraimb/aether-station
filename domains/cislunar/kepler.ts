export function hypot3(x: number, y: number, z: number): number {
  return Math.hypot(x, y, z);
}

export function eccentricAnomaly(M: number, e: number): number {
  let E = M;
  for (let i = 0; i < 12; i++) {
    const d = (E - e * Math.sin(E) - M) / (1 - e * Math.cos(E));
    E -= d;
    if (Math.abs(d) < 1e-12) break;
  }
  return E;
}

export function trueAnomaly(E: number, e: number): number {
  const s = Math.sqrt(1 + e) * Math.sin(E / 2);
  const c = Math.sqrt(1 - e) * Math.cos(E / 2);
  return 2 * Math.atan2(s, c);
}

/** Perifocal ellipse in XZ, periapsis at +X, Y up. */
export function ellipseState(
  mu: number,
  a: number,
  e: number,
  tFromPeri: number,
): { r: [number, number, number]; v: [number, number, number]; f: number } {
  const n = Math.sqrt(mu / (a * a * a));
  const M = n * tFromPeri;
  const E = eccentricAnomaly(M, e);
  const f = trueAnomaly(E, e);
  const rMag = (a * (1 - e * e)) / (1 + e * Math.cos(f));
  const cf = Math.cos(f);
  const sf = Math.sin(f);
  const r: [number, number, number] = [rMag * cf, 0, rMag * sf];
  const p = a * (1 - e * e);
  const vr = Math.sqrt(mu / p) * e * Math.sin(f);
  const vf = Math.sqrt(mu / p) * (1 + e * Math.cos(f));
  const v: [number, number, number] = [vr * cf - vf * sf, 0, vr * sf + vf * cf];
  return { r, v, f };
}

export function circularState(
  mu: number,
  radius: number,
  theta: number,
  cx: number,
  cy: number,
  cz: number,
): { r: [number, number, number]; v: [number, number, number] } {
  const w = Math.sqrt(mu / (radius * radius * radius));
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return {
    r: [cx + radius * c, cy, cz + radius * s],
    v: [-radius * w * s, 0, radius * w * c],
  };
}

export function rotateXZ(
  r: [number, number, number],
  v: [number, number, number],
  th: number,
): { r: [number, number, number]; v: [number, number, number] } {
  const c = Math.cos(th);
  const s = Math.sin(th);
  return {
    r: [r[0] * c - r[2] * s, r[1], r[0] * s + r[2] * c],
    v: [v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c],
  };
}
