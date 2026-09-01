import { useMemo, useRef, useEffect, type MutableRefObject } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Line, OrbitControls } from "@react-three/drei";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import * as THREE from "three";
import { A_EARTH, A_MARS, A_MOON, R_LEO, isHelioPhase, isLunarPhase, isMarsPhase, type Phase } from "../../domains/cislunar/constants";
import type { CislunarMission, CislunarSample } from "../../domains/cislunar/trajectory";
import {
  makeEarthTexture,
  makeMarsTexture,
  makeMoonTexture,
  makeStarPositions,
  makeSunDiskTexture,
  makeSunGlowTexture,
  useOptionalTexture,
} from "./globe-textures";
import type { CameraMode } from "./cislunar-types";

const PHASE_COLOR: Record<Phase, string> = {
  leo: "#6a92c8",
  tli: "#c88858",
  coast: "#c4a574",
  loi: "#c88858",
  llo: "#7dba9a",
  revolution: "#7dba9a",
  tmi: "#d07070",
  heliocoast: "#c88858",
  moi: "#d07070",
  lmo: "#c88858",
  marsrev: "#c88858",
};

const ORIGIN = new THREE.Vector3();
const VIS_MOON_DIST = 280;
const VIS_EARTH_R = 36;
const VIS_MOON_R = 11;
const VIS_LEO_R = 43;
const VIS_LLO_R = 14.4;
const VIS_AU = 210;
const VIS_MARS_ORBIT = VIS_AU * (A_MARS / A_EARTH);
const VIS_EARTH_HELIOS = 7.2;
const VIS_MOON_HELIOS = 2.3;
const VIS_MARS_R = 4.6;
const VIS_LMO_R = 6.8;
const WELL_RMIN = 16;
const WELL_RMAX = 400;
const WELL_AMP = 72;
const SIDEREAL_DAY = 86164.0905;
const SIDEREAL_YEAR = 365.256363 * 86400;
const OBLIQUITY = (23.44 * Math.PI) / 180;

function sunDir(t: number): THREE.Vector3 {
  const lam = 0.62 + (t / SIDEREAL_YEAR) * Math.PI * 2;
  return new THREE.Vector3(Math.cos(lam), 0, Math.sin(lam));
}

function vec3(p: readonly [number, number, number]): THREE.Vector3 {
  return new THREE.Vector3(p[0], p[1], p[2]);
}

/** Exaggerated 1/r embedding so the solar well is visible at orrery scale.
 *  Real Schwarzschild radius of the Sun is ~3 km; here the dish is teaching geometry. */
function spacetimeWell(rho: number): number {
  const r = Math.max(rho, WELL_RMIN);
  return -WELL_AMP * Math.pow(WELL_RMIN / r, 0.45);
}

function lift(p: THREE.Vector3, on: boolean): THREE.Vector3 {
  if (!on) return p;
  const rho = Math.hypot(p.x, p.z);
  p.y += spacetimeWell(rho);
  return p;
}

function visHelio(p: readonly [number, number, number]): THREE.Vector3 {
  return vec3(p).multiplyScalar(VIS_AU / A_EARTH);
}

function earthVisual(sample: CislunarSample): THREE.Vector3 {
  if (!isHelioPhase(sample.phase)) return ORIGIN.clone();
  return visHelio(sample.earthH);
}

function marsVisual(sample: CislunarSample): THREE.Vector3 {
  return visHelio(sample.mars);
}

function moonVisual(sample: CislunarSample): THREE.Vector3 {
  if (isHelioPhase(sample.phase)) {
    const earth = earthVisual(sample);
    const m = vec3(sample.moon);
    if (m.lengthSq() < 1e-8) return earth.clone().add(new THREE.Vector3(4.2, 0, 0));
    return earth.clone().add(m.setLength(4.2));
  }
  const v = vec3(sample.moon);
  if (v.lengthSq() < 1e-8) return new THREE.Vector3(VIS_MOON_DIST, 0, 0);
  return v.setLength(VIS_MOON_DIST);
}

function craftVisual(sample: CislunarSample): THREE.Vector3 {
  if (isMarsPhase(sample.phase)) {
    const mars = marsVisual(sample);
    const rel = vec3(sample.r).sub(vec3(sample.mars));
    if (rel.lengthSq() < 1e-8) return mars.clone().add(new THREE.Vector3(VIS_LMO_R, 0, 0));
    return mars.clone().add(rel.setLength(VIS_LMO_R));
  }
  if (isHelioPhase(sample.phase)) {
    const r = visHelio(sample.r);
    if (r.lengthSq() < 1e-8) return new THREE.Vector3(VIS_AU, 0, 0);
    return r;
  }
  const moon = moonVisual(sample);
  const r = vec3(sample.r);
  if (sample.phase === "leo" || sample.phase === "tli") {
    if (r.lengthSq() < 1e-8) return new THREE.Vector3(VIS_LEO_R, 0, 0);
    return r.setLength(VIS_LEO_R);
  }
  if (isLunarPhase(sample.phase)) {
    const rel = vec3(sample.r).sub(vec3(sample.moon));
    if (rel.lengthSq() < 1e-8) return moon.clone().add(new THREE.Vector3(VIS_LLO_R, 0, 0));
    return moon.clone().add(rel.setLength(VIS_LLO_R));
  }
  const distPhys = r.length();
  const u = Math.min(1, Math.max(0, (distPhys - R_LEO) / (A_MOON - R_LEO)));
  const dist = VIS_LEO_R + u * (VIS_MOON_DIST - VIS_LLO_R - VIS_LEO_R);
  if (r.lengthSq() < 1e-8) return new THREE.Vector3(dist, 0, 0);
  return r.setLength(dist);
}

function SpacetimeFabric() {
  const { surface, wires } = useMemo(() => {
    const nr = 40;
    const nth = 96;
    const positions: number[] = [];
    const indices: number[] = [];
    for (let i = 0; i <= nr; i++) {
      const rho = WELL_RMIN + (i / nr) * (WELL_RMAX - WELL_RMIN);
      const y = spacetimeWell(rho);
      for (let j = 0; j <= nth; j++) {
        const th = (j / nth) * Math.PI * 2;
        positions.push(rho * Math.cos(th), y, rho * Math.sin(th));
      }
    }
    const cols = nth + 1;
    for (let i = 0; i < nr; i++) {
      for (let j = 0; j < nth; j++) {
        const a = i * cols + j;
        const b = a + 1;
        const c = a + cols;
        const d = c + 1;
        indices.push(a, c, b, b, c, d);
      }
    }
    const surface = new THREE.BufferGeometry();
    surface.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    surface.setIndex(indices);
    surface.computeVertexNormals();
    const wires = new THREE.WireframeGeometry(surface);
    return { surface, wires };
  }, []);
  useEffect(
    () => () => {
      surface.dispose();
      wires.dispose();
    },
    [surface, wires],
  );
  return (
    <group>
      <mesh geometry={surface}>
        <meshStandardMaterial
          color="#243044"
          transparent
          opacity={0.42}
          metalness={0.05}
          roughness={0.92}
          side={THREE.DoubleSide}
          depthWrite={false}
        />
      </mesh>
      <lineSegments geometry={wires}>
        <lineBasicMaterial color="#6a92c8" transparent opacity={0.28} />
      </lineSegments>
    </group>
  );
}

function Stars() {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(makeStarPositions(), 3));
    return g;
  }, []);
  return (
    <points geometry={geom}>
      <pointsMaterial color="#e8eaef" size={2.2} sizeAttenuation={false} />
    </points>
  );
}

function AxisTriplet({ scale = 2.4, opacity = 1 }: { scale?: number; opacity?: number }) {
  const s = scale;
  const o: [number, number, number] = [0, 0, 0];
  return (
    <group>
      <Line points={[o, [s, 0, 0]]} color="#d07070" lineWidth={2} transparent opacity={opacity} />
      <Line points={[o, [0, s, 0]]} color="#70b080" lineWidth={2} transparent opacity={opacity} />
      <Line points={[o, [0, 0, s]]} color="#6a92c8" lineWidth={2} transparent opacity={opacity} />
    </group>
  );
}

function Atmosphere({ radius }: { radius: number }) {
  const mat = useMemo(
    () =>
      new THREE.ShaderMaterial({
        transparent: true,
        blending: THREE.AdditiveBlending,
        side: THREE.BackSide,
        depthWrite: false,
        uniforms: {},
        vertexShader: `
          varying vec3 vNormalW;
          varying vec3 vViewW;
          void main() {
            vec4 w = modelMatrix * vec4(position, 1.0);
            vNormalW = normalize(mat3(modelMatrix) * normal);
            vViewW = cameraPosition - w.xyz;
            gl_Position = projectionMatrix * viewMatrix * w;
          }
        `,
        fragmentShader: `
          varying vec3 vNormalW;
          varying vec3 vViewW;
          void main() {
            float f = pow(1.0 - abs(dot(normalize(vNormalW), normalize(vViewW))), 2.6);
            gl_FragColor = vec4(0.32, 0.58, 1.0, f * 0.55);
          }
        `,
      }),
    [],
  );
  useEffect(() => () => mat.dispose(), [mat]);
  return (
    <mesh scale={1.045}>
      <sphereGeometry args={[radius, 64, 48]} />
      <primitive object={mat} attach="material" />
    </mesh>
  );
}

function Earth({
  t,
  radius = VIS_EARTH_R,
  position,
}: {
  t: number;
  radius?: number;
  position?: THREE.Vector3;
}) {
  const day = useOptionalTexture("/textures/earth-day.jpg", true);
  const night = useOptionalTexture("/textures/earth-night.png", true);
  const clouds = useOptionalTexture("/textures/earth-clouds.png", false);
  const spec = useOptionalTexture("/textures/earth-specular.jpg", false);
  const normal = useOptionalTexture("/textures/earth-normal.jpg", false);
  const fallback = useMemo(() => makeEarthTexture(), []);
  const r = radius;
  const spin = (t / SIDEREAL_DAY) * Math.PI * 2;

  return (
    <group position={position ?? ORIGIN}>
      <group rotation={[0, 0, OBLIQUITY]}>
        <group rotation={[0, spin, 0]}>
          <mesh>
            <sphereGeometry args={[r, 96, 64]} />
            <meshStandardMaterial
              key={night ? "earth-lit" : "earth-plain"}
              map={day ?? fallback}
              normalMap={normal ?? undefined}
              normalScale={[0.55, 0.55]}
              metalnessMap={spec ?? undefined}
              metalness={spec ? 0.22 : 0.08}
              roughness={0.48}
              emissiveMap={night ?? undefined}
              emissive={night ? "#ffcc88" : "#1b4f86"}
              emissiveIntensity={night ? 1.15 : 0.22}
              onBeforeCompile={(shader) => {
                shader.fragmentShader = shader.fragmentShader.replace(
                  "#include <lights_physical_fragment>",
                  `#ifdef USE_EMISSIVEMAP
                   {
                     vec3 lightDir = directionalLights[0].direction;
                     float nightFactor = 1.0 - smoothstep(-0.05, 0.28, dot(normal, lightDir));
                     totalEmissiveRadiance *= nightFactor;
                   }
                   #endif
                   #include <lights_physical_fragment>`,
                );
              }}
            />
          </mesh>
          {clouds && (
            <mesh rotation={[0, spin * 0.04, 0]} scale={1.008}>
              <sphereGeometry args={[r, 64, 48]} />
              <meshBasicMaterial
                map={clouds}
                transparent
                opacity={0.7}
                depthWrite={false}
                blending={THREE.AdditiveBlending}
              />
            </mesh>
          )}
        </group>
        {r > 12 && <Atmosphere radius={r} />}
      </group>
    </group>
  );
}

function Moon({
  pos,
  face,
  radius = VIS_MOON_R,
}: {
  pos: THREE.Vector3;
  face: THREE.Vector3;
  radius?: number;
}) {
  const map = useOptionalTexture("/textures/moon.jpg", true);
  const fallback = useMemo(() => makeMoonTexture(), []);
  const r = radius;
  const ref = useRef<THREE.Group>(null);
  useFrame(() => {
    if (!ref.current) return;
    ref.current.position.copy(pos);
    ref.current.lookAt(face);
  });
  return (
    <group ref={ref}>
      <mesh rotation={[0, Math.PI, 0]}>
        <sphereGeometry args={[r, 64, 48]} />
        <meshStandardMaterial
          map={map ?? fallback}
          bumpMap={map ?? fallback}
          bumpScale={0.45}
          roughness={1}
          metalness={0}
        />
      </mesh>
    </group>
  );
}

function Mars({ pos, t, radius = VIS_MARS_R }: { pos: THREE.Vector3; t: number; radius?: number }) {
  const map = useOptionalTexture("/textures/mars.jpg", true);
  const fallback = useMemo(() => makeMarsTexture(), []);
  const sol = 88642.663;
  const spin = (t / sol) * Math.PI * 2;
  return (
    <group position={pos} rotation={[0, spin, 0]}>
      <mesh>
        <sphereGeometry args={[radius, 96, 64]} />
        <meshStandardMaterial
          map={map ?? fallback}
          bumpMap={map ?? fallback}
          bumpScale={0.35}
          roughness={0.9}
          metalness={0}
        />
      </mesh>
    </group>
  );
}

function makeBeaconTexture(): THREE.CanvasTexture {
  const c = document.createElement("canvas");
  c.width = 64;
  c.height = 64;
  const g = c.getContext("2d")!;
  const grd = g.createRadialGradient(32, 32, 1, 32, 32, 30);
  grd.addColorStop(0, "rgba(255,246,230,1)");
  grd.addColorStop(0.18, "rgba(232,176,122,0.95)");
  grd.addColorStop(0.45, "rgba(200,136,88,0.45)");
  grd.addColorStop(1, "rgba(200,136,88,0)");
  g.fillStyle = grd;
  g.fillRect(0, 0, 64, 64);
  const tex = new THREE.CanvasTexture(c);
  tex.needsUpdate = true;
  return tex;
}

function CraftBeacon({ position }: { position: THREE.Vector3 }) {
  const map = useMemo(() => makeBeaconTexture(), []);
  const ref = useRef<THREE.Sprite>(null);
  useFrame(({ camera }) => {
    if (!ref.current) return;
    ref.current.position.copy(position);
    const dist = camera.position.distanceTo(position);
    const s = Math.max(1.6, Math.min(18, dist * 0.022));
    ref.current.scale.setScalar(s);
  });
  return (
    <sprite ref={ref}>
      <spriteMaterial map={map} transparent depthWrite={false} blending={THREE.AdditiveBlending} />
    </sprite>
  );
}

function attitudeOf(sample: CislunarSample): THREE.Quaternion {
  const burning =
    sample.phase === "tli" ||
    sample.phase === "loi" ||
    sample.phase === "tmi" ||
    sample.phase === "moi";
  const v = vec3(sample.v);
  if (v.lengthSq() < 1e-10) v.set(0, 0, 1);
  else v.normalize();
  const m = new THREE.Matrix4();
  if (burning) {
    const r = vec3(sample.r);
    const primary = isMarsPhase(sample.phase)
      ? vec3(sample.mars)
      : isLunarPhase(sample.phase)
        ? vec3(sample.moon)
        : ORIGIN.clone();
    const radial = r.sub(primary);
    if (radial.lengthSq() < 1e-10) radial.set(1, 0, 0);
    else radial.normalize();
    const z = v;
    const y = new THREE.Vector3().crossVectors(radial, z);
    if (y.lengthSq() < 1e-10) y.set(0, 1, 0);
    else y.normalize();
    const x = new THREE.Vector3().crossVectors(y, z).normalize();
    m.makeBasis(x, y, z);
  } else if (isHelioPhase(sample.phase)) {
    const x = vec3(sample.r).normalize();
    const up = new THREE.Vector3(0, 1, 0);
    const z = new THREE.Vector3().crossVectors(x, up);
    if (z.lengthSq() < 1e-10) z.set(0, 0, 1);
    else z.normalize();
    const y = new THREE.Vector3().crossVectors(z, x).normalize();
    m.makeBasis(x, y, z);
  } else {
    const x = sunDir(sample.t);
    const up = new THREE.Vector3(0, 1, 0);
    const z = new THREE.Vector3().crossVectors(x, up);
    if (z.lengthSq() < 1e-10) z.set(0, 0, 1);
    else z.normalize();
    const y = new THREE.Vector3().crossVectors(z, x).normalize();
    m.makeBasis(x, y, z);
  }
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

function Sun({ t, helio, well = false }: { t: number; helio: boolean; well?: boolean }) {
  const dir = sunDir(t);
  const pos = helio
    ? new THREE.Vector3(0, well ? spacetimeWell(22) : 0, 0)
    : dir.clone().multiplyScalar(720);
  const fill = dir.clone().multiplyScalar(-90);
  const disk = useMemo(() => makeSunDiskTexture(), []);
  const glow = useMemo(() => makeSunGlowTexture(), []);
  const core = helio ? 14 : 9.5;
  const mid = helio ? 32 : 22;
  const outer = helio ? 70 : 56;
  return (
    <>
      <ambientLight intensity={helio ? 0.04 : 0.05} />
      <hemisphereLight args={["#b7c8de", "#0a0908", helio ? 0.08 : 0.12]} />
      {helio ? (
        <>
          <pointLight position={[0, 0, 0]} intensity={4.2} decay={0} color="#fff1c8" />
          <directionalLight position={[0.2, 0.05, 0]} intensity={0.05} color="#fff1c8" />
        </>
      ) : (
        <>
          <directionalLight position={pos} intensity={3.4} color="#fff1c8" />
          <directionalLight position={fill} intensity={0.07} color="#6a7fa0" />
        </>
      )}
      <sprite position={pos} scale={[outer, outer, 1]} renderOrder={-2}>
        <spriteMaterial
          map={glow}
          transparent
          depthWrite={false}
          blending={THREE.AdditiveBlending}
          opacity={0.85}
        />
      </sprite>
      <sprite position={pos} scale={[mid, mid, 1]} renderOrder={-1}>
        <spriteMaterial
          map={glow}
          transparent
          depthWrite={false}
          blending={THREE.AdditiveBlending}
          color="#fff4c8"
          opacity={0.9}
        />
      </sprite>
      <sprite position={pos} scale={[core, core, 1]} renderOrder={0}>
        <spriteMaterial map={disk} transparent depthWrite={false} />
      </sprite>
    </>
  );
}

/** CSM-style probe. Cruise: solar arrays (+X) face the sun. Burns: +Z prograde. */
export function CraftModel({
  sample,
  showAxes = true,
  scale = 1,
}: {
  sample: CislunarSample;
  showAxes?: boolean;
  scale?: number;
}) {
  const burning =
    sample.phase === "tli" ||
    sample.phase === "loi" ||
    sample.phase === "tmi" ||
    sample.phase === "moi";
  return (
    <group quaternion={attitudeOf(sample)} scale={scale}>
      <mesh position={[0, 0, 0.18]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.22, 0.22, 0.72, 28]} />
        <meshStandardMaterial color="#9aa4b2" metalness={0.62} roughness={0.32} />
      </mesh>
      {[-0.36, 0.36].map((z) => (
        <mesh key={z} position={[0, 0, z]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.225, 0.225, 0.04, 28]} />
          <meshStandardMaterial color="#7c8796" metalness={0.5} roughness={0.4} />
        </mesh>
      ))}
      <mesh position={[0, 0, 0.68]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.09, 0.22, 0.28, 24]} />
        <meshStandardMaterial color="#c5ccd8" metalness={0.45} roughness={0.35} />
      </mesh>
      <mesh position={[0, 0, 0.86]}>
        <sphereGeometry args={[0.09, 16, 12]} />
        <meshStandardMaterial color="#e8eaef" metalness={0.3} roughness={0.28} />
      </mesh>
      <mesh position={[0, 0, -0.28]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.2, 0.2, 0.05, 28]} />
        <meshStandardMaterial color="#5c6572" metalness={0.7} roughness={0.25} />
      </mesh>
      <mesh position={[0, 0, -0.48]} rotation={[Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.16, 0.32, 20]} />
        <meshStandardMaterial
          color="#6a7180"
          metalness={0.55}
          roughness={0.35}
          emissive={burning ? "#c88858" : "#000000"}
          emissiveIntensity={burning ? 0.85 : 0}
        />
      </mesh>
      {burning && (
        <mesh position={[0, 0, -0.92]} rotation={[Math.PI / 2, 0, 0]}>
          <coneGeometry args={[0.14, 0.7, 16]} />
          <meshBasicMaterial color="#e8b07a" transparent opacity={0.78} />
        </mesh>
      )}
      <mesh position={[0.72, 0, 0.12]}>
        <boxGeometry args={[0.95, 0.02, 0.38]} />
        <meshStandardMaterial
          color="#6a92c8"
          metalness={0.35}
          roughness={0.28}
          emissive="#6a92c8"
          emissiveIntensity={0.28}
        />
      </mesh>
      <mesh position={[-0.72, 0, 0.12]}>
        <boxGeometry args={[0.95, 0.02, 0.38]} />
        <meshStandardMaterial
          color="#6a92c8"
          metalness={0.35}
          roughness={0.28}
          emissive="#6a92c8"
          emissiveIntensity={0.28}
        />
      </mesh>
      <mesh position={[0.22, 0.18, 0.22]} rotation={[0.4, 0.2, 0]}>
        <cylinderGeometry args={[0.07, 0.07, 0.02, 16]} />
        <meshStandardMaterial color="#c5ccd8" metalness={0.6} roughness={0.25} />
      </mesh>
      {[
        [0.22, 0.22, 0.42],
        [-0.22, 0.22, 0.42],
        [0.22, -0.22, 0.42],
        [-0.22, -0.22, 0.42],
      ].map((p, i) => (
        <mesh key={i} position={p as [number, number, number]}>
          <boxGeometry args={[0.04, 0.04, 0.06]} />
          <meshStandardMaterial color="#8a919c" metalness={0.5} roughness={0.4} />
        </mesh>
      ))}
      <mesh position={[0.24, 0, 0.55]}>
        <sphereGeometry args={[0.025, 8, 8]} />
        <meshBasicMaterial color="#d07070" />
      </mesh>
      <mesh position={[-0.24, 0, 0.55]}>
        <sphereGeometry args={[0.025, 8, 8]} />
        <meshBasicMaterial color="#70b080" />
      </mesh>
      {showAxes && <AxisTriplet scale={1.6} />}
    </group>
  );
}

function Probe({ sample, well = false }: { sample: CislunarSample; well?: boolean }) {
  const p = lift(craftVisual(sample), well && isHelioPhase(sample.phase));
  const v = vec3(sample.v);
  const vDir = v.lengthSq() > 1e-10 ? v.clone().normalize() : new THREE.Vector3(0, 0, 1);
  const vTip: [number, number, number] = [vDir.x * 2.4, vDir.y * 2.4, vDir.z * 2.4];
  return (
    <group position={p}>
      <CraftModel sample={sample} scale={1.15} />
      <Line points={[[0, 0, 0], vTip]} color="#c4a574" lineWidth={2} />
    </group>
  );
}

function Trajectory({ mission, helio, well }: { mission: CislunarMission; helio: boolean; well: boolean }) {
  const segments = useMemo(() => {
    const byPhase: { phase: Phase; pts: [number, number, number][] }[] = [];
    let cur: Phase | null = null;
    let pts: [number, number, number][] = [];
    for (const s of mission.samples) {
      if (s.phase === "revolution" || s.phase === "marsrev") continue;
      if (isHelioPhase(s.phase) !== helio) continue;
      if (s.phase !== cur) {
        if (cur && pts.length > 1) byPhase.push({ phase: cur, pts });
        cur = s.phase;
        pts = [];
      }
      pts.push(lift(craftVisual(s), well && helio).toArray() as [number, number, number]);
    }
    if (cur && pts.length > 1) byPhase.push({ phase: cur, pts });
    return byPhase;
  }, [mission, helio, well]);
  return (
    <group>
      {segments.map((seg, i) => (
        <Line key={i} points={seg.pts} color={PHASE_COLOR[seg.phase]} lineWidth={2.2} transparent opacity={0.92} />
      ))}
    </group>
  );
}

function OrbitRing({
  radius,
  color,
  opacity = 0.4,
  well = false,
}: {
  radius: number;
  color: string;
  opacity?: number;
  well?: boolean;
}) {
  const pts = useMemo(() => {
    const n = 128;
    const y = well ? spacetimeWell(radius) : 0;
    const out: [number, number, number][] = [];
    for (let i = 0; i <= n; i++) {
      const th = (i / n) * Math.PI * 2;
      out.push([radius * Math.cos(th), y, radius * Math.sin(th)]);
    }
    return out;
  }, [radius, well]);
  return <Line points={pts} color={color} lineWidth={1} transparent opacity={opacity} />;
}

function LunarOrbit({ moon }: { moon: THREE.Vector3 }) {
  return (
    <group position={moon}>
      <OrbitRing radius={VIS_LLO_R} color="#7dba9a" opacity={0.7} />
    </group>
  );
}

function MarsOrbitRing({ mars }: { mars: THREE.Vector3 }) {
  return (
    <group position={mars}>
      <OrbitRing radius={VIS_LMO_R} color="#c88858" opacity={0.7} />
    </group>
  );
}

function modeFraming(
  mode: CameraMode,
  sample: CislunarSample,
  well: boolean,
): { target: THREE.Vector3; position: THREE.Vector3 } {
  const helio = isHelioPhase(sample.phase);
  const apply = (p: THREE.Vector3) => lift(p, well && helio);
  const craft = apply(craftVisual(sample));
  const moon = apply(moonVisual(sample));
  const earth = apply(earthVisual(sample));
  const mars = apply(marsVisual(sample));
  const earthR = helio ? VIS_EARTH_HELIOS : VIS_EARTH_R;
  const moonR = helio ? VIS_MOON_HELIOS : VIS_MOON_R;
  if (mode === "earth") {
    return {
      target: earth.clone(),
      position: earth.clone().add(new THREE.Vector3(earthR * 2.4, earthR * 1.15, earthR * 2.1)),
    };
  }
  if (mode === "moon") {
    const away = moon.clone().sub(earth);
    if (away.lengthSq() < 1e-8) away.set(1, 0, 0);
    else away.normalize();
    return {
      target: moon.clone(),
      position: moon.clone().add(away.multiplyScalar(moonR * 4.6)).add(new THREE.Vector3(0, moonR * 2.0, 0)),
    };
  }
  if (mode === "mars") {
    const away = mars.clone().normalize();
    return {
      target: mars.clone(),
      position: mars.clone().add(away.multiplyScalar(VIS_MARS_R * 5.2)).add(new THREE.Vector3(0, VIS_MARS_R * 2.4, 0)),
    };
  }
  if (mode === "craft") {
    const v = vec3(sample.v);
    if (v.lengthSq() < 1e-10) v.set(0, 0, 1);
    else v.normalize();
    const back = helio && !isMarsPhase(sample.phase) ? 12 : 5.4;
    return {
      target: craft.clone(),
      position: craft.clone().add(v.multiplyScalar(-back)).add(new THREE.Vector3(0, 2.2, 1.6)),
    };
  }
  if (helio) {
    if (well) {
      return {
        target: new THREE.Vector3(0, spacetimeWell(140), 0),
        position: new THREE.Vector3(40, 95, 390),
      };
    }
    return {
      target: ORIGIN.clone(),
      position: new THREE.Vector3(40, 210, 360),
    };
  }
  const mid = moon.clone().multiplyScalar(0.46);
  return {
    target: mid,
    position: mid.clone().add(new THREE.Vector3(18, 110, 200)),
  };
}

function CameraRig({
  sample,
  mode,
  offset,
  well,
}: {
  sample: CislunarSample;
  mode: CameraMode;
  offset: MutableRefObject<THREE.Vector3>;
  well: boolean;
}) {
  const { camera, controls } = useThree();
  const lastMode = useRef<CameraMode | null>(null);
  const lastHelio = useRef<boolean | null>(null);
  const lastWell = useRef<boolean | null>(null);
  const snap = useRef(1);
  const craftFollow = mode === "craft";
  const helio = isHelioPhase(sample.phase);

  useFrame((_, dt) => {
    const ctrl = controls as OrbitControlsImpl | null;
    if (!ctrl) return;
    const frame = modeFraming(mode, sample, well);
    if (lastMode.current !== mode || lastHelio.current !== helio || lastWell.current !== well) {
      lastMode.current = mode;
      lastHelio.current = helio;
      lastWell.current = well;
      snap.current = 1;
      offset.current.copy(frame.position).sub(frame.target);
    }
    const d = Math.min(dt, 0.08);
    if (snap.current > 0) {
      snap.current = Math.max(0, snap.current - d * 1.15);
      const k = 1 - Math.exp(-3.4 * d);
      camera.position.lerp(frame.position, k);
      ctrl.target.lerp(frame.target, k);
      ctrl.update();
      offset.current.copy(camera.position).sub(ctrl.target);
    } else if (craftFollow) {
      ctrl.target.copy(frame.target);
      camera.position.copy(frame.target).add(offset.current);
      ctrl.update();
    }
    camera.near = 0.08;
    camera.far = 4000;
    camera.updateProjectionMatrix();
  });
  return null;
}

function SceneBody({
  mission,
  sample,
  mode,
  well,
}: {
  mission: CislunarMission;
  sample: CislunarSample;
  mode: CameraMode;
  well: boolean;
}) {
  const helio = isHelioPhase(sample.phase);
  const drape = well && helio;
  const moonPos = lift(moonVisual(sample), drape);
  const earthPos = lift(earthVisual(sample), drape);
  const marsPos = lift(marsVisual(sample), drape);
  const craftPos = lift(craftVisual(sample), drape);
  const controlsRef = useRef<OrbitControlsImpl>(null);
  const offset = useRef(new THREE.Vector3());

  return (
    <>
      <color attach="background" args={["#07080c"]} />
      <Sun t={sample.t} helio={helio} well={drape} />
      <Stars />
      {helio ? (
        <>
          {drape ? <SpacetimeFabric /> : <gridHelper args={[900, 18, "#1c2230", "#12161f"]} />}
          <Earth t={sample.t} radius={VIS_EARTH_HELIOS} position={earthPos} />
          <Moon pos={moonPos} face={earthPos} radius={VIS_MOON_HELIOS} />
          <Mars pos={marsPos} t={sample.t} />
          <OrbitRing radius={VIS_AU} color="#6a92c8" opacity={0.35} well={drape} />
          <OrbitRing radius={VIS_MARS_ORBIT} color="#c88858" opacity={0.4} well={drape} />
          {isMarsPhase(sample.phase) && <MarsOrbitRing mars={marsPos} />}
        </>
      ) : (
        <>
          <gridHelper args={[720, 18, "#1c2230", "#12161f"]} />
          <Earth t={sample.t} />
          <Moon pos={moonPos} face={ORIGIN} />
          <OrbitRing radius={VIS_MOON_DIST} color="#6d7686" opacity={0.4} />
          <LunarOrbit moon={moonPos} />
        </>
      )}
      <Trajectory mission={mission} helio={helio} well={drape} />
      <Probe sample={sample} well={drape} />
      <CraftBeacon position={craftPos} />
      <CameraRig sample={sample} mode={mode} offset={offset} well={well} />
      <OrbitControls
        ref={controlsRef}
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={3.2}
        maxDistance={1400}
        zoomSpeed={1.15}
        rotateSpeed={0.72}
        panSpeed={0.55}
        enablePan
        onStart={() => {
          const ctrl = controlsRef.current;
          if (!ctrl) return;
          offset.current.copy(ctrl.object.position).sub(ctrl.target);
        }}
        onChange={() => {
          const ctrl = controlsRef.current;
          if (!ctrl) return;
          offset.current.copy(ctrl.object.position).sub(ctrl.target);
        }}
      />
    </>
  );
}

export function CislunarCanvas({
  mission,
  sample,
  mode,
  well,
}: {
  mission: CislunarMission;
  sample: CislunarSample;
  mode: CameraMode;
  well: boolean;
}) {
  return (
    <Canvas
      dpr={[1, 2]}
      gl={{ antialias: true, alpha: false, preserveDrawingBuffer: true, powerPreference: "high-performance" }}
      camera={{ position: [150, 120, 220], fov: 46, near: 0.1, far: 4000 }}
      style={{ width: "100%", height: "100%", background: "#07080c", touchAction: "none" }}
    >
      <SceneBody mission={mission} sample={sample} mode={mode} well={well} />
    </Canvas>
  );
}

