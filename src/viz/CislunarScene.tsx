import { useMemo, useRef, useEffect, type MutableRefObject } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Line, OrbitControls } from "@react-three/drei";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import * as THREE from "three";
import { A_MOON, R_LEO, isLunarPhase, type Phase } from "../../domains/cislunar/constants";
import type { CislunarMission, CislunarSample } from "../../domains/cislunar/trajectory";
import {
  makeEarthTexture,
  makeMoonTexture,
  makeStarPositions,
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
};

const ORIGIN = new THREE.Vector3();
const VIS_MOON_DIST = 280;
const VIS_EARTH_R = 36;
const VIS_MOON_R = 11;
const VIS_LEO_R = 43;
const VIS_LLO_R = 14.4;
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

function moonVisual(moon: readonly [number, number, number]): THREE.Vector3 {
  const v = vec3(moon);
  if (v.lengthSq() < 1e-8) return new THREE.Vector3(VIS_MOON_DIST, 0, 0);
  return v.setLength(VIS_MOON_DIST);
}

function craftVisual(sample: CislunarSample): THREE.Vector3 {
  const moon = moonVisual(sample.moon);
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

function Earth({ t }: { t: number }) {
  const day = useOptionalTexture("/textures/earth-day.jpg", true);
  const night = useOptionalTexture("/textures/earth-night.png", true);
  const clouds = useOptionalTexture("/textures/earth-clouds.png", false);
  const spec = useOptionalTexture("/textures/earth-specular.jpg", false);
  const normal = useOptionalTexture("/textures/earth-normal.jpg", false);
  const fallback = useMemo(() => makeEarthTexture(), []);
  const r = VIS_EARTH_R;
  const spin = (t / SIDEREAL_DAY) * Math.PI * 2;

  return (
    <group rotation={[0, 0, OBLIQUITY]}>
      <group rotation={[0, spin, 0]}>
        <mesh>
          <sphereGeometry args={[r, 128, 96]} />
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
            <sphereGeometry args={[r, 96, 64]} />
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
      <Atmosphere radius={r} />
    </group>
  );
}

function Moon({ pos }: { pos: THREE.Vector3 }) {
  const map = useOptionalTexture("/textures/moon.jpg", true);
  const fallback = useMemo(() => makeMoonTexture(), []);
  const r = VIS_MOON_R;
  const ref = useRef<THREE.Group>(null);
  useFrame(() => {
    if (!ref.current) return;
    ref.current.position.copy(pos);
    ref.current.lookAt(0, 0, 0);
  });
  return (
    <group ref={ref}>
      <mesh rotation={[0, Math.PI, 0]}>
        <sphereGeometry args={[r, 96, 64]} />
        <meshStandardMaterial
          map={map ?? fallback}
          bumpMap={map ?? fallback}
          bumpScale={0.55}
          roughness={1}
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
  const burning = sample.phase === "tli" || sample.phase === "loi";
  const v = vec3(sample.v);
  if (v.lengthSq() < 1e-10) v.set(0, 0, 1);
  else v.normalize();
  const m = new THREE.Matrix4();
  if (burning) {
    const r = vec3(sample.r);
    const primary = isLunarPhase(sample.phase) ? vec3(sample.moon) : ORIGIN.clone();
    const radial = r.sub(primary);
    if (radial.lengthSq() < 1e-10) radial.set(1, 0, 0);
    else radial.normalize();
    const z = v;
    const y = new THREE.Vector3().crossVectors(radial, z);
    if (y.lengthSq() < 1e-10) y.set(0, 1, 0);
    else y.normalize();
    const x = new THREE.Vector3().crossVectors(y, z).normalize();
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

function Sun({ t }: { t: number }) {
  const dir = sunDir(t);
  const pos = dir.clone().multiplyScalar(420);
  const fill = dir.clone().multiplyScalar(-80);
  return (
    <>
      <ambientLight intensity={0.05} />
      <hemisphereLight args={["#b7c8de", "#0a0908", 0.12]} />
      <directionalLight position={pos} intensity={3.4} color="#fff1c8" />
      <directionalLight position={fill} intensity={0.07} color="#6a7fa0" />
      <mesh position={pos}>
        <sphereGeometry args={[5.5, 16, 12]} />
        <meshBasicMaterial color="#fff6d0" />
      </mesh>
      <sprite position={pos} scale={[28, 28, 1]}>
        <spriteMaterial color="#ffe29a" transparent opacity={0.55} depthWrite={false} blending={THREE.AdditiveBlending} />
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
  const burning = sample.phase === "tli" || sample.phase === "loi";
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

function Probe({ sample }: { sample: CislunarSample }) {
  const p = craftVisual(sample);
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

function Trajectory({ mission }: { mission: CislunarMission }) {
  const segments = useMemo(() => {
    const byPhase: { phase: Phase; pts: [number, number, number][] }[] = [];
    let cur: Phase | null = null;
    let pts: [number, number, number][] = [];
    for (const s of mission.samples) {
      if (s.phase === "revolution") continue;
      if (s.phase !== cur) {
        if (cur && pts.length > 1) byPhase.push({ phase: cur, pts });
        cur = s.phase;
        pts = [];
      }
      pts.push(craftVisual(s).toArray() as [number, number, number]);
    }
    if (cur && pts.length > 1) byPhase.push({ phase: cur, pts });
    return byPhase;
  }, [mission]);
  return (
    <group>
      {segments.map((seg, i) => (
        <Line key={i} points={seg.pts} color={PHASE_COLOR[seg.phase]} lineWidth={2.2} transparent opacity={0.92} />
      ))}
    </group>
  );
}

function LunarOrbit({ moon }: { moon: THREE.Vector3 }) {
  const pts = useMemo(() => {
    const n = 64;
    const r = VIS_LLO_R;
    const out: [number, number, number][] = [];
    for (let i = 0; i <= n; i++) {
      const th = (i / n) * Math.PI * 2;
      out.push([r * Math.cos(th), 0, r * Math.sin(th)]);
    }
    return out;
  }, []);
  return (
    <group position={moon}>
      <Line points={pts} color="#7dba9a" lineWidth={1.4} transparent opacity={0.7} />
    </group>
  );
}

function MoonOrbit() {
  const pts = useMemo(() => {
    const n = 128;
    const r = VIS_MOON_DIST;
    const out: [number, number, number][] = [];
    for (let i = 0; i <= n; i++) {
      const th = (i / n) * Math.PI * 2;
      out.push([r * Math.cos(th), 0, r * Math.sin(th)]);
    }
    return out;
  }, []);
  return <Line points={pts} color="#6d7686" lineWidth={1} transparent opacity={0.4} />;
}

function modeFraming(mode: CameraMode, sample: CislunarSample): { target: THREE.Vector3; position: THREE.Vector3 } {
  const craft = craftVisual(sample);
  const moon = moonVisual(sample.moon);
  if (mode === "earth") {
    return {
      target: ORIGIN.clone(),
      position: new THREE.Vector3(VIS_EARTH_R * 2.4, VIS_EARTH_R * 1.15, VIS_EARTH_R * 2.1),
    };
  }
  if (mode === "moon") {
    const away = moon.clone().normalize();
    return {
      target: moon.clone(),
      position: moon.clone().add(away.multiplyScalar(VIS_MOON_R * 4.6)).add(new THREE.Vector3(0, VIS_MOON_R * 2.0, 0)),
    };
  }
  if (mode === "craft") {
    const v = vec3(sample.v);
    if (v.lengthSq() < 1e-10) v.set(0, 0, 1);
    else v.normalize();
    return {
      target: craft.clone(),
      position: craft.clone().add(v.multiplyScalar(-5.4)).add(new THREE.Vector3(0, 2.2, 1.6)),
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
}: {
  sample: CislunarSample;
  mode: CameraMode;
  offset: MutableRefObject<THREE.Vector3>;
}) {
  const { camera, controls } = useThree();
  const lastMode = useRef<CameraMode | null>(null);
  const snap = useRef(1);
  const craftFollow = mode === "craft";

  useFrame((_, dt) => {
    const ctrl = controls as OrbitControlsImpl | null;
    if (!ctrl) return;
    const frame = modeFraming(mode, sample);
    if (lastMode.current !== mode) {
      lastMode.current = mode;
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
    camera.far = 2500;
    camera.updateProjectionMatrix();
  });
  return null;
}

function SceneBody({
  mission,
  sample,
  mode,
}: {
  mission: CislunarMission;
  sample: CislunarSample;
  mode: CameraMode;
}) {
  const moonPos = moonVisual(sample.moon);
  const craftPos = craftVisual(sample);
  const controlsRef = useRef<OrbitControlsImpl>(null);
  const offset = useRef(new THREE.Vector3());

  return (
    <>
      <color attach="background" args={["#07080c"]} />
      <Sun t={sample.t} />
      <Stars />
      <gridHelper args={[720, 18, "#1c2230", "#12161f"]} />
      <Earth t={sample.t} />
      <Moon pos={moonPos} />
      <MoonOrbit />
      <LunarOrbit moon={moonPos} />
      <Trajectory mission={mission} />
      <Probe sample={sample} />
      <CraftBeacon position={craftPos} />
      <CameraRig sample={sample} mode={mode} offset={offset} />
      <OrbitControls
        ref={controlsRef}
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={3.2}
        maxDistance={900}
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
}: {
  mission: CislunarMission;
  sample: CislunarSample;
  mode: CameraMode;
}) {
  return (
    <Canvas
      dpr={[1, 2]}
      gl={{ antialias: true, alpha: false, preserveDrawingBuffer: true, powerPreference: "high-performance" }}
      camera={{ position: [150, 120, 220], fov: 46, near: 0.1, far: 2500 }}
      style={{ width: "100%", height: "100%", background: "#07080c", touchAction: "none" }}
    >
      <SceneBody mission={mission} sample={sample} mode={mode} />
    </Canvas>
  );
}

export function CraftInsetCanvas({ sample }: { sample: CislunarSample }) {
  return (
    <Canvas
      dpr={[1, 2]}
      gl={{ antialias: true, alpha: false, preserveDrawingBuffer: true, powerPreference: "high-performance" }}
      camera={{ position: [2.8, 1.7, 3.4], fov: 42, near: 0.1, far: 40 }}
      style={{ width: "100%", height: "100%", background: "#07080c", touchAction: "none" }}
    >
      <color attach="background" args={["#07080c"]} />
      <ambientLight intensity={0.55} />
      <hemisphereLight args={["#cfd6e2", "#1a1d24", 0.7]} />
      <directionalLight position={[6, 8, 5]} intensity={1.8} />
      <directionalLight position={[-5, -3, -6]} intensity={0.45} />
      <gridHelper args={[10, 10, "#1c2230", "#12161f"]} />
      <CraftModel sample={sample} scale={1.35} />
      <OrbitControls enableDamping dampingFactor={0.08} minDistance={2.2} maxDistance={12} />
    </Canvas>
  );
}
