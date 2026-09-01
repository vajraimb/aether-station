import { useMemo, useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, Line } from "@react-three/drei";
import * as THREE from "three";
import { KM_TO_SCENE, R_EARTH, R_MOON, type Phase } from "../../domains/cislunar/constants";
import type { CislunarMission, CislunarSample } from "../../domains/cislunar/trajectory";
import { makeEarthTexture, makeMoonTexture, makeStarPositions } from "./globe-textures";

import type { CameraMode } from "./cislunar-types";
const PHASE_COLOR: Record<Phase, string> = {
  leo: "#6a92c8",
  tli: "#c88858",
  coast: "#c4a574",
  loi: "#c88858",
  llo: "#7dba9a",
};

function toScene(p: readonly [number, number, number]): THREE.Vector3 {
  return new THREE.Vector3(p[0] * KM_TO_SCENE, p[1] * KM_TO_SCENE, p[2] * KM_TO_SCENE);
}

function Stars() {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(makeStarPositions(), 3));
    return g;
  }, []);
  return (
    <points geometry={geom}>
      <pointsMaterial color="#e8eaef" size={2.4} sizeAttenuation={false} />
    </points>
  );
}

function Earth() {
  const map = useMemo(() => makeEarthTexture(), []);
  const r = R_EARTH * KM_TO_SCENE;
  const ref = useRef<THREE.Mesh>(null);
  useFrame((_, d) => {
    if (ref.current) ref.current.rotation.y += Math.min(d, 0.1) * 0.02;
  });
  return (
    <group>
      <mesh ref={ref}>
        <sphereGeometry args={[r, 64, 48]} />
        <meshStandardMaterial map={map} roughness={0.72} metalness={0.02} emissive="#1b4f86" emissiveIntensity={0.35} />
      </mesh>
      <mesh scale={1.045}>
        <sphereGeometry args={[r, 48, 32]} />
        <meshBasicMaterial color="#7eb6e8" transparent opacity={0.2} side={THREE.BackSide} />
      </mesh>
    </group>
  );
}

function Moon({ pos }: { pos: THREE.Vector3 }) {
  const map = useMemo(() => makeMoonTexture(), []);
  const r = R_MOON * KM_TO_SCENE;
  const ref = useRef<THREE.Group>(null);
  useFrame(() => {
    if (!ref.current) return;
    ref.current.position.copy(pos);
    ref.current.lookAt(0, 0, 0);
  });
  return (
    <group ref={ref}>
      <mesh>
        <sphereGeometry args={[r, 48, 32]} />
        <meshStandardMaterial map={map} roughness={0.95} metalness={0} />
      </mesh>
    </group>
  );
}

function Probe({ sample }: { sample: CislunarSample }) {
  const ref = useRef<THREE.Group>(null);
  useFrame(() => {
    if (!ref.current) return;
    const p = toScene(sample.r);
    ref.current.position.copy(p);
    const v = toScene(sample.v);
    if (v.lengthSq() > 1e-8) {
      const target = p.clone().add(v.normalize());
      ref.current.lookAt(target);
    }
  });
  return (
    <group ref={ref}>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <capsuleGeometry args={[0.06, 0.16, 6, 10]} />
        <meshStandardMaterial color="#e8eaef" metalness={0.55} roughness={0.28} />
      </mesh>
      <mesh position={[0.14, 0, 0]}>
        <boxGeometry args={[0.18, 0.008, 0.09]} />
        <meshStandardMaterial color="#6a92c8" metalness={0.4} roughness={0.35} emissive="#6a92c8" emissiveIntensity={0.35} />
      </mesh>
      <mesh position={[-0.14, 0, 0]}>
        <boxGeometry args={[0.18, 0.008, 0.09]} />
        <meshStandardMaterial color="#6a92c8" metalness={0.4} roughness={0.35} emissive="#6a92c8" emissiveIntensity={0.35} />
      </mesh>
      <pointLight color="#c88858" intensity={0.8} distance={8} />
    </group>
  );
}

function Trajectory({ mission }: { mission: CislunarMission }) {
  const segments = useMemo(() => {
    const byPhase: { phase: Phase; pts: [number, number, number][] }[] = [];
    let cur: Phase | null = null;
    let pts: [number, number, number][] = [];
    for (const s of mission.samples) {
      if (s.phase !== cur) {
        if (cur && pts.length > 1) byPhase.push({ phase: cur, pts });
        cur = s.phase;
        pts = [];
      }
      pts.push([s.r[0] * KM_TO_SCENE, s.r[1] * KM_TO_SCENE, s.r[2] * KM_TO_SCENE]);
    }
    if (cur && pts.length > 1) byPhase.push({ phase: cur, pts });
    return byPhase;
  }, [mission]);
  return (
    <group>
      {segments.map((seg, i) => (
        <Line key={i} points={seg.pts} color={PHASE_COLOR[seg.phase]} lineWidth={1.4} transparent opacity={0.85} />
      ))}
    </group>
  );
}

function MoonOrbit() {
  const pts = useMemo(() => {
    const n = 96;
    const r = 384400 * KM_TO_SCENE;
    const out: [number, number, number][] = [];
    for (let i = 0; i <= n; i++) {
      const th = (i / n) * Math.PI * 2;
      out.push([r * Math.cos(th), 0, r * Math.sin(th)]);
    }
    return out;
  }, []);
  return <Line points={pts} color="#6d7686" lineWidth={0.6} transparent opacity={0.35} />;
}

function liftOutside(p: THREE.Vector3, center: THREE.Vector3, radius: number, pad: number): THREE.Vector3 {
  const d = p.clone().sub(center);
  const need = radius + pad;
  if (d.length() < 1e-6) return center.clone().add(new THREE.Vector3(0, need, 0));
  if (d.length() < need) d.setLength(need);
  return center.clone().add(d);
}

function MissionCamera({
  sample,
  mode,
}: {
  sample: CislunarSample;
  mode: CameraMode;
}) {
  const look = useRef(new THREE.Vector3());
  const primed = useRef(false);
  useFrame(({ camera }, dt) => {
    if (mode === "free") return;
    const d = Math.min(dt, 0.1);
    const craft = toScene(sample.r);
    const moon = toScene(sample.moon);
    const earth = new THREE.Vector3(0, 0, 0);
    const rE = R_EARTH * KM_TO_SCENE;
    const rM = R_MOON * KM_TO_SCENE;
    let desired: THREE.Vector3;
    let target: THREE.Vector3;
    if (mode === "earth") {
      desired = new THREE.Vector3(rE * 2.6, rE * 1.1, rE * 2.2);
      target = earth.clone();
    } else if (mode === "moon") {
      const away = moon.clone().normalize();
      desired = moon.clone().add(away.multiplyScalar(rM * 4.5)).add(new THREE.Vector3(0, rM * 1.6, 0));
      target = moon.clone();
    } else if (mode === "overview") {
      const mid = moon.clone().multiplyScalar(0.5);
      desired = mid.clone().add(new THREE.Vector3(30, 560, 700));
      target = mid.clone();
    } else if (mode === "follow") {
      const v = toScene(sample.v);
      if (v.lengthSq() < 1e-10) v.set(0, 0, 1);
      else v.normalize();
      const near = sample.phase === "llo" || sample.phase === "leo" || sample.phase === "tli";
      const dist = near ? 1.6 : 18;
      desired = craft.clone().add(v.multiplyScalar(-dist)).add(new THREE.Vector3(0, dist * 0.35, 0));
      target = craft.clone();
    } else if (sample.phase === "leo" || sample.phase === "tli") {
      const sun = new THREE.Vector3(1.6, 0.7, 0.4).normalize();
      desired = craft.clone().add(sun.multiplyScalar(2.4)).add(new THREE.Vector3(0, 0.9, 0));
      target = earth.clone().lerp(craft, 0.08);
    } else if (sample.phase === "coast") {
      const u = Math.min(1, Math.max(0, sample.altEarth / 3.7e5));
      if (u < 0.18) {
        const radial = craft.clone().normalize();
        desired = craft.clone().add(radial.multiplyScalar(4)).add(new THREE.Vector3(0, 2.5, 6));
        target = earth.clone();
      } else if (u < 0.72) {
        const mid = moon.clone().multiplyScalar(0.5);
        desired = mid.clone().add(new THREE.Vector3(40, 520, 640));
        target = mid.clone();
      } else {
        const away = craft.clone().sub(moon);
        if (away.lengthSq() < 1e-8) away.set(0, 0, 1);
        else away.normalize();
        desired = craft.clone().add(away.multiplyScalar(8)).add(new THREE.Vector3(0, 4, 0));
        target = moon.clone().lerp(craft, 0.25);
      }
    } else {
      const away = craft.clone().sub(moon);
      if (away.lengthSq() < 1e-8) away.set(0, 0, 1);
      else away.normalize();
      desired = craft.clone().add(away.multiplyScalar(2.4)).add(new THREE.Vector3(0, 1.1, 0));
      target = moon.clone().lerp(craft, 0.4);
    }
    const body = sample.phase === "llo" || sample.phase === "loi" ? moon : earth;
    const rad = sample.phase === "llo" || sample.phase === "loi" ? rM : rE;
    desired = liftOutside(desired, body, rad, 0.8);
    desired = liftOutside(desired, sample.phase === "llo" ? earth : moon, sample.phase === "llo" ? rE : rM, 0.5);
    const k = primed.current ? 1 - Math.exp(-2.4 * d) : 1;
    primed.current = true;
    camera.position.lerp(desired, k);
    look.current.lerp(target, k);
    camera.lookAt(look.current);
    camera.near = 0.05;
    camera.far = 8000;
    camera.updateProjectionMatrix();
    const w = window as unknown as { __cislunar?: Record<string, unknown> };
    w.__cislunar = {
      pos: camera.position.toArray(),
      look: look.current.toArray(),
      craft: craft.toArray(),
      moon: moon.toArray(),
      mode,
      phase: sample.phase,
    };
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
  const moonPos = toScene(sample.moon);
  return (
    <>
      <color attach="background" args={["#07080c"]} />
      <ambientLight intensity={0.55} />
      <hemisphereLight args={["#c5d6ea", "#2a241c", 0.7]} />
      <directionalLight position={[420, 180, 90]} intensity={3.2} color="#fff7ea" />
      <Stars />
      <Earth />
      <Moon pos={moonPos} />
      <MoonOrbit />
      <Trajectory mission={mission} />
      <Probe sample={sample} />
      <MissionCamera key={mode} sample={sample} mode={mode} />
      {mode === "free" ? <OrbitControls enableDamping makeDefault /> : null}
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
      gl={{ antialias: true, alpha: false, preserveDrawingBuffer: true }}
      camera={{ position: [0, 8, 40], fov: 50, near: 0.1, far: 8000 }}
    >
      <SceneBody mission={mission} sample={sample} mode={mode} />
    </Canvas>
  );
}
