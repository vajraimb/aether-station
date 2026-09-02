import { useEffect, useMemo } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Line } from "@react-three/drei";
import * as THREE from "three";
import { THRUSTERS } from "@/sim/constants";
import { pendulumPos } from "@/sim/dynamics";
import type { SceneSample, ViewOpts } from "./types";

export type { SceneSample, ViewOpts };

function Stars() {
  const geom = useMemo(() => {
    const n = 700;
    const arr = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      const r = 40 + Math.random() * 70;
      const u = Math.random();
      const v = Math.random();
      const theta = 2 * Math.PI * u;
      const phi = Math.acos(2 * v - 1);
      arr[i * 3] = r * Math.sin(phi) * Math.cos(theta);
      arr[i * 3 + 1] = r * Math.sin(phi) * Math.sin(theta);
      arr[i * 3 + 2] = r * Math.cos(phi);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(arr, 3));
    return g;
  }, []);

  useEffect(() => () => geom.dispose(), [geom]);

  return (
    <points geometry={geom}>
      <pointsMaterial color="#c8ceda" size={0.09} sizeAttenuation />
    </points>
  );
}

function AxisTriplet({
  scale = 2.6,
  origin = [0, 0, 0] as [number, number, number],
  opacity = 1,
}) {
  const s = scale;
  const o = origin;
  return (
    <group>
      <Line points={[o, [o[0] + s, o[1], o[2]]]} color="#d07070" lineWidth={2} transparent opacity={opacity} />
      <Line points={[o, [o[0], o[1] + s, o[2]]]} color="#70b080" lineWidth={2} transparent opacity={opacity} />
      <Line points={[o, [o[0], o[1], o[2] + s]]} color="#6a92c8" lineWidth={2} transparent opacity={opacity} />
    </group>
  );
}

function ThrusterMesh({
  i,
  actual,
  failed,
}: {
  i: number;
  actual: boolean;
  failed: boolean;
}) {
  const g = THRUSTERS[i]!;
  const dir = new THREE.Vector3(...g.dir);
  const quat = new THREE.Quaternion();
  quat.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  const color = failed ? "#d07272" : actual ? "#c88858" : "#6a7180";
  return (
    <group position={g.pos} quaternion={quat}>
      <mesh position={[0, -0.08, 0]}>
        <cylinderGeometry args={[0.07, 0.09, 0.16, 10]} />
        <meshStandardMaterial
          color={color}
          emissive={actual ? "#c88858" : failed ? "#d07272" : "#000000"}
          emissiveIntensity={actual ? 0.7 : failed ? 0.35 : 0}
          metalness={0.5}
          roughness={0.4}
        />
      </mesh>
      {actual && (
        <mesh position={[0, -0.38, 0]}>
          <coneGeometry args={[0.11, 0.5, 10]} />
          <meshBasicMaterial color="#e8b07a" transparent opacity={0.75} />
        </mesh>
      )}
    </group>
  );
}

function StationBody({ sample, opts, trail }: { sample: SceneSample; opts: ViewOpts; trail: [number, number, number][] }) {
  const q = sample.q;
  const quat = new THREE.Quaternion(q[1], q[2], q[3], q[0]);
  const r1 = pendulumPos(sample.th1, 1, 1.25);
  const r2 = pendulumPos(sample.th2, 2, 1.25);
  const w = sample.w;
  const wlen = Math.hypot(w[0], w[1], w[2]);

  return (
    <group position={sample.r}>
      {opts.showTarget && (
        <group>
          <mesh rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[1.08, 1.08, 4.4, 28]} />
            <meshStandardMaterial color="#d8dce6" transparent opacity={0.08} depthWrite={false} wireframe />
          </mesh>
        </group>
      )}
      <group quaternion={quat}>
        <mesh rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[1.08, 1.08, 4.4, 36]} />
          <meshStandardMaterial color="#9aa4b2" metalness={0.62} roughness={0.32} />
        </mesh>
        {[-2.2, 2.2].map((x) => (
          <mesh key={x} position={[x, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[1.1, 1.1, 0.08, 36]} />
            <meshStandardMaterial color="#7c8796" metalness={0.5} roughness={0.4} />
          </mesh>
        ))}
        <mesh rotation={[0, Math.PI / 2, 0]}>
          <torusGeometry args={[1.25, 0.11, 10, 48]} />
          <meshStandardMaterial color="#6d7c90" metalness={0.4} roughness={0.45} transparent opacity={0.85} />
        </mesh>
        <mesh rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.045, 0.045, 3.7, 10]} />
          <meshStandardMaterial color="#c5ccd8" metalness={0.7} roughness={0.25} />
        </mesh>
        <mesh position={[sample.s, 0, 0]}>
          <boxGeometry args={[0.38, 0.32, 0.32]} />
          <meshStandardMaterial color="#eef2f6" emissive="#c5ccd8" emissiveIntensity={0.25} metalness={0.2} roughness={0.3} />
        </mesh>
        <mesh position={r1}>
          <sphereGeometry args={[0.16, 16, 16]} />
          <meshStandardMaterial color="#7dba9a" emissive="#7dba9a" emissiveIntensity={0.35} transparent opacity={0.85} />
        </mesh>
        <mesh position={r2}>
          <sphereGeometry args={[0.14, 16, 16]} />
          <meshStandardMaterial color="#6a92c8" emissive="#6a92c8" emissiveIntensity={0.35} transparent opacity={0.85} />
        </mesh>
        {opts.showAxes && <AxisTriplet scale={2.4} />}
        {THRUSTERS.map((_, i) => (
          <ThrusterMesh
            key={i}
            i={i}
            actual={sample.thrusterActual[i] === 1}
            failed={opts.isolated === i || sample.detectedFailedThruster === i}
          />
        ))}
        {wlen > 1e-4 && (
          <Line points={[[0, 0, 0], [w[0] * 8, w[1] * 8, w[2] * 8]]} color="#c4a574" lineWidth={2} />
        )}
        <mesh>
          <sphereGeometry args={[0.06, 10, 10]} />
          <meshBasicMaterial color="#e8eaef" />
        </mesh>
      </group>
      {opts.showTrail && trail.length > 1 && (
        <Line points={trail} color="#8a9bb0" lineWidth={1} transparent opacity={0.45} />
      )}
    </group>
  );
}

export function StationCanvas({
  sample,
  opts,
  trail,
}: {
  sample: SceneSample;
  opts: ViewOpts;
  trail: [number, number, number][];
}) {
  return (
    <Canvas
      camera={{ position: [7.2, 3.6, 8.4], fov: 42, near: 0.1, far: 200 }}
      dpr={[1, 2]}
      gl={{ antialias: true, alpha: false, powerPreference: "high-performance" }}
      style={{ width: "100%", height: "100%", background: "#07080c", touchAction: "none" }}
    >
      <color attach="background" args={["#07080c"]} />
      <ambientLight intensity={0.28} />
      <hemisphereLight args={["#cfd6e2", "#1a1d24", 0.55]} />
      <directionalLight position={[8, 12, 6]} intensity={1.15} />
      <directionalLight position={[-6, -4, -8]} intensity={0.25} />
      <Stars />
      <gridHelper args={[24, 24, "#1c2230", "#12161f"]} />
      <AxisTriplet scale={3.4} opacity={0.55} />
      <StationBody sample={sample} opts={opts} trail={trail} />
      <OrbitControls enableDamping dampingFactor={0.08} minDistance={3.5} maxDistance={28} />
    </Canvas>
  );
}
