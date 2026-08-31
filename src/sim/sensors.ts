import { clamp, makeRng, qmul, qnormalize, type Quat, type Rng, type Vec3 } from "./math3d";
import { pressureFromSlosh } from "./dynamics";
import type { Observation, SimState } from "./types";

const PRESS_DELAY = 0.08;
const Q_ANG_STD = (0.15 * Math.PI) / 180; // 0.15 deg
const GYRO_ARW = 3e-4; // rad/s /√Hz  white
const GYRO_RRW = 8e-6; // rad/s^1.5
const PACKET_LOSS = 0.01;
const JITTER = 0.003;

export class SensorSystem {
  private rng: Rng;
  private bias: Vec3;
  private pressBuf: { t: number; p1: number; p2: number }[] = [];
  private dtSens: number;

  constructor(seed: number, bias0: Vec3, dtSens = 0.05) {
    this.rng = makeRng(seed ^ 0x51ed);
    this.bias = [bias0[0], bias0[1], bias0[2]];
    this.dtSens = dtSens;
  }

  sample(
    st: SimState,
    fuelTrue: number,
    current: [number, number, number, number, number, number],
    abnormal: boolean,
  ): Observation | null {
    // Packet loss
    if (this.rng.u01() < PACKET_LOSS) return null;

    // Gyro bias RW + white
    const dt = this.dtSens;
    this.bias = [
      this.bias[0] + GYRO_RRW * Math.sqrt(dt) * this.rng.gauss(),
      this.bias[1] + GYRO_RRW * Math.sqrt(dt) * this.rng.gauss(),
      this.bias[2] + GYRO_RRW * Math.sqrt(dt) * this.rng.gauss(),
    ];
    const gyro: Vec3 = [
      st.w[0] + this.bias[0] + GYRO_ARW * this.rng.gauss(),
      st.w[1] + this.bias[1] + GYRO_ARW * this.rng.gauss(),
      st.w[2] + this.bias[2] + GYRO_ARW * this.rng.gauss(),
    ];

    // Quaternion noise as small rotation
    const ax = Q_ANG_STD * this.rng.gauss();
    const ay = Q_ANG_STD * this.rng.gauss();
    const az = Q_ANG_STD * this.rng.gauss();
    const dq: Quat = qnormalize([1, 0.5 * ax, 0.5 * ay, 0.5 * az]);
    const qMeas = qmul(st.q, dq);

    const [p1t, p2t] = pressureFromSlosh(st.th1, st.th2);
    this.pressBuf.push({ t: st.t, p1: p1t, p2: p2t });
    const want = st.t - PRESS_DELAY;
    while (this.pressBuf.length > 1 && this.pressBuf[1]!.t <= want) this.pressBuf.shift();
    let p1: number;
    let p2: number;
    if (this.pressBuf.length === 0 || this.pressBuf[0]!.t > want) {
      // Still filling the delay line — hold last (or zeros at t=0).
      p1 = this.pressBuf[0]?.p1 ?? p1t;
      p2 = this.pressBuf[0]?.p2 ?? p2t;
    } else {
      p1 = this.pressBuf[0]!.p1;
      p2 = this.pressBuf[0]!.p2;
    }
    p1 += 12 * this.rng.gauss();
    p2 += 12 * this.rng.gauss();

    const jitter = (this.rng.u01() * 2 - 1) * JITTER;
    const fuelEst = fuelTrue * (1 + 0.04 * (2 * this.rng.u01() - 1));
    const curr: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < 6; i++) {
      curr[i] = clamp(current[i] + 0.03 * this.rng.gauss(), 0, 1.4);
    }

    return {
      timestamp: st.t + jitter,
      quaternionMeasured: qMeas,
      gyroMeasured: gyro,
      sliderPosition: st.s + 0.002 * this.rng.gauss(),
      sliderVelocity: st.sd + 0.01 * this.rng.gauss(),
      tankWallPressure1: p1,
      tankWallPressure2: p2,
      remainingFuelEstimate: fuelEst,
      thrusterCurrentFeedback: curr,
      actuatorResponseAbnormal: abnormal,
    };
  }
}
