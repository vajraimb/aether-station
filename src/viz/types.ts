import type { Sample } from "@/sim/types";

export interface ViewOpts {
  showTrail: boolean;
  showTarget: boolean;
  showAxes: boolean;
  isolated: number;
}

/**
 * The only fields the 3D scene actually reads.
 *
 * Declaring it as a projection of `Sample` keeps the live simulator working
 * unchanged (a `Sample` is assignable here) while letting a replayed frame -
 * which carries no estimator or diagnostic fields - drive the same scene.
 */
export type SceneSample = Pick<
  Sample,
  "t" | "r" | "q" | "w" | "s" | "th1" | "th2" | "thrusterActual" | "detectedFailedThruster"
>;
