/** Earth–Moon patched-conic visualization. Distances in km, time in seconds. */

export const MU_EARTH = 398600.4418;
export const MU_MOON = 4902.800066;
export const R_EARTH = 6371.0;
export const R_MOON = 1737.4;
export const A_MOON = 384400;
export const SOI_MOON = 66100;

export const LEO_ALT = 300;
export const LLO_ALT = 100;

export const R_LEO = R_EARTH + LEO_ALT;
export const R_LLO = R_MOON + LLO_ALT;

/** Scene units per kilometre. Earth radius ≈ 12.7, Earth–Moon ≈ 769. */
export const KM_TO_SCENE = 1 / 500;

export const PHASES = ["leo", "tli", "coast", "loi", "llo"] as const;
export type Phase = (typeof PHASES)[number];

export const PHASE_LABEL: Record<Phase, string> = {
  leo: "LEO parking",
  tli: "TLI burn",
  coast: "Translunar coast",
  loi: "LOI burn",
  llo: "Lunar orbit",
};
