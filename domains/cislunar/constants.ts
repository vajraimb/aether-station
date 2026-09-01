/** Earth–Moon patched-conic visualization. Distances in km, time in seconds. */

export const MU_SUN = 1.32712440018e11;
export const AU = 149597870.7;
export const A_EARTH = AU;
export const A_MARS = 1.52366231 * AU;
export const MU_EARTH = 398600.4418;
export const MU_MOON = 4902.800066;
export const MU_MARS = 42828.375214;
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

export const R_MARS = 3389.5;
export const LMO_ALT = 250;
export const R_LMO = R_MARS + LMO_ALT;

export const PHASES = [
  "leo",
  "tli",
  "coast",
  "loi",
  "llo",
  "revolution",
  "tmi",
  "heliocoast",
  "moi",
  "lmo",
  "marsrev",
] as const;
export type Phase = (typeof PHASES)[number];

export const PHASE_LABEL: Record<Phase, string> = {
  leo: "LEO parking",
  tli: "TLI burn",
  coast: "Translunar coast",
  loi: "LOI burn",
  llo: "Lunar orbit",
  revolution: "Moon around Earth",
  tmi: "TMI burn",
  heliocoast: "Earth–Mars coast",
  moi: "MOI burn",
  lmo: "Mars orbit",
  marsrev: "Mars around Sun",
};

export function isLunarPhase(phase: Phase): boolean {
  return phase === "loi" || phase === "llo" || phase === "revolution";
}

export function isHelioPhase(phase: Phase): boolean {
  return (
    phase === "tmi" ||
    phase === "heliocoast" ||
    phase === "moi" ||
    phase === "lmo" ||
    phase === "marsrev"
  );
}

export function isMarsPhase(phase: Phase): boolean {
  return phase === "moi" || phase === "lmo" || phase === "marsrev";
}
