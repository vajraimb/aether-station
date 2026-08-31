import { defaultPublicConfig } from "../constants";
import {
  analyzeMask,
  fibonacciSphere,
  jacobiEigen3,
  rhoOfTorque,
  runWrenchStudy,
  svdFromColumns,
} from "./wrench-geometry";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runWrenchGeometryTests(): T[] {
  const out: T[] = [];
  const eig = jacobiEigen3([
    [4, 0, 0],
    [0, 1, 0],
    [0, 0, 9],
  ]);
  check(
    "jacobi_diag_sorted",
    Math.abs(eig.values[0] - 9) < 1e-8 && Math.abs(eig.values[1] - 4) < 1e-8 && Math.abs(eig.values[2] - 1) < 1e-8,
    eig.values.join(","),
    out,
  );

  const svd = svdFromColumns([
    [1, 0, 0],
    [0, 2, 0],
    [0, 0, 3],
  ]);
  check("svd_rank3", svd.rank === 3, `rank=${svd.rank} s=${svd.values.join(",")}`, out);
  check("svd_cond", Math.abs(svd.cond - 3 / 1) < 1e-6, `cond=${svd.cond}`, out);

  const r = rhoOfTorque([1, 0, 0], [1, 0, 0]);
  check("rho_aligned", r.rho > 1e6 && Math.abs(r.par - 1) < 1e-12, `rho=${r.rho}`, out);
  const r2 = rhoOfTorque([1, 1, 0], [1, 0, 0]);
  check("rho_45", Math.abs(r2.rho - 1) < 1e-9, `rho=${r2.rho}`, out);

  const axes = fibonacciSphere(40);
  check("sphere_count", axes.length === 40, `n=${axes.length}`, out);
  const norms = axes.map((a) => Math.hypot(a[0], a[1], a[2]));
  check("sphere_unit", norms.every((n) => Math.abs(n - 1) < 1e-9), `min=${Math.min(...norms)}`, out);

  const plant = defaultPublicConfig();
  const healthy = analyzeMask(plant, [], axes, 0.873, [0, 0, 0]);
  check("healthy_singles_rank3", healthy.singles.rank === 3, `rank=${healthy.singles.rank} cond=${healthy.singles.cond}`, out);
  check("healthy_all_rank3", healthy.all.rank === 3, `rank=${healthy.all.rank}`, out);

  for (let i = 0; i < 6; i += 1) {
    const m = analyzeMask(plant, [i], axes, 0.873, [0, 0, 0]);
    check(`isolated_${i}_rank3`, m.singles.rank === 3, `rank=${m.singles.rank} cond=${m.singles.cond.toFixed(2)}`, out);
  }

  const study = runWrenchStudy(plant, { axisCount: 24 });
  check("study_7_masks", study.masks.length === 7, `n=${study.masks.length}`, out);
  check(
    "rho_finite",
    study.masks.every((m) => Number.isFinite(m.rhoAll.median)),
    "median rho",
    out,
  );
  return out;
}
