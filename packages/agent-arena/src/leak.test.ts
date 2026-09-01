import { readdirSync, readFileSync, statSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function walk(dir: string, acc: string[] = []): string[] {
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    if (statSync(p).isDirectory()) walk(p, acc);
    else if (name.endsWith(".ts") && !name.endsWith(".test.ts")) acc.push(p);
  }
  return acc;
}

const BANNED_IMPORT = /from\s+["'][^"']*(dynamics|math3d|simulator|thrusters|slosh)[^"']*["']/;

export function runArenaLeakTests(): T[] {
  const out: T[] = [];
  const root = dirname(fileURLToPath(import.meta.url));
  for (const f of walk(root)) {
    const src = readFileSync(f, "utf8");
    const hit = src.match(BANNED_IMPORT);
    out.push({
      name: `arena_leak_${f.split("/").pop()}`,
      pass: !hit,
      detail: hit ? hit[0] : "clean",
    });
  }
  return out;
}
