import { mkdirSync, writeFileSync } from "node:fs";
import { dirname } from "node:path";

export interface ArtifactRecorder {
  write(path: string, body: string): void;
}

export const fileRecorder: ArtifactRecorder = {
  write(path, body) {
    mkdirSync(dirname(path), { recursive: true });
    writeFileSync(path, body.endsWith("\n") ? body : `${body}\n`, "utf8");
  },
};
