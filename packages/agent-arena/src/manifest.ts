import type { ClaimType } from "./types";

export interface ArenaClaim {
  readonly type: ClaimType;
  readonly statement: string;
  readonly evidence_artifact: string;
}

export interface EpisodeManifest {
  readonly run_id: string;
  readonly domain: string;
  readonly environment_version: string;
  readonly agent_version: string;
  readonly code_sha: string;
  readonly scenario_hash: string | null;
  readonly parent_run_id: string | null;
  readonly artifacts: readonly string[];
  readonly claims: readonly ArenaClaim[];
}
