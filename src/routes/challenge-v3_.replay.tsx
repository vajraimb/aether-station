import { createFileRoute } from "@tanstack/react-router";
import { ChallengeV3Replay } from "@/components/challenge-v3/ChallengeV3Replay";

export const Route = createFileRoute("/challenge-v3_/replay")({
  component: ChallengeV3Replay,
});
