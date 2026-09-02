import { createFileRoute } from "@tanstack/react-router";
import { ChallengeV3Report } from "@/components/challenge-v3/ChallengeV3Report";

export const Route = createFileRoute("/challenge-v3")({ component: ReportPage });

function ReportPage() {
  return <ChallengeV3Report />;
}
