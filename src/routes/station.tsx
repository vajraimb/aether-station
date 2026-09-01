import { createFileRoute } from "@tanstack/react-router";
import { MissionApp } from "@/components/MissionApp";

export const Route = createFileRoute("/station")({ component: StationPage });

function StationPage() {
  return <MissionApp />;
}
