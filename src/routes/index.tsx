import { createFileRoute } from "@tanstack/react-router";
import { MissionApp } from "@/components/MissionApp";

export const Route = createFileRoute("/")({ component: Home });

function Home() {
  return <MissionApp />;
}
