import { createFileRoute } from "@tanstack/react-router";
import { CislunarApp } from "@/components/CislunarApp";

export const Route = createFileRoute("/")({ component: Home });

function Home() {
  return <CislunarApp />;
}
