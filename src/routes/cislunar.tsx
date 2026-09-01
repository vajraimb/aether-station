import { createFileRoute } from "@tanstack/react-router";
import { CislunarApp } from "@/components/CislunarApp";

export const Route = createFileRoute("/cislunar")({ component: CislunarPage });

function CislunarPage() {
  return <CislunarApp />;
}
