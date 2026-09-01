import { createFileRoute, Navigate } from "@tanstack/react-router";

export const Route = createFileRoute("/cislunar")({ component: CislunarRedirect });

function CislunarRedirect() {
  return <Navigate to="/" replace />;
}
