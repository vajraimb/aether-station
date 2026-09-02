/**
 * Standalone bundle of the Challenge V3 report.
 *
 * The app itself serves this page at /challenge-v3 through the router. This
 * entry exists only so the same component can be shipped as a self-contained
 * static bundle with relative asset URLs, which survives being hosted under an
 * arbitrary path prefix. No router, no SSR, no server data.
 */
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./styles.css";
import { ChallengeV3Report } from "../src/components/challenge-v3/ChallengeV3Report";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ChallengeV3Report />
  </StrictMode>,
);
