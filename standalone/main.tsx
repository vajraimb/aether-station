/**
 * Standalone bundle of the Challenge V3 result pages.
 *
 * The app serves the same two components at /challenge-v3 and
 * /challenge-v3/replay through the router. This entry exists only so they can
 * ship as a self-contained static bundle with relative asset URLs, which
 * survives being hosted under an arbitrary path prefix. No router, no SSR, no
 * server data: the view is picked from the hash, and the components receive
 * hash link targets instead of paths.
 */
import { StrictMode, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import "./styles.css";
import {
  ChallengeV3Report,
  type PageLinks,
} from "../src/components/challenge-v3/ChallengeV3Report";
import { ChallengeV3Replay } from "../src/components/challenge-v3/ChallengeV3Replay";

const LINKS: PageLinks = { home: "#report", report: "#report", replay: "#replay" };

function App() {
  const [hash, setHash] = useState(() => window.location.hash);
  useEffect(() => {
    const on = () => setHash(window.location.hash);
    window.addEventListener("hashchange", on);
    return () => window.removeEventListener("hashchange", on);
  }, []);
  return hash === "#replay" ? (
    <ChallengeV3Replay links={LINKS} />
  ) : (
    <ChallengeV3Report links={LINKS} />
  );
}

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
