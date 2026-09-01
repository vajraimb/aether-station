import { useEffect, useState, type ComponentType } from "react";
import type { CameraMode } from "./cislunar-types";
import type { CislunarMission, CislunarSample } from "../../domains/cislunar/trajectory";

type Props = {
  mission: CislunarMission;
  sample: CislunarSample;
  mode: CameraMode;
};

export function CislunarCanvas(props: Props) {
  const [Canvas, setCanvas] = useState<ComponentType<Props> | null>(null);
  useEffect(() => {
    let live = true;
    void import("./CislunarScene").then((m) => {
      if (live) setCanvas(() => m.CislunarCanvas);
    });
    return () => {
      live = false;
    };
  }, []);
  if (!Canvas) return <div className="h-full w-full bg-bg" aria-hidden />;
  return <Canvas {...props} />;
}
