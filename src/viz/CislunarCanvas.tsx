import { useEffect, useState, type ComponentType } from "react";
import type { CameraMode } from "./cislunar-types";
import type { CislunarMission, CislunarSample } from "../../domains/cislunar/trajectory";

type MainProps = {
  mission: CislunarMission;
  sample: CislunarSample;
  mode: CameraMode;
};

type InsetProps = { sample: CislunarSample };

export function CislunarCanvas(props: MainProps) {
  const [Canvas, setCanvas] = useState<ComponentType<MainProps> | null>(null);
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

export function CraftInset(props: InsetProps) {
  const [Canvas, setCanvas] = useState<ComponentType<InsetProps> | null>(null);
  useEffect(() => {
    let live = true;
    void import("./CislunarScene").then((m) => {
      if (live) setCanvas(() => m.CraftInsetCanvas);
    });
    return () => {
      live = false;
    };
  }, []);
  if (!Canvas) return <div className="h-full w-full bg-bg" aria-hidden />;
  return <Canvas {...props} />;
}
