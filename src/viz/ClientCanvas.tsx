import { useEffect, useState, type ComponentType } from "react";
import type { SceneSample, ViewOpts } from "./types";

type Props = {
  sample: SceneSample;
  opts: ViewOpts;
  trail: [number, number, number][];
};

export function ClientCanvas(props: Props) {
  const [Canvas, setCanvas] = useState<ComponentType<Props> | null>(null);

  useEffect(() => {
    let live = true;
    void import("./StationScene").then((m) => {
      if (live) setCanvas(() => m.StationCanvas);
    });
    return () => {
      live = false;
    };
  }, []);

  if (!Canvas) {
    return <div className="h-full w-full bg-bg" aria-hidden />;
  }
  return <Canvas {...props} />;
}
