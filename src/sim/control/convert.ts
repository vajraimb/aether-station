import type { Command } from "../types";
import type { ControlCommand } from "./interface";

export function emptyPulseWidth(): [number, number, number, number, number, number] {
  return [0, 0, 0, 0, 0, 0];
}

export function commandFromControl(command: ControlCommand): Command {
  const pulseWidth = emptyPulseWidth();
  for (const thruster of command.thrusters) {
    if (thruster.id < 0 || thruster.id > 5) continue;
    pulseWidth[thruster.id] = thruster.pulseWidthS;
  }
  return { sliderForce: command.sliderForceN, pulseWidth };
}

export function controlFromCommand(command: Command): ControlCommand {
  const thrusters = [];
  for (let id = 0; id < 6; id += 1) {
    const pulseWidthS = command.pulseWidth[id] ?? 0;
    if (pulseWidthS > 0) thrusters.push({ id, pulseWidthS });
  }
  return { thrusters, sliderForceN: command.sliderForce };
}

export function pulseWidthFromPrimitive(thrusterIds: readonly number[], durationS: number): [number, number, number, number, number, number] {
  const pulseWidth = emptyPulseWidth();
  for (const id of thrusterIds) {
    if (id >= 0 && id < 6) pulseWidth[id] = durationS;
  }
  return pulseWidth;
}
