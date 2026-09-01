# Cislunar flight visualization

Patched-conic Earth → Moon transfer, 100 km lunar orbit, then one sidereal
month with the probe remaining in LLO while the Moon completes a full
revolution around Earth. HUD speeds and altitudes are physical. The 3D layout
uses an orrery scale. This is **not** the frozen AETHER attitude plant.

The app root `/` is this view. Station lab is at `/station`.

- Drag to orbit, scroll / pinch to zoom.
- System / Earth / Moon / Craft snap the camera target; zoom stays user-controlled.
- After capture the camera moves to Moon (watch the probe complete lunar revs),
  then to System for the Moon's trip around Earth.
- Loop replays the full circuit.
