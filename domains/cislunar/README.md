# Earth → Moon → Mars visualization

Patched-conic Earth → Moon transfer, 100 km lunar orbit, one sidereal month,
then a heliocentric Hohmann transfer to Mars and one Mars year in 250 km
orbit. HUD speeds and altitudes are physical. The 3D layout uses an orrery
scale (Earth–Moon, then Sun–Earth–Mars). This is **not** the frozen AETHER
attitude plant.

The app root `/` is this view. Station lab is at `/station`.

- Drag to orbit, scroll / pinch to zoom. Swipe the HUD handle down to hide it.
- System / Earth / Moon / Mars / Craft snap the camera.
- After lunar capture: Moon view, then System for the Moon's trip around Earth.
- After TMI the scene switches to a heliocentric orrery. Toggle **Spacetime**
  for an exaggerated solar gravity well: the Hohmann path is draped as a
  geodesic on that surface. Mars view on capture, then System for Mars around
  the Sun.
- Loop replays the full circuit (~978 days of mission time).
