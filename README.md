UPDATES FROM 22/10/25 LAB DISCUSSION/ASSESSMENT
- All buildings in the scene have associated planar shadows.
- All shadows only render on the scene floor, and not outside of it.
- The spider in the scene is reflected onto the floor, and has an associated planar shadow.
- Bump mapping on the floor is made more obvious. Please uncomment line 87 in mesh.frag if you would like to verify that the normal map is correctly utilized.

Text Billboard Implementation: The label above the beam is a cylindrical billboard. We calculate the rotation matrix that makes the text face the camera around the y-axis in makeBillboardAt(). Then we render the text at that position using alpha blending in renderBillboardLabelAt().

This project depicts a drone flying through a city in a game. The goal is to pick up a package on the top the indicated building, and drop it off at the next indicated building. The drone's battery level depletes over time, during movement, and (significantly) upon collision with a building or a flying object. The player accumulates a score throughout the game, corresponding to the battery level when the package is dropped off. The drone battery is fully recharged after dropping off a package successfully.


*Controls*

Game Play:
- P → Pause/Resume
- R → Reset (active after game is lost)

Drone Movement:
- Arrow Up ⬆️ → Pitch nose up and move forward.
- Arrow Down ⬇️ → Pitch nose down and move backward (slower drift than forward).
- Arrow Left ⬅️ → Roll drone left
- Arrow Right ➡️ → Roll drone right

Vertical Control:
- W → Increase throttle (go up).
- S → Decrease throttle (go down).

Yaw Rotation:
- A → Rotate left (turn yaw).
- D → Rotate right (turn yaw).

Camera Views:
- 1 → Top-down perspective camera.
- 2 → Top-down orthographic camera.
- 3 → Chase camera that follows the drone.

Mouse (drag left button) → Orbit the active camera (chase cam allows yaw/pitch offsets).

Environment & Lighting:
- C → Toggle point lights (colored city lights).
- H → Toggle spotlights (drone headlights).
- N → Toggle directional light (global light).
- L → Toggle on and off the Lens Flare Effect.
- F → Toggle fog effect.
