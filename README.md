UPDATES FROM 22/10/25 LAB DISCUSSION/ASSESSMENT
- All buildings in the scene have associated planar shadows.
- All shadows only render on the scene floor, and not outside of it.
- The spider in the scene is reflected onto the floor, and has an associated planar shadow.
- Bump mapping on the floor is made more obvious. Please uncomment line 87 in mesh.frag if you would like to verify that the normal map is correctly utilized.

This project depicts a drone flying through a city.

Controls:
Drone Movement
Arrow Up ⬆️ → Pitch nose up and move forward.
Arrow Down ⬇️ → Pitch nose down and move backward (slower drift than forward).
Arrow Left ⬅️ → Roll drone left
Arrow Right ➡️ → Roll drone right

Vertical Control
W → Increase throttle (go up).
S → Decrease throttle (go down).

Yaw Rotation
A → Rotate left (turn yaw).
D → Rotate right (turn yaw).

Camera Views
1 → Top-down perspective camera.
2 → Top-down orthographic camera.
3 → Chase camera that follows the drone.

Mouse (drag left button) → Orbit the active camera (chase cam allows yaw/pitch offsets).

Environment & Lighting
C → Toggle point lights (colored city lights).
H → Toggle spotlights (drone headlights).
N → Toggle directional light (global light).
L → Toggle on and off the Lens Flare Effect.
F → Toggle fog effect.
