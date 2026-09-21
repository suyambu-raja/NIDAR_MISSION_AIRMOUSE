# NIDAR M2 — Coordinate Frames & Transform Conventions

## 1. Coordinate Frame Tree (TF)

In GPS-denied indoor environments, rigorous adherence to REP-103 and REP-105 standard coordinate conventions is critical for state estimation, SLAM, and visual alignment.

```
       map (Global Fixed Reference Frame: ENU)
        │
        ▼ (SLAM drift correction transform)
       odom (Continuous Odometry Frame: ENU)
        │
        ▼ (EKF continuous state estimation)
       base_link (Vehicle Center of Mass: Forward-Left-Up)
        │
        ├───────────────┬───────────────┬───────────────┐
        ▼               ▼               ▼               ▼
      lidar_link   camera_rgb_link  camera_th_link   rangefinder_link
  (X:Fwd, Y:Left) (Optical: Z:Fwd) (Optical: Z:Fwd) (Z: Downward)
```

---

## 2. Frame Conventions & Handedness

* **World Frame (`map`)**: Right-handed **ENU** (East-North-Up).
  * $+X$: East (Arena length axis)
  * $+Y$: North (Arena width axis)
  * $+Z$: Up
  * Origin $(0, 0, 0)$: Calibrated at the vehicle launch pad staging location.
* **Body Frame (`base_link`)**:
  * $+X$: Forward
  * $+Y$: Left
  * $+Z$: Up
* **Optical Frames (`camera_rgb_optical_frame`, `camera_thermal_optical_frame`)**: Standard OpenCV optical convention.
  * $+Z$: Optical axis pointing forward into the scene
  * $+X$: Right in image plane
  * $+Y$: Down in image plane
* **Transform to Map**:
  $$\begin{bmatrix} X_{\text{map}} \\ Y_{\text{map}} \\ Z_{\text{map}} \end{bmatrix} = T_{\text{map}\to\text{base\_link}} \cdot T_{\text{base\_link}\to\text{camera}} \cdot \begin{bmatrix} X_{\text{cam}} \\ Y_{\text{cam}} \\ Z_{\text{cam}} \end{bmatrix}$$

---

## 3. GCS Display Coordinate Alignment

* **Canvas Coordinates vs. Map Coordinates**:
  * In the Custom GCS canvas renderer, the mathematical map origin $(0, 0)$ is mapped to screen space using explicit pan offsets and zoom scaling:
    $$\text{screen\_x} = \text{center\_x} + \left(\frac{x - \text{origin\_x}}{\text{resolution}}\right) \cdot \text{scale} + \text{pan\_x}$$
    $$\text{screen\_y} = \text{center\_y} - \left(\frac{y - \text{origin\_y}}{\text{resolution}}\right) \cdot \text{scale} + \text{pan\_y}$$
  * Note the inverted Y-axis between ROS ENU ($+Y$ North/Up) and HTML5 Canvas ($+Y$ Downwards). This is handled transparently inside `map_renderer.js`.
* **Zero Fake Offsets**: The GCS applies no artificial offsets, shifts, or arbitrary scalings. The vehicle pose, SLAM grid, A* path, and survivor locations all share the exact same metric coordinate frame.
