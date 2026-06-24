# Stewart Platform Simulation V3 — Eigen-Based Linear Algebra

Extends V2 by replacing all custom math with Eigen — the industry standard
linear algebra library used by ROS2, MoveIt, Drake, and Pinocchio.
Adds Forward Kinematics (Newton-Raphson) and Task-Space Control.

---

## What's New in V3

| Feature | V2 | V3 |
|---|---|---|
| Math library | Custom std::array functions | Eigen |
| Vector type | std::array<double,3> | Eigen::Vector3d |
| Matrix type | std::array<std::array<double,3>,3> | Eigen::Matrix3d |
| Joint coordinates | std::array<std::array<double,3>,6> | Eigen::Matrix<double,3,6> |
| IK computation | Manual loop | Vectorized column-wise operations |
| Rotation matrix | Manual trig computation | Eigen::AngleAxisd |
| Forward kinematics | Not available | Newton-Raphson solver |
| Control mode | Joint-space only | Joint-space + Task-space |

---

## Key Eigen Types

```cpp
using Vec3   = Eigen::Vector3d;
using Mat3   = Eigen::Matrix3d;
using Vec6   = Eigen::Matrix<double, 6, 1>;
using Mat6   = Eigen::Matrix<double, 6, 6>;
using Mat3x6 = Eigen::Matrix<double, 3, 6>;
```

---

## Vectorized Inverse Kinematics

```cpp
Mat3x6 platform_world = R * geom.platform_joints;
platform_world.colwise() += p;
Mat3x6 leg_vectors  = platform_world - geom.base_joints;
Vec6   leg_lengths  = leg_vectors.colwise().norm();
Mat3x6 unit_vectors = leg_vectors.colwise().normalized();
```

No explicit loop — all 6 legs computed simultaneously using SIMD.

---

## Forward Kinematics — Newton-Raphson

```
iter=0  residual=0.400925
iter=1  residual=0.049195
iter=2  residual=0.000401
iter=3  residual=3.72e-08  ← converged
```

Quadratic convergence — 4 iterations from cold start.

---

## Task-Space Control

Controls platform pose directly in Cartesian space:

```
desired pose → pose error → wrench W → (J^T)^{-1} → leg forces → FOC
```

Velocity estimated from actuator velocities via Jacobian inverse:
```cpp
Vec6 actual_velocity = J.colPivHouseholderQr().solve(L_dot);
```

Settling time ~2 seconds for heave=5cm, roll=0.1rad simultaneously.

---

## Building

### Requirements
- C++17 compiler (MSVC, GCC, Clang)
- CMake 3.16+
- Eigen 3.4+ (header-only)

### Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
./StewartSim_V3
```

### Visual Studio
Open `Stewart_Platform_V3.sln` directly.

---

## Simulation Results

- Pure heave (z=+5cm): all 6 legs converge identically ✅
- Heave + roll (z=+5cm, roll=0.1rad): converge in ~1.8s ✅
- FK: converges in 4 iterations from cold start ✅
- Task-space: both z and roll settle in ~2s simultaneously ✅
- ESTOP: all iq_refs zeroed immediately ✅
- Clean shutdown: all threads join in reverse order ✅

---

## Project Structure

```
Stewart_Platform_V3/
├── main.cpp
├── CMakeLists.txt
├── math/Math.h
├── geometry/
│   ├── Geometry.h/.cpp
│   └── Kinematics.h/.cpp
├── actuator/
│   ├── MotorDriver.h/.cpp
│   └── ActuatorState.h/.cpp
├── control/
│   ├── PDController.h
│   └── TaskSpaceController.h/.cpp
├── platform/
│   ├── StewartPlatform.h/.cpp
│   └── SafetyMonitor.h/.cpp
├── threading/
│   ├── SharedData.h
│   ├── ThreadedFOCDriver.h/.cpp
│   ├── ThreadedController.h/.cpp
│   ├── ThreadedSafetyMonitor.h/.cpp
│   └── Logger.h/.cpp
└── utils/
    ├── LoopTimer.h
    └── RingBuffer.h
```

---

## Roadmap

- [x] Eigen-based math
- [x] Vectorized inverse kinematics
- [x] Forward kinematics (Newton-Raphson)
- [x] Task-space control via Jacobian
- [ ] ROS2 node wrapper (rclcpp)
- [ ] Real hardware interface (CAN bus, ODrive)

---

## Author

Masoud Roudneshin — PhD Electrical & Computer Engineering
Specialization: Robotics, Optimization and Control