
# Inverse Kinematics (FABRIK)

A lightweight Unity implementation of **FABRIK** (Forward And Backward Reaching Inverse Kinematics) with support for constrained joints, useful for procedural limb/tentacle animation, robotic arm rigs, or any chain of bones that needs to reach toward a target in real time.

<p align="center">
  <img width="380" alt="FABRIK demo 1" src="https://github.com/user-attachments/assets/cc811046-dacd-408a-a4f4-9373274f37ab" />
  <img width="380" alt="FABRIK demo 2" src="https://github.com/user-attachments/assets/5b0e3c86-8717-44fc-99e3-11356fe2a2d6" />
</p>

## What is FABRIK?

FABRIK solves inverse kinematics iteratively, by repeatedly sweeping forward (from the end effector to the root) and backward (root to end effector) along the joint chain, repositioning each joint to satisfy bone-length constraints until the chain converges on the target - without the matrix/Jacobian math traditional IK solvers rely on. It's fast, stable, and easy to reason about, which makes it a common choice for real-time character and creature rigs.

## Features

- **FABRIK solver** for arbitrary-length joint chains
- **Constrained joint types**:
  - Ball-Socket joint
  - Hinge joint
- **Extensible** - add custom joint constraints by extending the base `IKJoint` class
- Runs in real time, suitable for procedural animation in Unity

## Getting Started

### Prerequisites
- [Unity](https://unity.com/) (check `ProjectSettings` in the repo for the exact version used)

### Setup

1. Clone the repository
   ```bash
   git clone https://github.com/MichalisTamiolakis/InverseKinematics.git
   ```
2. Open the project folder in Unity.
3. Open one of the demo scenes to see the FABRIK chain solving toward a moving target.
4. To use it in your own project, add an IK chain to your rig and attach `IKJoint`-derived components (Ball-Socket or Hinge) to each bone in the chain.

### Adding a custom joint type

Extend the base `IKJoint` class and implement its constraint logic to support joint types beyond Ball-Socket and Hinge - this is the intended extension point for the library.

## License

Distributed under the MIT License. See [`LICENSE`](./LICENSE) for details.
