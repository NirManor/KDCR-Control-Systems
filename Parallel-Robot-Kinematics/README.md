# Parallel Robot Kinematics

Implementation of constraint-based kinematics for a 3-DOF parallel robot, including inverse kinematics, forward kinematics via polynomial solving, trajectory planning, and singularity analysis.

## Problem Description

We analyze a **3-DOF parallel robot** with:
- **Central platform:** Equilateral triangle with side length r = 2 m
- **Leg 1 & 2:** Revolute-Prismatic chain on circular track (radius R = 4 m)
  - Link length: L = 3.5 m
  - Actuated: slider angles θ₁, θ₂ on track
- **Leg 3:** Prismatic joint with variable length d₃
- **End-effector:** Platform position [x, y] and orientation φ

## Robot Parameters

- r = 2 m (platform side)
- L = 3.5 m (link length)  
- R = 4 m (track radius)

## Constraint Equations

Each leg connects platform vertices to track points:

**Leg 1:** (a₁ₓ - b₁ₓ)² + (a₁ᵧ - b₁ᵧ)² = L²
**Leg 2:** (a₂ₓ - b₂ₓ)² + (a₂ᵧ - b₂ᵧ)² = L²
**Leg 3:** (a₃ₓ - b₃ₓ)² + (a₃ᵧ - b₃ᵧ)² = d₃²

**Platform geometry** (equilateral triangle):
- a₁ = [x + r·cos(φ + π/3), y + r·sin(φ + π/3)]ᵀ
- a₂ = [x, y]ᵀ
- a₃ = [x + r·cos(φ), y + r·sin(φ)]ᵀ

**Slider positions** (on circular track):
- b₁ = [R·cos(θ₁), R·sin(θ₁)]ᵀ
- b₂ = [R·cos(θ₂), R·sin(θ₂)]ᵀ
- b₃ = [0, -R]ᵀ (fixed)

## Main Algorithms

### 1. Inverse Kinematics

**Problem:** Given platform pose [x, y, φ], find all joint configurations [θ₁, θ₂, d₃]

**Algorithm:**
1. Form constraint equations F(x,q) = 0
2. Solve system symbolically/numerically
3. Eliminates variables systematically to find all solution branches
4. Filter physically valid solutions (joint limits, collisions)

**Implementation:** inverse_kin.m
- Uses MATLAB symbolic solving
- Returns 4-8 valid configurations per pose
- Round-trip error: < 10⁻⁴

### 2. Forward Kinematics

**Problem:** Given joint values [θ₁, θ₂, d₃], find all platform poses [x, y, φ]

**Key Innovation - Polynomial Solving:**
1. Eliminate x² and y² to get linear equations in x(φ), y(φ)
2. Substitute back into constraint to get polynomial in φ
3. Use Weierstrass substitution: t = tan(φ/2)
   - cos(φ) = (1 - t²)/(1 + t²)
   - sin(φ) = 2t/(1 + t²)
4. Solve resulting polynomial P(t) = 0 numerically
5. Convert roots back to φ values and verify

**Implementation:** forward_kin.m
- Handles up to 8th degree polynomials
- Returns up to 4 distinct solutions

### 3. Trajectory Planning

**Problem:** Move platform smoothly from point A to point B

**Approach:** Use parametric shape function s(t) ∈ [0,1]:
- x(t) = xₐ + (xᵦ - xₐ)·s(t)
- y(t) = yₐ + (yᵦ - yₐ)·s(t)
- φ(t) = φₐ + (φᵦ - φₐ)·s(t)

**Three velocity profiles offered:**
1. **Constant:** s(t) = t/T (simple but jerky acceleration)
2. **Trapezoidal:** Accel → Constant → Decel phases
3. **5th-order polynomial:** s(t) = 10(t/T)³ - 15(t/T)⁴ + 6(t/T)⁵
   - Zero boundary accelerations
   - Smooth start and stop

**Algorithm:**
1. Plan Cartesian trajectory using shape function
2. Sample at discrete time steps
3. Solve inverse kinematics at each step
4. Compute joint velocities using Jacobian
5. Validate for joint limits and singularities

**Implementation:** joint_plan.m, pos_plan.m
- Generates smooth trajectories with zero boundary conditions
- Validates feasibility throughout motion

### 4. Jacobian Analysis  

**Constraint-based approach:**
- From F(x,q) = 0, differentiate to get: J_x·ẋ + J_q·q̇ = 0
- Jacobian: J = -J_q⁻¹·J_x
- Maps joint velocities to Cartesian velocities

**Singularity Detection:**
- Singularities when det(J_x) = 0
- Robot loses one DOF
- Identified by sweeping workspace and computing det(J_x)
- Characterized via eigenvector analysis

## Function Summary

| Function | Purpose |
|---|---|
| inverse_kin.m | Solve IK for platform pose |
| forward_kin.m | Solve FK for joint values |
| joint_plan.m | Generate joint trajectory |
| pos_plan.m | Generate Cartesian trajectory |
| Jacobian_calculation.m | Symbolic Jacobian |
| Jacobian_x.m | Numerical Jacobian |
| HW2_Main.m | Complete demonstration |

## Results & Validation

### Inverse Kinematics
- Number of solutions: 4-8 per pose
- Accuracy: Verified by forward kinematics
- Round-trip error: < 10⁻⁴ rad/m

### Forward Kinematics
- Solution multiplicity: Up to 4 distinct poses
- All solutions satisfy original constraints
- Polynomial degree: Up to 8

### Trajectory Planning
- 3 velocity profile options
- Polynomial profile has zero boundary accelerations
- Validated for joint limits and singularities

### Jacobian Analysis
- Singularities: 4-6 within workspace
- Location: Workspace boundaries
- Characterized via eigenvector analysis

## Mathematical References

1. Tsai, L. W. (1999). Robot Analysis: The Mechanics of Serial and Parallel Manipulators
2. Craig, J. J. (2005). Introduction to Robotics: Mechanics and Control
3. Weierstrass substitution technique for polynomial solving

## Key Implementation Details

- All calculations done parametrically first, numerical substitution at end
- Multiple solution branches handled explicitly
- Singularities identified for workspace analysis
- Trajectories validated for feasibility

**Execution time:** HW2_Main.m ~5-10 minutes (includes visualizations)
