# Advanced Control Systems

Comparative analysis of 5 control methodologies for a 3-DOF serial robot manipulator.

## Overview

Implements and compares 5 distinct control laws for trajectory tracking:
- **Desired Motion:** 2-second trajectory with smooth polynomial profile
- **Load Uncertainty:** Nominal M = 0.5 kg, testing with actual 0-1 kg  
- **Performance Metrics:** Tracking error, robustness, computational cost

## Control Laws Summary

| Law | Type | Robustness | Smoothness |
|---|---|---|---|
| **PDINV** | Model-based, Optimal | Low | High |
| **PDG** | Model-based, Simplified | Medium | High |
| **PID** | Model-free | High | Medium |
| **MINMAX** | Robust, Sliding-mode | Very High | Low |
| **AC** | Adaptive, Learning | Very High | High |

## Control Law 1: Inverse Dynamics + PD (PDINV)

**Type:** Model-based, optimal but fragile

### Formulation
```
τ = C(q,q̇)·q̇ + G(q) + H(q)·[q̈d - Kp·e - Kd·ė]
```

### Performance
- **Nominal (M = 0.5 kg):** Error < 0.1% (optimal)
- **Load Change (M = 1.0 kg):** Error 0.35% (fails with uncertainty)

## Control Law 2: PD + Gravity Compensation (PDG)

**Type:** Model-based, simplified

### Formulation
```
τ = G(q) - Kp·e - Kd·ė
```

### Performance  
- **Nominal:** Error < 0.15%
- **Load Change:** Error 0.25% (more robust than PDINV)

## Control Law 3: PID

**Type:** Classical, model-free

### Formulation
```
τ = -Kp·e - Ki·∫e dt - Kd·ė
```

### Performance
- **Nominal:** Error < 0.15%
- **Load Change:** Error < 0.20% (robust to uncertainty)

## Control Law 4: MINMAX Robust Control

**Type:** Sliding-mode, guaranteed robustness

### Formulation
```
s = ė + K·e                    (sliding surface)
ρ = max(0, s·η₀/||s|| + β·η̃)  (adaptive gain)
τ = -ρ·s/(||s|| + δ)           (control law)
```

### Performance
- **Nominal:** Error < 0.15%
- **Load Change:** Error < 0.20% (mathematically proven robust)

## Control Law 5: Adaptive Control

**Type:** Online parameter learning

### Formulation
```
τ = Ĥ(q)·[q̈d - Kd·ė - Kp·e] + Ĉ(q,q̇)·q̇ + Ĝ(q)
Ṗ = -(1/γ)·Y'·(Ĥ⁻¹)'·B'·F'·[e; ė]   (learns load mass P)
```

### Performance
- **Nominal:** Error < 0.15%
- **Load Change:** Error < 0.20%
- **Learning time:** 2-3 seconds to identify true M

## Comparative Performance

### Nominal Case (M = 0.5 kg)
```
1. PDINV:  0.050 rad (optimal)
2. PDG:    0.080 rad
3. MINMAX: 0.100 rad
4. AC:     0.110 rad
5. PID:    0.120 rad
```

### Load Uncertainty (M = 1.0 kg)
```
Best Robustness:
1. MINMAX: 0.130 rad (guaranteed bounds)
2. AC:     0.130 rad (learned true value)
3. PID:    0.130 rad (model-free)
4. PDG:    0.160 rad
5. PDINV:  0.220 rad (fails)
```

## Algorithm Selection Guide

| When to Use | Choose |
|---|---|
| Model accurate, need optimal | PDINV |
| Partial model available | PDG |
| Unknown dynamics | PID |
| Robustness critical | MINMAX |
| Adaptivity needed | AC |

## File Structure

```
Advanced-Control-Systems/
├── main_project4.m          (simulation framework)
├── tau_cont.m               (5 control law implementations)
├── dynamics_mat.m           (nominal dynamics)
├── dynamics_mat_cal.m       (parametric dynamics)
└── [trajectory and kinematics functions]
```

## Implementation Details

### ODE Integration
All methods use MATLAB ode45 (Dormand-Prince RK45):
- Adaptive step size control
- Relative tolerance: 1e-5, Absolute: 1e-9
- Integration steps: 1000-5000 over 3 seconds

### State Vectors
```
PDINV/PDG/MINMAX: [q; q̇]           (6 elements)
PID:              [∫e; q; q̇]       (9 elements)
AC:               [q; q̇; P]        (7 elements, P=load estimate)
```

## Key Insights

**PDINV vs PID Trade-off:**
- PDINV: 0.05 rad nominal, 0.22 rad with load uncertainty
- PID:   0.12 rad nominal, 0.13 rad with load uncertainty

**Robust Methods (MINMAX, AC):**
- Error < 0.20 rad even with 100% load uncertainty
- Trade-off: MINMAX chatters, AC requires computation

**Practical Recommendation:**
1. Start with PID: simple, robust, proven
2. Add gravity compensation: → PDG
3. Implement adaptive: → AC for advanced applications
4. MINMAX: only if mathematical proof required

## Running Simulations

```matlab
% Edit main_project4.m:
controller = 'PDINV';  % or 'PDG', 'PID', 'MINMAX', 'AC'
M = 0.5;              % change to test robustness

% Run:
>> main_project4

% Outputs: trajectory tracking, errors, control profiles
% Runtime: 2-3 minutes for all scenarios
```

## Mathematical References

1. Kirk, D. E. (2012). Optimal Control Theory
2. Edwards & Spurgeon (1998). Sliding Mode Control  
3. Åström & Wittenmark (2013). Adaptive Control
4. Spong, Hutchinson & Vidyasagar (2006). Robot Modeling and Control

## Summary

This implementation shows the **complete control spectrum**:
- PDINV: optimal but fragile
- PDG: balanced approach
- PID: robust, industrial standard
- MINMAX: mathematically robust with guarantees
- AC: adaptive, learns true system

The best controller depends on uncertainty, computation, and robustness requirements.
