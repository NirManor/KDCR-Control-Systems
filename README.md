# KDCR: Kinematics, Dynamics, and Control of Robots

A comprehensive robotics systems implementation demonstrating the complete pipeline from geometric motion analysis through advanced nonlinear control theory. This repository contains three major implementations from Technion's graduate-level robotics course.

**Course:** Kinematics, Dynamics, and Control of Robots (036026)  
**Institution:** Technion - Israel Institute of Technology  
**Faculty:** Mechanical Engineering  
**Instructor:** Assoc. Prof. Yizhar Or  
**Semester:** Spring 2023  
**Authors:** Nir Manor & Sheng Liu  

---

## 📚 Repository Contents

### 1. **Parallel-Robot-Kinematics** 🤖
Constraint-based geometric analysis of parallel mechanisms

**What's Inside:**
- Inverse kinematics solver handling 4-8 solutions per pose
- Forward kinematics via polynomial solving (Weierstrass substitution)
- Trajectory planning with three velocity profile options
- Jacobian analysis and singularity detection
- Problem diagrams and trajectory visualizations

**Key Files:** `README.md`, `report.pdf`, `inverse_kin.m`, `forward_kin.m`, `HW2_Main.m`

**Round-trip accuracy:** < 10⁻⁴ rad/m | **Execution:** 5-10 minutes

[Read More →](Parallel-Robot-Kinematics/README.md)

---

### 2. **Serial-Robot-Dynamics** ⚙️
Lagrangian mechanics and forward dynamics simulation

**What's Inside:**
- Lagrangian-based dynamics (H, C, G matrices)
- Inverse dynamics torque planning
- Forward dynamics ODE integration with adaptive step size
- Robustness validation under 100% load uncertainty
- Mathematical derivations from first principles

**Key Files:** `README.md`, `report.pdf`, `dynamics_mat.m`, `tau_plan.m`, `main.m`

**Nominal accuracy:** <0.1% | **Load uncertainty:** 0.3-0.5% error at 100% change

[Read More →](Serial-Robot-Dynamics/README.md)

---

### 3. **Advanced-Control-Systems** 🎯
Comparative analysis of 5 control methodologies

**What's Inside:**
- **PDINV:** Model-based inverse dynamics + PD (optimal)
- **PDG:** PD + gravity compensation (simplified)
- **PID:** Classical model-free robust control
- **MINMAX:** Sliding-mode guaranteed robustness
- **AC:** Adaptive control with online parameter learning

**Key Files:** `README.md`, `report.pdf`, `tau_cont.m`, `main_project4.m`

**Performance:** Compares nominal vs 100% load uncertainty across all 5 methods

[Read More →](Advanced-Control-Systems/README.md)

---

## 📊 Quick Statistics

| Metric | Value |
|--------|-------|
| **MATLAB Files** | 40 |
| **Code** | 1500+ lines |
| **Documentation** | 5000+ lines |
| **Robot Models** | 2 (parallel + serial) |
| **Control Methods** | 5 |
| **Test Scenarios** | 9+ |
| **Report PDFs** | 3 |
| **Diagrams** | 5 |

---

## 🚀 Quick Start

**Prerequisites:** MATLAB R2019a+, Symbolic Math Toolbox, Simulink

```matlab
% Parallel Robot Kinematics
cd Parallel-Robot-Kinematics && run HW2_Main.m

% Serial Robot Dynamics  
cd Serial-Robot-Dynamics && run main.m

% Advanced Control Systems
cd Advanced-Control-Systems && run main_project4.m
```

---

## 🎯 Key Highlights

### Parallel Robot Kinematics
- ✅ Multiple IK solutions: 4-8 per pose
- ✅ FK validation: < 10⁻⁴ error
- ✅ Trajectory smoothness: zero boundary accelerations
- ✅ Singularity detection: 4-6 in workspace

### Serial Robot Dynamics
- ✅ Inverse Dynamics: <0.1% error (nominal)
- ✅ Forward Dynamics: adaptive ODE45 integration
- ✅ Robustness: tested under 3 uncertainty scenarios
- ✅ Validation: comprehensive test suite

### Advanced Control Systems
- ✅ PDINV: optimal (0.1%) but fragile
- ✅ PID: robust (0.2% with 100% load change)
- ✅ MINMAX: guaranteed stability proof
- ✅ AC: learns true model in 2-3 seconds

---

## 🔑 Technical Concepts

**Parallel Kinematics:** Constraint-based solving with multiple solution branches

**Serial Dynamics:** H(q)·q̈ + C(q,q̇)·q̇ + G(q) = τ

**Control Methods:** Model-based optimal → model-free robust → guaranteed robust → adaptive learning

---

## 📖 Each Folder Contains

1. **README.md** - Complete technical documentation with:
   - Problem description with diagrams
   - Mathematical formulations
   - Algorithm explanations
   - Usage examples
   - Validation results

2. **report.pdf** - Original assignment report with:
   - Detailed analysis
   - Mathematical development
   - Experimental results
   - Performance discussion

3. **MATLAB Code** - Production-grade implementation with:
   - Clear comments
   - Proper error handling
   - Modular design
   - Example scripts

---

## 💡 Real-World Applications

- **Autonomous Vehicles:** Trajectory planning and tracking
- **Robotic Manipulation:** Multi-DOF arm control with uncertainty
- **Industrial Automation:** Real-time control with constraints
- **Medical Robotics:** Precise trajectory following
- **Aerospace:** Control under model uncertainty

---

## 🎓 What This Demonstrates

**Theoretical Knowledge:**
- Constraint-based kinematics for parallel mechanisms
- Lagrangian mechanics from first principles
- Nonlinear control theory (5 methodologies)
- Robust control with Lyapunov stability
- Adaptive systems with online learning

**Practical Engineering:**
- MATLAB implementation (1500+ lines)
- Numerical methods (polynomial solving, ODE integration)
- Performance optimization and trade-off analysis
- Validation methodology
- Professional software engineering

**Systems Thinking:**
- Complete robotics pipeline: geometry → dynamics → control
- Multiple solution approaches
- Real-world constraints handling
- Professional documentation

---

## 📝 Citation

```bibtex
@project{KDCR2023,
  title={Kinematics, Dynamics, and Control of Robots},
  author={Manor, Nir and Liu, Sheng},
  school={Technion - Israel Institute of Technology},
  faculty={Mechanical Engineering},
  course={036026},
  year={2023}
}
```

---

## 📄 License

Educational License - See individual folders for details.

---

**Status:** Complete and Production-Ready  
**Version:** 1.0  
**Last Updated:** November 2024

---

**Explore the projects:**
- [Parallel-Robot-Kinematics](Parallel-Robot-Kinematics/README.md)
- [Serial-Robot-Dynamics](Serial-Robot-Dynamics/README.md)
- [Advanced-Control-Systems](Advanced-Control-Systems/README.md)
