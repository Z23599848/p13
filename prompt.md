You are an autonomous engineering AI tasked with designing and implementing a **minimal physics simulation of a falling rigid stick with attitude control**.

## Objective

Create a **simple Python simulation and visualizer** that models a rigid stick falling through air and attempting to **reorient vertically and land upright** using:

* reaction wheels for attitude control
* a gimballed propeller providing thrust and torque

The goal is **concept validation**, not engineering accuracy.

---

# System description

The system is a **rigid rod ("stick")** initially at altitude and entering **free fall**.

The vehicle contains:

1. **Reaction wheels**

   * up to 4 wheels
   * produce body torques only
   * limited torque and max angular momentum

2. **Brushless propeller**

   * produces thrust force
   * mounted on a **2-axis gimbal**
   * thrust magnitude is controllable

3. **Sensors (idealized)**

   * perfect knowledge of position
   * perfect knowledge of orientation
   * perfect angular velocity

---

# Physics model

Implement a simplified **6-DOF rigid body model**.

State vector:

position:
x, y, z

velocity:
vx, vy, vz

orientation:
roll, pitch, yaw

angular velocity:
wx, wy, wz

Forces:

* gravity
* propeller thrust
* optional drag

Torques:

* reaction wheel torque
* torque from thrust vector offset

---

# Numerical integration

Use simple Euler integration.

Typical timestep:

dt = 0.01 seconds

---

# Control objective

The controller should attempt to:

1. rotate the stick so its long axis becomes **vertical**
2. reduce angular velocity
3. guide the body toward a target landing point
4. attempt vertical touchdown

The controller can be simple:

* PID
* LQR
* heuristic stabilization

Accuracy is not critical.

---

# Visualization

Provide a **real-time visual simulation** showing:

* the falling stick
* its orientation
* trajectory
* thrust vector

Use one of these libraries:

* vpython (preferred)
* matplotlib animation
* pybullet

---

# Output requirements

The final project must contain:

1. a single runnable Python script
2. clear code structure
3. adjustable parameters
4. comments explaining physics assumptions
5. simple real-time visualization

---

# Development plan

Follow these steps:

1. Implement rigid body state
2. Implement gravity and free fall
3. Add stick orientation update
4. Add reaction wheel torque
5. Add propeller thrust
6. Implement stabilization controller
7. Implement visualization
8. Test different initial conditions

---

# Code requirements

Use:

Python
numpy
scipy (optional)
vpython or matplotlib

Avoid heavy frameworks.

The code should be **under ~300 lines if possible**.

---

# Deliverables

Produce:

1. complete Python code
2. explanation of model
3. instructions for running the simulation
4. notes on limitations
