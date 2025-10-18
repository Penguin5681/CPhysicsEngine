# RigidBody Class Documentation

The `RigidBody` class represents a physical object that responds to forces and torques in a 3D simulation. It handles both linear dynamics (translation) and angular dynamics (rotation), making it suitable for realistic physics simulations.

## Overview

In physics simulation, a rigid body is an idealized solid body that doesn't deform under the influence of forces. The movement of rigid bodies is governed by Newton's laws of motion for both linear and rotational motion.

## Properties

### Linear Motion
- **Position**: $\vec{p}$ - The location of the rigid body's center of mass in 3D space
- **Velocity**: $\vec{v}$ - The rate of change of position over time (m/s)
- **Acceleration**: $\vec{a}$ - The rate of change of velocity over time (m/s²)
- **Inverse Mass**: $m^{-1}$ - Allows representation of infinite mass (immovable objects) as zero

### Angular Motion
- **Orientation**: $q$ - Quaternion representing the body's rotation in 3D space
- **Angular Velocity**: $\vec{\omega}$ - The rate of rotation around an axis (rad/s)
- **Inverse Inertia Tensor**: $\mathbf{I}^{-1}$ - A 3×3 matrix representing resistance to rotation

### Physical Properties
- **Shape**: Pointer to collision shape (e.g., `BoundingSphere`)
- **Restitution**: $e \in [0, 1]$ - Bounciness coefficient (0 = no bounce, 1 = perfect bounce)

### Internal Accumulators
- **Accumulated Forces**: $\vec{F}_{total}$ - Sum of all forces applied during this time step
- **Accumulated Torque**: $\vec{\tau}_{total}$ - Sum of all torques applied during this time step

## Core Concepts

### Mass and Inverse Mass

We use inverse mass ($m^{-1}$) rather than direct mass for two reasons:
1. Division by mass is common in physics calculations: $\vec{a} = \vec{F} \cdot m^{-1}$
2. Infinite mass (immovable objects) can be represented as zero inverse mass

$$m^{-1} = \frac{1}{m}$$

**Special Cases**:
- $m^{-1} = 0$ → Infinite mass (immovable, like the ground)
- $m^{-1} = 1$ → Mass of 1 kg
- $m^{-1} = 0.1$ → Mass of 10 kg

### Inertia Tensor

The inertia tensor $\mathbf{I}$ is the rotational equivalent of mass. It describes how the mass is distributed in the body and determines resistance to rotation.

For a 3D rigid body, the inertia tensor is a 3×3 matrix:

$$\mathbf{I} = \begin{bmatrix}
I_{xx} & I_{xy} & I_{xz} \\
I_{yx} & I_{yy} & I_{yz} \\
I_{zx} & I_{zy} & I_{zz}
\end{bmatrix}$$

**For simple shapes aligned with axes**, the inertia tensor is diagonal:

$$\mathbf{I} = \begin{bmatrix}
I_x & 0 & 0 \\
0 & I_y & 0 \\
0 & 0 & I_z
\end{bmatrix}$$

We store the **inverse** inertia tensor $\mathbf{I}^{-1}$ for the same reasons we use inverse mass.

### Forces and Torques

#### Force

A force $\vec{F}$ causes linear acceleration:

$$\vec{F} = m\vec{a} \quad \Rightarrow \quad \vec{a} = \vec{F} \cdot m^{-1}$$

#### Torque

A torque $\vec{\tau}$ (also called moment) causes angular acceleration:

$$\vec{\tau} = \mathbf{I}\vec{\alpha} \quad \Rightarrow \quad \vec{\alpha} = \mathbf{I}^{-1}\vec{\tau}$$

where $\vec{\alpha}$ is angular acceleration (rad/s²).

#### Force at a Point

When a force is applied at a specific point (not at the center of mass), it creates both linear motion AND rotation:

$$\vec{F}_{applied} \text{ at point } \vec{p}_{world}$$

This produces:
1. **Linear force**: The full force vector $\vec{F}$
2. **Torque**: $\vec{\tau} = \vec{r} \times \vec{F}$

where $\vec{r} = \vec{p}_{world} - \vec{p}_{COM}$ is the lever arm (vector from center of mass to application point).

**Cross Product for Torque**:
$$\vec{\tau} = \vec{r} \times \vec{F} = \begin{bmatrix}
r_y F_z - r_z F_y \\
r_z F_x - r_x F_z \\
r_x F_y - r_y F_x
\end{bmatrix}$$

**Physical Meaning**: The longer the lever arm perpendicular to the force, the more torque is generated.

### Integration

The numerical integration process updates the position and velocity of the rigid body over time.

#### Linear Integration (Semi-Implicit Euler)

$$\vec{a} = \vec{F}_{total} \cdot m^{-1}$$
$$\vec{v}_{new} = \vec{v}_{old} + \vec{a} \cdot \Delta t$$
$$\vec{p}_{new} = \vec{p}_{old} + \vec{v}_{new} \cdot \Delta t$$

**Note**: We use $\vec{v}_{new}$ for position update (semi-implicit/symplectic Euler), which is more stable than basic Euler.

#### Angular Integration

$$\vec{\alpha} = \mathbf{I}^{-1} \vec{\tau}_{total}$$
$$\vec{\omega}_{new} = \vec{\omega}_{old} + \vec{\alpha} \cdot \Delta t$$

#### Orientation Update

The orientation quaternion is updated using the angular velocity:

$$\dot{q} = \frac{1}{2} q_{spin} \times q$$

where $q_{spin} = (0, \omega_x, \omega_y, \omega_z)$

Using Euler integration:
$$q_{new} = q_{old} + \dot{q} \cdot \Delta t$$

After integration, normalize to prevent drift:
$$q_{new} \leftarrow \frac{q_{new}}{|q_{new}|}$$

## Key Methods

### Force Management

#### `addForce(Vector3 force)`
Adds a force to the accumulated forces. Forces are applied at the center of mass, causing only linear acceleration.

**Example**:
```cpp
Vector3 gravity(0, -9.81f, 0);
body.addForce(gravity * (1.0f / body.inverseMass)); // Apply weight
```

#### `addTorque(Vector3 torque)`
Adds a pure torque (rotational force) to the accumulated torques.

**Example**:
```cpp
Vector3 spinTorque(0, 10.0f, 0); // Spin around Y axis
body.addTorque(spinTorque);
```

#### `addForceAtPoint(Vector3 force, Vector3 worldPoint)`
Adds a force at a specific world position, creating both linear force and torque.

**Implementation**:
```cpp
void addForceAtPoint(Vector3 force, Vector3 worldPoint) {
    this->addForce(force);
    Vector3 r = worldPoint - this->position;
    Vector3 torque = r.cross(force);
    this->addTorque(torque);
}
```

**Physical Example**: Hitting a billiard ball off-center creates both translation and spin.

#### `clearAccumulators()`
Resets both force and torque accumulators to zero. Called after each integration step.

### Physics Integration

#### `integrate(float dt)`
The core physics update method. Updates both linear and angular motion based on accumulated forces and torques.

**Algorithm**:
1. **Safety check**: Skip if inverse mass is zero (infinite mass)
2. **Linear acceleration**: $\vec{a} = \vec{F}_{total} \cdot m^{-1}$
3. **Update linear velocity**: $\vec{v} += \vec{a} \cdot \Delta t$
4. **Update position**: $\vec{p} += \vec{v} \cdot \Delta t$
5. **Angular acceleration**: $\vec{\alpha} = \mathbf{I}^{-1} \vec{\tau}_{total}$
6. **Update angular velocity**: $\vec{\omega} += \vec{\alpha} \cdot \Delta t$
7. **Update orientation**: $q$ using quaternion integration
8. **Normalize orientation**: Prevent drift
9. **Clear accumulators**: Ready for next frame

## Restitution (Bounciness)

The coefficient of restitution $e$ determines how much kinetic energy is retained after a collision:

$$e = \frac{\text{relative velocity after collision}}{\text{relative velocity before collision}}$$

**Values**:
- $e = 0.0$ - Perfectly inelastic (clay, putty) - no bounce
- $e = 0.3$ - Soft objects (soft rubber ball)
- $e = 0.7$ - Default value - moderately bouncy
- $e = 0.9$ - Hard objects (steel ball)
- $e = 1.0$ - Perfectly elastic - perfect bounce (theoretical)

**Energy Loss**:
$$E_{after} = e^2 \cdot E_{before}$$

## Complete Physics Equations

### Newton's Second Law (Linear)
$$\sum \vec{F} = m \vec{a}$$

### Newton's Second Law (Rotational)
$$\sum \vec{\tau} = \mathbf{I} \vec{\alpha}$$

### Kinematic Equations
$$\vec{v}(t) = \vec{v}_0 + \int_0^t \vec{a}(\tau) \, d\tau$$
$$\vec{p}(t) = \vec{p}_0 + \int_0^t \vec{v}(\tau) \, d\tau$$

### Angular Kinematic Equations
$$\vec{\omega}(t) = \vec{\omega}_0 + \int_0^t \vec{\alpha}(\tau) \, d\tau$$
$$q(t) = q_0 + \int_0^t \frac{1}{2}q_{spin}(\tau) \times q(\tau) \, d\tau$$

## Usage Example

```cpp
// Create a rigid body sphere (1 kg, 0.5m radius)
RigidBody* ball = new RigidBody();
ball->position = Vector3(0.0f, 10.0f, 0.0f);
ball->velocity = Vector3(5.0f, 0.0f, 0.0f);
ball->inverseMass = 1.0f; // 1 kg
ball->restitution = 0.8f; // Bouncy

// Set up a sphere collision shape
ball->shape = new BoundingSphere(0.5f);

// Set up inertia tensor for a solid sphere
// I = (2/5) * m * r^2 for each axis
float mass = 1.0f / ball->inverseMass;
float radius = 0.5f;
float I = 0.4f * mass * radius * radius;
ball->inverseInertiaTensor.data[0] = 1.0f / I;
ball->inverseInertiaTensor.data[4] = 1.0f / I;
ball->inverseInertiaTensor.data[8] = 1.0f / I;

// Simulation loop
Vector3 gravity(0.0f, -9.81f, 0.0f);
float timeStep = 0.016f; // ~60fps

while (simulating) {
    // Apply gravity
    ball->addForce(gravity * mass);
    
    // Apply a force at a point (creates spin)
    Vector3 hitPoint = ball->position + Vector3(0.3f, 0.0f, 0.0f);
    Vector3 hitForce(0.0f, 0.0f, 10.0f);
    ball->addForceAtPoint(hitForce, hitPoint);
    
    // Update physics
    ball->integrate(timeStep);
    
    // Render
    render(ball->position, ball->orientation);
}
```

## Implementation Considerations

### Stability
- **Semi-implicit Euler**: More stable than basic Euler for energy conservation
- **Quaternion normalization**: Essential to prevent orientation drift
- **Force clearing**: Always clear forces after integration to prevent accumulation

### Infinite Mass Objects
Objects with `inverseMass = 0` are treated as immovable:
```cpp
if (inverseMass <= 0.0) {
    return; // Skip integration
}
```

### Damping (Future Enhancement)
Real objects experience drag. Linear and angular damping could be added:
$$\vec{v}_{new} = \vec{v}_{old} \cdot (1 - damping)^{\Delta t}$$
$$\vec{\omega}_{new} = \vec{\omega}_{old} \cdot (1 - angularDamping)^{\Delta t}$$

## Advanced Topics

### Moments of Inertia for Common Shapes

**Solid Sphere** (radius $r$, mass $m$):
$$I_x = I_y = I_z = \frac{2}{5}mr^2$$

**Solid Cuboid** (dimensions $w \times h \times d$, mass $m$):
$$I_x = \frac{1}{12}m(h^2 + d^2)$$
$$I_y = \frac{1}{12}m(w^2 + d^2)$$
$$I_z = \frac{1}{12}m(w^2 + h^2)$$

**Solid Cylinder** (radius $r$, height $h$, mass $m$, axis along Y):
$$I_x = I_z = \frac{1}{12}m(3r^2 + h^2)$$
$$I_y = \frac{1}{2}mr^2$$

### Parallel Axis Theorem
For off-center rotations:
$$I = I_{COM} + md^2$$

where $d$ is the distance from the center of mass to the rotation axis.

## Physics Principles Demonstrated

1. **Newton's Laws**: Force causes acceleration proportional to mass
2. **Conservation of Momentum**: Impulses preserve total momentum
3. **Conservation of Angular Momentum**: Torque impulses preserve angular momentum
4. **Energy Dissipation**: Restitution controls energy loss in collisions
5. **Superposition**: Multiple forces and torques can be accumulated and applied together

## Performance Notes

- **Memory**: ~200 bytes per rigid body (including shape pointer)
- **CPU**: Integration is O(1) per body
- **Cache Friendly**: Compact data structure
- **SIMD Potential**: Vector operations can be vectorized

This class forms the foundation of the entire physics simulation, working in conjunction with `PhysicsWorld` for collision detection and resolution.
