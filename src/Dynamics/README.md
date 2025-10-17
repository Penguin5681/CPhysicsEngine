# RigidBody Class Documentation

The `RigidBody` class represents a physical object that responds to forces in a simulation. It handles the dynamics of objects including position, velocity, acceleration, and the accumulation and application of forces.

## Overview

In physics simulation, a rigid body is an idealized solid body that doesn't deform under the influence of forces. The movement of rigid bodies is governed by Newton's laws of motion.

## Properties

- **Position**: $\vec{p}$ - The location of the rigid body in 3D space
- **Velocity**: $\vec{v}$ - The rate of change of position over time
- **Acceleration**: $\vec{a}$ - The rate of change of velocity over time
- **Inverse Mass**: $m^{-1}$ - Allows representation of infinite mass (immovable objects) as zero
- **Accumulated Forces**: The sum of all forces currently acting on the body

## Core Concepts

### Mass and Inverse Mass

We use inverse mass ($m^{-1}$) rather than direct mass for two reasons:
- Division by mass is common in physics calculations
- Infinite mass (immovable objects) can be represented as zero inverse mass

$$m^{-1} = \frac{1}{m}$$

### Force and Acceleration

According to Newton's second law:

$$\vec{F} = m\vec{a}$$

Therefore:

$$\vec{a} = \vec{F} \cdot m^{-1}$$

### Integration

The numerical integration process updates the position and velocity of the rigid body over time. Using Euler integration:

$$\vec{v}_{new} = \vec{v}_{old} + \vec{a} \cdot \Delta t$$
$$\vec{p}_{new} = \vec{p}_{old} + \vec{v}_{new} \cdot \Delta t$$

## Key Methods

### Force Management

- `addForce(Vector3 force)`: Adds a force to the accumulated forces
- `clearAccumulatedForces()`: Resets accumulated forces to zero, called after each integration step

### Physics Integration

- `integrate(float deltaTime)`: Updates position and velocity based on acceleration and accumulated forces

## Usage Example

```cpp
// Create a rigid body with mass 2
RigidBody body;
body.setMass(2.0f);

// Set initial position and velocity
body.position = Vector3(0.0f, 10.0f, 0.0f);
body.velocity = Vector3(5.0f, 0.0f, 0.0f);

// Apply gravity force in each simulation step
Vector3 gravity(0.0f, -9.81f, 0.0f);
float timeStep = 0.016f; // ~60fps

// Main simulation loop
while (simulating) {
    // Apply forces
    body.addForce(gravity * body.getMass());
    
    // Update physics
    body.integrate(timeStep);
    
    // Render at new position
    render(body.position);
}
```

## Implementation Considerations

- Damping should be applied to simulate drag and prevent unrealistic perpetual motion
- The integrate method should handle the case where inverse mass is zero (infinite mass)
- For stability, consider more advanced integration methods like Verlet or RK4 for the final implementation