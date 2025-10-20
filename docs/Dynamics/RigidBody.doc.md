# RigidBody Class Documentation

The `RigidBody` class represents a physical object in the simulation with both linear and rotational motion properties. It is the core component of the physics engine, encapsulating all the state and behavior of a dynamic object.

## Overview

A rigid body is a solid object where the distance between any two points remains constant regardless of external forces. This class manages:
- Position and velocity in 3D space
- Orientation and angular velocity
- Mass and inertia properties
- Force and torque accumulation
- Integration of motion equations

## Structure

### Linear Motion Properties

- `position` (`Vector3`) - The location of the body's center of mass in world space
- `velocity` (`Vector3`) - Linear velocity in m/s
- `acceleration` (`Vector3`) - Linear acceleration in m/s²
- `inverseMass` (`float`) - Reciprocal of mass (0 for immovable objects)

**Why inverse mass?** Using $m^{-1}$ instead of $m$ is more efficient:
$$\vec{a} = \vec{F} \times m^{-1}$$
This avoids division operations and naturally handles infinite mass (immovable objects) with $m^{-1} = 0$.

### Rotational Motion Properties

- `orientation` (`Quaternion`) - Rotation of the body as a unit quaternion
- `angularVelocity` (`Vector3`) - Angular velocity in rad/s
- `inverseInertiaTensor` (`Matrix3x3`) - Inverse inertia tensor in body space
- `rotationMatrix` (`Matrix3x3`) - Current rotation matrix (derived from orientation)

### Collision Properties

- `shape` (`CollisionShape*`) - Pointer to the collision geometry (sphere, box, etc.)
- `restitution` (`float`) - Coefficient of restitution (bounciness), range [0, 1]
  - 0.0 = perfectly inelastic (no bounce)
  - 1.0 = perfectly elastic (full bounce)
  - Default: 0.7 (moderately bouncy)

### Private Accumulators

- `accumulatedForces` (`Vector3`) - Sum of all forces applied this frame
- `accumulatedTorque` (`Vector3`) - Sum of all torques applied this frame

These are reset each frame after integration.

## Constructor

```cpp
RigidBody()
```

Initializes a rigid body with default values:
- Orientation: Identity quaternion (1, 0, 0, 0) - no rotation
- Shape: `nullptr` - no collision shape
- Restitution: 0.7 - moderate bounciness
- All other values: Zero-initialized

## Force and Torque Application

### Adding Forces

```cpp
void addForce(Vector3 force)
```

Adds a force to the accumulated forces. Multiple forces can be added before integration.

**Physics**: Forces cause linear acceleration according to Newton's second law:
$$\vec{F} = m \vec{a} \implies \vec{a} = \vec{F} \times m^{-1}$$

**Example usage**:
```cpp
body->addForce(Vector3(0.0f, -9.81f, 0.0f) * mass);  // Gravity
body->addForce(Vector3(10.0f, 0.0f, 0.0f));          // Push force
```

### Adding Torques

```cpp
void addTorque(Vector3 torque)
```

Adds a torque (rotational force) to the accumulated torques.

**Physics**: Torques cause angular acceleration:
$$\vec{\tau} = I \vec{\alpha} \implies \vec{\alpha} = I^{-1} \vec{\tau}$$

where:
- $\vec{\tau}$ is torque
- $I$ is the inertia tensor
- $\vec{\alpha}$ is angular acceleration

**Example usage**:
```cpp
body->addTorque(Vector3(0.0f, 5.0f, 0.0f));  // Spin around Y-axis
```

### Adding Force at a Point

```cpp
void addForceAtPoint(Vector3 force, Vector3 worldPoint)
```

Applies a force at a specific point in world space. This generates both:
1. Linear force (affecting translation)
2. Torque (affecting rotation)

**Physics**: The torque is calculated using the cross product:
$$\vec{\tau} = \vec{r} \times \vec{F}$$

where:
- $\vec{r} = \vec{p}_{point} - \vec{p}_{body}$ is the lever arm (vector from center of mass to force application point)
- $\vec{F}$ is the force vector

**Physical meaning**: Forces applied away from the center of mass cause rotation. The further from the center and the more perpendicular the force, the greater the torque.

**Example usage**:
```cpp
// Hit the right side of an object, causing it to spin
Vector3 hitPoint = body->position + Vector3(1.0f, 0.0f, 0.0f);
Vector3 force = Vector3(0.0f, 0.0f, 10.0f);
body->addForceAtPoint(force, hitPoint);
// This causes the object to move in +Z and rotate around Y
```

### Clearing Accumulators

```cpp
void clearAccumulatedForces()
void clearAccumulatedTorque()
void clearAccumulators()  // Clears both
```

Resets the accumulated forces and torques to zero. This is called automatically after integration but can be called manually if needed.

## Integration

```cpp
void integrate(float dt)
```

The heart of the physics simulation. This method integrates the equations of motion over a time step $\Delta t$, updating position, velocity, and orientation.

### Integration Algorithm

#### 1. Angular Motion (if not immovable)

**Calculate angular acceleration:**
$$\vec{\alpha} = I^{-1}_{body} \vec{\tau}$$

```cpp
Vector3 angularAcceleration = inverseInertiaTensor.transform(accumulatedTorque);
```

**Update angular velocity:**
$$\vec{\omega}_{new} = \vec{\omega}_{old} + \vec{\alpha} \times \Delta t$$

```cpp
angularVelocity += angularAcceleration * dt;
```

**Update orientation quaternion:**

The quaternion is updated using the derivative formula:
$$\dot{q} = \frac{1}{2} q_{spin} \times q$$

where $q_{spin} = (0, \omega_x, \omega_y, \omega_z)$ is a pure quaternion constructed from angular velocity.

Using Euler integration:
$$q_{new} = q_{old} + \dot{q} \times \Delta t$$

```cpp
orientation.updateByAngularVelocity(angularVelocity, dt);
```

#### 2. Normalize Orientation

**Critical step**: After integration, the quaternion may drift from unit length due to numerical errors. Normalization ensures it remains a valid rotation:

$$q_{normalized} = \frac{q}{|q|}$$

```cpp
orientation.normalize();
```

#### 3. Update Rotation Matrix

Convert the quaternion to a rotation matrix for efficient vector transformations:

```cpp
rotationMatrix.setOrientation(orientation);
```

#### 4. Linear Motion (if not immovable)

**Early exit for immovable objects:**
```cpp
if (inverseMass <= 0.0f) {
    clearAccumulators();
    return;
}
```

Objects with $m^{-1} = 0$ (infinite mass) don't move.

**Calculate linear acceleration:**
$$\vec{a} = \vec{F}_{total} \times m^{-1}$$

```cpp
acceleration = accumulatedForces * inverseMass;
```

**Update velocity (semi-implicit Euler):**
$$\vec{v}_{new} = \vec{v}_{old} + \vec{a} \times \Delta t$$

```cpp
velocity += acceleration * dt;
```

**Update position:**
$$\vec{p}_{new} = \vec{p}_{old} + \vec{v}_{new} \times \Delta t$$

```cpp
position += velocity * dt;
```

**Note**: We use the *new* velocity to update position (semi-implicit Euler), which provides better stability than explicit Euler.

#### 5. Clear Accumulators

Reset forces and torques for the next frame:
```cpp
clearAccumulators();
```

### Integration Order

The method integrates angular motion before linear motion. This order matters because:
1. Angular motion updates the rotation matrix
2. The rotation matrix may be needed for collision detection
3. Linear motion is typically the final state update

### Time Step Considerations

The time step $\Delta t$ significantly affects simulation quality:

- **Too large**: Unstable simulation, objects passing through each other
- **Too small**: Computationally expensive, slower simulation
- **Recommended**: 1/60 second (0.0167) or 1/120 second (0.0083) for high accuracy

**Fixed time step**: Use a constant $\Delta t$ for consistent physics, regardless of frame rate:

```cpp
const float dt = 1.0f / 60.0f;  // 60 Hz physics
world.step(dt);
```

## Usage Examples

### Creating a Dynamic Box

```cpp
RigidBody* box = new RigidBody();

// Set mass (inverseMass = 1/mass)
float mass = 10.0f;
box->inverseMass = 1.0f / mass;

// Set position
box->position = Vector3(0.0f, 5.0f, 0.0f);

// Set up inertia tensor for a box
Vector3 dimensions(2.0f, 1.0f, 1.0f);
box->inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, dimensions);

// Add collision shape
box->shape = new BoundingBox(dimensions * 0.5f);  // Half-extents

// Set bounciness
box->restitution = 0.8f;  // Fairly bouncy

// Add to physics world
world.addBody(box);
```

### Creating an Immovable Ground

```cpp
RigidBody* ground = new RigidBody();

// Infinite mass = immovable
ground->inverseMass = 0.0f;

// Position
ground->position = Vector3(0.0f, -1.0f, 0.0f);

// Large box shape
ground->shape = new BoundingBox(Vector3(50.0f, 0.5f, 50.0f));

world.addBody(ground);
```

### Applying Forces

```cpp
// Apply gravity (done automatically by PhysicsWorld)
Vector3 gravity(0.0f, -9.81f, 0.0f);
body->addForce(gravity * (1.0f / body->inverseMass));

// Apply a jump impulse
body->addForce(Vector3(0.0f, 500.0f, 0.0f));

// Apply force at a point (e.g., hitting a ball off-center)
Vector3 hitPoint = ball->position + Vector3(0.5f, 0.0f, 0.0f);
Vector3 force = Vector3(0.0f, 100.0f, 0.0f);
ball->addForceAtPoint(force, hitPoint);  // Ball will move up and spin

// Integrate motion
body->integrate(dt);
```

### Manually Controlling a Body

```cpp
// Set velocity directly (kinematic control)
body->velocity = Vector3(5.0f, 0.0f, 0.0f);

// Set angular velocity directly (spin it)
body->angularVelocity = Vector3(0.0f, 3.14f, 0.0f);  // Spin around Y

// Integration will update position/orientation based on these velocities
body->integrate(dt);
```

## Physics Equations Summary

### Linear Motion
$$\vec{F} = m \vec{a}$$
$$\vec{v}_{t+\Delta t} = \vec{v}_t + \vec{a} \Delta t$$
$$\vec{p}_{t+\Delta t} = \vec{p}_t + \vec{v}_{t+\Delta t} \Delta t$$

### Angular Motion
$$\vec{\tau} = I \vec{\alpha}$$
$$\vec{\omega}_{t+\Delta t} = \vec{\omega}_t + \vec{\alpha} \Delta t$$
$$q_{t+\Delta t} = q_t + \frac{1}{2}(q_{spin} \times q_t) \Delta t$$

### Force at Point
$$\vec{\tau} = \vec{r} \times \vec{F}$$
where $\vec{r} = \vec{p}_{point} - \vec{p}_{body}$

## Important Notes

- **Always normalize the orientation quaternion** after integration to prevent drift
- **Update the rotation matrix** from the orientation before collision detection
- **Use inverse mass and inverse inertia** for efficiency
- **Immovable objects** have `inverseMass = 0` and don't integrate linear motion
- **The shape pointer** must be set for collision detection to work
- **Restitution** affects collision response (handled by PhysicsWorld)

