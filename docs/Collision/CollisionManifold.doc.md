# CollisionManifold Class Documentation

The `CollisionManifold` structure encapsulates all the information about a detected collision between two rigid bodies. It serves as a data container that bridges collision detection and collision resolution phases in the physics simulation.

## Overview

When two objects collide, we need to know more than just "they collided." We need detailed information to:
- Apply physically accurate impulses
- Separate interpenetrating objects
- Calculate realistic contact responses

The `CollisionManifold` stores all this critical information for one collision event.

## Structure

The manifold contains the following data:

### Colliding Bodies
- `bodyA` - Pointer to the first rigid body in the collision
- `bodyB` - Pointer to the second rigid body in the collision

### Geometric Information
- `contactNormal` - The collision normal vector
- `contactPoint` - The point of contact in world space
- `penetrationDepth` - How far the objects have interpenetrated

## Mathematical Details

### Contact Normal

The contact normal $\vec{n}$ is a unit vector that points from body A toward body B along the shortest path of separation:

$$\vec{n} = \frac{\vec{c}_B - \vec{c}_A}{|\vec{c}_B - \vec{c}_A|}$$

where $\vec{c}_A$ and $\vec{c}_B$ are the centers of the two bodies.

**Properties**:
- Always normalized: $|\vec{n}| = 1$
- Points from A to B by convention
- Perpendicular to the collision surface at the contact point

**Physical Meaning**: The direction in which the collision impulse will be primarily applied.

### Contact Point

For two spheres with centers $\vec{c}_A$ and $\vec{c}_B$ and radii $r_A$ and $r_B$, the contact point $\vec{p}_{contact}$ is calculated as:

$$\vec{p}_{contact} = \vec{c}_A + \vec{n} \cdot \left(r_A - \frac{d_{pen}}{2}\right)$$

where:
- $\vec{n}$ is the contact normal
- $r_A$ is the radius of sphere A
- $d_{pen}$ is the penetration depth

**Physical Meaning**: The actual location in world space where the two objects are touching (or would be touching if they weren't interpenetrating).

**Why It Matters**: The contact point is crucial for calculating:
- The lever arm for torque: $\vec{r} = \vec{p}_{contact} - \vec{c}_{body}$
- Where to apply impulses for realistic rotational response
- Visual effects (sparks, impact marks, etc.)

### Penetration Depth

The penetration depth $d_{pen}$ measures how far the objects have overlapped:

$$d_{pen} = (r_A + r_B) - d$$

where:
- $r_A + r_B$ is the sum of the radii (minimum safe distance)
- $d$ is the actual distance between centers: $d = |\vec{c}_B - \vec{c}_A|$

**Physical Meaning**: 
- When $d_{pen} > 0$: Objects are interpenetrating (collision!)
- When $d_{pen} = 0$: Objects are exactly touching
- When $d_{pen} < 0$: Objects are separated (no collision)

**Why It Matters**: Used for positional correction to prevent objects from sinking into each other.

## Constructor

```cpp
CollisionManifold(
    RigidBody* a, 
    RigidBody* b, 
    Vector3 normal, 
    float depth, 
    Vector3 contact
)
```

Creates a manifold with all the collision data. This is typically called by the collision detection system.

## Usage in Physics Pipeline

### 1. Collision Detection Phase

The `PhysicsWorld::detectCollisions()` method creates manifolds:

```cpp
// Detect if spheres overlap
Vector3 dir = body2->position - body1->position;
float dist = dir.magnitude();
float totalRadius = body1->shape->radius + body2->shape->radius;

if (dist < totalRadius) {
    // Calculate collision data
    Vector3 normal = (dist > 0.0f) ? dir / dist : Vector3(0, 1, 0);
    float penetration = totalRadius - dist;
    Vector3 contact = body1->position + normal * (body1->shape->radius - penetration / 2.0f);
    
    // Create manifold
    collisions.push_back(CollisionManifold(body1, body2, normal, penetration, contact));
}
```

### 2. Collision Resolution Phase

The `PhysicsWorld::resolveCollisions()` method uses manifolds to apply impulses:

```cpp
for (auto manifold : collisions) {
    // Extract data
    RigidBody* A = manifold.bodyA;
    RigidBody* B = manifold.bodyB;
    Vector3 normal = manifold.contactNormal;
    Vector3 contactPoint = manifold.contactPoint;
    float penetration = manifold.penetrationDepth;
    
    // Calculate lever arms
    Vector3 rA = contactPoint - A->position;
    Vector3 rB = contactPoint - B->position;
    
    // ... impulse calculations ...
}
```

## Advanced Collision Information

### Relative Velocity at Contact Point

The velocity at the contact point includes both linear and angular components:

$$\vec{v}_{contact} = \vec{v}_{linear} + \vec{\omega} \times \vec{r}$$

where:
- $\vec{v}_{linear}$ is the body's linear velocity
- $\vec{\omega}$ is the angular velocity
- $\vec{r}$ is the vector from center of mass to contact point

The relative velocity between the two bodies at the contact is:

$$\vec{v}_{rel} = \vec{v}_{contact,B} - \vec{v}_{contact,A}$$

### Velocity Along Normal

The component of relative velocity along the collision normal:

$$v_n = \vec{v}_{rel} \cdot \vec{n}$$

**Interpretation**:
- If $v_n < 0$: Bodies are approaching (need impulse response)
- If $v_n = 0$: Bodies are sliding past each other
- If $v_n > 0$: Bodies are separating (no response needed)

### Impulse Calculation

The collision impulse magnitude is calculated using:

$$j = \frac{-(1 + e) \cdot v_n}{m_A^{-1} + m_B^{-1} + [(\mathbf{I}_A^{-1}(\vec{r}_A \times \vec{n})) \times \vec{r}_A + (\mathbf{I}_B^{-1}(\vec{r}_B \times \vec{n})) \times \vec{r}_B] \cdot \vec{n}}$$

where:
- $e$ is the coefficient of restitution (bounciness)
- $m^{-1}$ is the inverse mass
- $\mathbf{I}^{-1}$ is the inverse inertia tensor
- $\vec{r}$ is the lever arm from center of mass to contact point

The impulse vector is then:

$$\vec{J} = j \cdot \vec{n}$$

### Positional Correction

To prevent objects from sinking into each other due to numerical errors, we apply a correction:

$$\vec{correction} = \vec{n} \cdot \frac{\max(d_{pen} - slop, 0)}{m_A^{-1} + m_B^{-1}} \cdot percent$$

where:
- $slop$ is a small tolerance (typically 0.01) to allow minor penetration
- $percent$ is the correction strength (typically 0.2 or 20%)

This is applied to positions:

$$\vec{p}_A \leftarrow \vec{p}_A - \vec{correction} \cdot m_A^{-1}$$
$$\vec{p}_B \leftarrow \vec{p}_B + \vec{correction} \cdot m_B^{-1}$$

## Why Store Contact Point?

The contact point might seem redundant since we have the normal and penetration depth, but it's crucial because:

1. **Lever Arm Calculation**: $\vec{r} = \vec{p}_{contact} - \vec{c}_{mass}$ determines rotational response
2. **Off-Center Impacts**: Hitting an object off-center creates torque, causing rotation
3. **Stability**: Using a consistent contact point throughout resolution prevents jittering
4. **Multiple Contacts**: For complex shapes, we might need multiple contact points

## Example Scenario

Imagine two spheres colliding:

```
Sphere A:
  - Position: (0, 0, 0)
  - Radius: 1.0
  - Velocity: (2, 0, 0)

Sphere B:
  - Position: (1.8, 0, 0)
  - Radius: 1.0
  - Velocity: (-1, 0, 0)

Collision Manifold:
  - bodyA: pointer to Sphere A
  - bodyB: pointer to Sphere B
  - contactNormal: (1, 0, 0) [normalized direction from A to B]
  - penetrationDepth: 0.2 [2.0 total radius - 1.8 distance = 0.2]
  - contactPoint: (0.9, 0, 0) [on the surface between them]
```

## Performance Considerations

- **Size**: Lightweight structure (2 pointers + 3 Vector3s + 1 float ≈ 44 bytes on 64-bit systems)
- **Allocation**: Created on the stack or in a std::vector for cache efficiency
- **Lifetime**: Typically exists for one frame only, recreated each physics step
- **Copy Semantics**: Uses default copy constructor, safe to pass by value

## Future Enhancements

For more advanced collision handling:
- **Multiple Contact Points**: Store array of contact points for stability
- **Contact ID**: Persistent tracking of contacts across frames for warm-starting
- **Friction Impulse**: Separate tangential impulse for friction
- **Material Properties**: Store combined friction/restitution coefficients
- **Contact Feature**: Track which edges/faces are in contact for better accuracy

