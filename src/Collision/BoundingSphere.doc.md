# BoundingSphere Class Documentation

The `BoundingSphere` class represents a spherical collision volume used for efficient collision detection in 3D physics simulations. It encapsulates an object with a simple sphere defined by a radius.

## Overview

Bounding spheres are one of the simplest and most efficient collision detection primitives. They are used to:
- Quickly determine if two objects might be colliding
- Provide a simple approximation of an object's volume
- Enable efficient collision detection for spherical objects

## Structure

The bounding sphere is defined by a single property:
- `radius` (`float`) - The distance from the center to the surface of the sphere (in world units)

**Note**: The center position is stored separately in the `RigidBody` that owns this shape.

## Mathematical Representation

A sphere in 3D space is defined by:

$$S = \{\vec{p} \in \mathbb{R}^3 : |\vec{p} - \vec{c}| \leq r\}$$

where:
- $\vec{c}$ is the center position (stored in the rigid body's `position`)
- $r$ is the radius
- $\vec{p}$ is any point on or inside the sphere

The surface area and volume are:
$$A = 4\pi r^2$$
$$V = \frac{4}{3}\pi r^3$$

## Constructor

```cpp
BoundingSphere(float radius)
```

Creates a bounding sphere with the specified radius. The radius should be positive and large enough to encompass the entire object it represents.

**Example**:
```cpp
// Create a sphere with 1 meter radius
BoundingSphere* sphere = new BoundingSphere(1.0f);

// Create a small sphere for a marble
BoundingSphere* marble = new BoundingSphere(0.05f);
```

## Shape Interface

```cpp
ShapeType getType() const override
```

Returns `SPHERE`, identifying this as a sphere shape for collision detection dispatch.

This method is part of the `CollisionShape` interface and is used by the physics engine to determine which collision algorithm to use.

## Collision Detection

Sphere collision detection is handled by the `PhysicsWorld` class. The algorithm varies based on what the sphere is colliding with.

### Sphere-Sphere Collision

The `PhysicsWorld::checkSphereSphere()` method detects collisions between two spheres.

#### Algorithm

Two spheres overlap if the distance between their centers is less than the sum of their radii:

$$\text{collision} \iff |\vec{c}_2 - \vec{c}_1| < r_1 + r_2$$

**Implementation steps:**

1. **Calculate the direction vector**:
   $$\vec{d} = \vec{c}_2 - \vec{c}_1$$

2. **Compute distance**:
   $$dist = |\vec{d}| = \sqrt{d_x^2 + d_y^2 + d_z^2}$$

3. **Calculate total radius**:
   $$r_{total} = r_1 + r_2$$

4. **Check for collision**:
   ```cpp
   if (dist < totalRadius) {
       // Collision detected
   }
   ```

5. **Compute collision normal**:
   $$\vec{n} = \begin{cases}
   \frac{\vec{d}}{dist} & \text{if } dist > 0 \\
   (0, 1, 0) & \text{if } dist = 0
   \end{cases}$$
   
   The fallback handles the rare case where spheres have identical centers.

6. **Calculate penetration depth**:
   $$d_{pen} = r_{total} - dist$$

7. **Find contact point**:
   $$\vec{p}_{contact} = \vec{c}_1 + \vec{n} \cdot \left(r_1 - \frac{d_{pen}}{2}\right)$$
   
   This places the contact point at the midpoint of the overlapping region.

### Sphere-Box Collision

The `PhysicsWorld::checkSphereBox()` method handles sphere-box collisions using a closest-point algorithm.

#### Algorithm

1. **Transform sphere center to box's local space**:
   $$\vec{c}_{local} = R^T \times (\vec{c}_{sphere} - \vec{p}_{box})$$

2. **Find closest point on the box (in local space)**:
   $$\vec{closest}_{local} = \begin{pmatrix}
   \text{clamp}(c_x, -h_x, h_x) \\
   \text{clamp}(c_y, -h_y, h_y) \\
   \text{clamp}(c_z, -h_z, h_z)
   \end{pmatrix}$$
   
   where $(h_x, h_y, h_z)$ are the box's half-extents.

3. **Transform closest point back to world space**:
   $$\vec{closest}_{world} = R \times \vec{closest}_{local} + \vec{p}_{box}$$

4. **Calculate distance to closest point**:
   $$\vec{dir} = \vec{c}_{sphere} - \vec{closest}_{world}$$
   $$dist^2 = |\vec{dir}|^2$$

5. **Check for collision**:
   $$\text{collision} \iff dist^2 < r^2$$

6. **Compute collision data**:
   - **Normal**: $\vec{n} = \frac{\vec{dir}}{dist}$
   - **Penetration**: $d_{pen} = r - dist$
   - **Contact point**: $\vec{closest}_{world}$

## Choosing Radius for Different Objects

For a given object, the bounding sphere radius should typically be:

$$r = \max_{i} |\vec{v}_i - \vec{c}|$$

where $\vec{v}_i$ are the vertices of the object and $\vec{c}$ is its center. This ensures the sphere fully contains the object.

### Common Shapes

- **Point**: $r = 0$
- **Sphere**: Use the actual radius
- **Cube** (side length $s$): $r = \frac{s\sqrt{3}}{2}$ (diagonal to corner)
- **Cylinder** (radius $r_c$, height $h$): $r = \sqrt{r_c^2 + \frac{h^2}{4}}$
- **Arbitrary mesh**: Distance to furthest vertex from center

## Advantages and Limitations

### Advantages

1. **Computational Efficiency**: Only requires basic arithmetic (addition, multiplication, one square root)
2. **Simplicity**: Easy to understand and implement
3. **Rotation Invariant**: Spheres look the same from any angle, so no rotation updates needed
4. **Perfect for round objects**: Natural representation for balls, planets, bubbles
5. **Fast broad phase**: Excellent for quickly eliminating non-colliding pairs
6. **No edge cases**: Unlike boxes, spheres have no edges or corners to handle

### Limitations

1. **Poor fit for non-spherical objects**: Leaves large gaps for elongated or rectangular shapes
2. **Conservative bounds**: May report potential collisions for objects that are close but not touching
3. **Wasted space**: For a cube, the bounding sphere contains ~64% empty space

### When to Use

**Good for:**
- Balls, projectiles, particles
- Quick broad-phase rejection before expensive narrow-phase tests
- Simple games where accuracy isn't critical
- Objects that are roughly spherical

**Not ideal for:**
- Rectangular objects (use `BoundingBox`)
- Elongated objects (use capsules)
- Precise collision detection (use convex hulls or triangle meshes)

## Usage Examples

### Creating a Ball

```cpp
RigidBody* ball = new RigidBody();

// Set mass (5 kg)
ball->inverseMass = 1.0f / 5.0f;

// Position in space
ball->position = Vector3(0.0f, 5.0f, 0.0f);

// Create sphere shape (0.5m radius)
ball->shape = new BoundingSphere(0.5f);

// Set bounciness
ball->restitution = 0.9f;  // Very bouncy

// Set up inertia tensor for a sphere
// For a sphere: I = (2/5) * m * r²
float mass = 5.0f;
float radius = 0.5f;
float I = (2.0f / 5.0f) * mass * radius * radius;
float invI = 1.0f / I;

// For a sphere, the inertia tensor is diagonal with equal values
ball->inverseInertiaTensor.data[0] = invI;
ball->inverseInertiaTensor.data[4] = invI;
ball->inverseInertiaTensor.data[8] = invI;

// Add to physics world
world.addBody(ball);
```

### Creating Multiple Spheres

```cpp
void createSphereGrid(PhysicsWorld& world, int count, float spacing) {
    for (int x = 0; x < count; x++) {
        for (int y = 0; y < count; y++) {
            RigidBody* sphere = new RigidBody();
            sphere->position = Vector3(x * spacing, 10.0f + y * spacing, 0.0f);
            sphere->inverseMass = 0.1f;  // 10 kg
            sphere->shape = new BoundingSphere(0.4f);
            sphere->restitution = 0.7f;
            
            world.addBody(sphere);
        }
    }
}
```

### Large Static Sphere (Ground)

```cpp
// Create a large immovable sphere as ground
RigidBody* ground = new RigidBody();
ground->position = Vector3(0.0f, -100.0f, 0.0f);
ground->inverseMass = 0.0f;  // Infinite mass (immovable)
ground->shape = new BoundingSphere(100.0f);  // Large radius
ground->restitution = 0.5f;

world.addBody(ground);
```

## Integration with Physics Engine

The physics engine automatically handles sphere collision detection:

```cpp
// In PhysicsWorld::findCollisionFeatures()
if (typeA == SPHERE && typeB == SPHERE) {
    checkSphereSphere(bodyA, bodyB);
}
else if (typeA == SPHERE && typeB == BOX) {
    checkSphereBox(bodyA, bodyB);  // Sphere first, box second
}
else if (typeA == BOX && typeB == SPHERE) {
    checkSphereBox(bodyB, bodyA);  // Swap order
}
```

## Performance Characteristics

### Time Complexity
- **Sphere-Sphere**: $O(1)$ - Constant time distance check
- **Sphere-Box**: $O(1)$ - Constant time clamping and distance check

### Memory
- **Storage**: 4 bytes (1 float for radius)
- **Cache friendly**: Minimal memory footprint

### Operations Count
For sphere-sphere collision:
- 3 subtractions (direction vector)
- 3 multiplications + 2 additions (dot product for squared distance)
- 1 square root (distance)
- 2 additions (total radius)
- 1 comparison (collision check)

**Total**: ~10 operations per sphere pair

## Physical Properties

For physics simulation, spheres have simple, uniform properties:

### Moment of Inertia

For a solid sphere of mass $m$ and radius $r$:

$$I = \frac{2}{5}mr^2$$

For a hollow spherical shell:

$$I = \frac{2}{3}mr^2$$

The inverse inertia tensor is diagonal:

$$I^{-1} = \begin{bmatrix}
\frac{5}{2mr^2} & 0 & 0 \\
0 & \frac{5}{2mr^2} & 0 \\
0 & 0 & \frac{5}{2mr^2}
\end{bmatrix}$$

### Rolling Motion

When a sphere rolls without slipping down an incline of angle $\theta$:

$$a = \frac{g\sin\theta}{1 + \frac{I}{mr^2}} = \frac{5}{7}g\sin\theta$$

This is slower than a sliding object ($a = g\sin\theta$) because some energy goes into rotation.

## Implementation Notes

- The sphere shape contains only the radius; position comes from the rigid body
- Collision detection is performed by `PhysicsWorld`, not by the shape itself
- The shape is immutable after creation (radius doesn't change)
- Use `static_cast<BoundingSphere*>` after confirming `getType() == SPHERE`

## Summary

`BoundingSphere` is the simplest and fastest collision shape:
- Defined by a single radius value
- Perfect for spherical objects
- Very efficient collision detection
- Rotation-invariant
- Foundation of broad-phase collision systems
- Best choice for balls, projectiles, and particles
