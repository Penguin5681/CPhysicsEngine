# BoundingBox Class Documentation

The `BoundingBox` class represents an axis-aligned bounding box (AABB) in local space that can be rotated with its parent rigid body. It is used for collision detection with oriented box geometry.

## Overview

A bounding box is a rectangular collision volume defined by its half-extents along each axis. Unlike an axis-aligned bounding box (AABB) that always aligns with world axes, this box rotates with the rigid body it's attached to, making it an **Oriented Bounding Box (OBB)** in world space.

**Use cases**:
- Representing box-shaped objects (crates, buildings, platforms)
- More accurate collision detection than spheres for rectangular objects
- Efficient for many common game objects

## Structure

The bounding box is defined by a single property:

- `halfExtents` (`Vector3`) - Half the dimensions along each local axis

**Half-extents explained**: Instead of storing full width/height/depth, we store half of each dimension:
- `halfExtents.x` = half-width along local X-axis
- `halfExtents.y` = half-height along local Y-axis  
- `halfExtents.z` = half-depth along local Z-axis

**Why half-extents?** They simplify many calculations:
- The local space box ranges from `-halfExtents` to `+halfExtents` on each axis
- Easy to compute corners: just multiply by ±1 on each axis
- Natural for symmetric objects centered at origin

## Mathematical Representation

### In Local Space

In the body's local coordinate system, the box is an axis-aligned bounding box (AABB):

$$B_{local} = \{(x, y, z) : |x| \leq h_x, |y| \leq h_y, |z| \leq h_z\}$$

where $(h_x, h_y, h_z)$ are the half-extents.

**Full dimensions**:
- Width (X): $2h_x$
- Height (Y): $2h_y$
- Depth (Z): $2h_z$

### In World Space

When attached to a rigid body with rotation matrix $R$ and position $\vec{p}$, the box becomes oriented:

$$\vec{v}_{world} = R \times \vec{v}_{local} + \vec{p}$$

The 8 corners of the box in local space are:
$$(\pm h_x, \pm h_y, \pm h_z)$$

Transformed to world space:
$$\vec{corner}_{world} = R \times (\pm h_x, \pm h_y, \pm h_z) + \vec{p}$$

## Constructor

```cpp
BoundingBox(Vector3 halfExtents)
```

Creates a bounding box with specified half-extents.

**Example**:
```cpp
// Box with dimensions 4×2×6 (width×height×depth)
BoundingBox* box = new BoundingBox(Vector3(2.0f, 1.0f, 3.0f));

// Cube with side length 2
BoundingBox* cube = new BoundingBox(Vector3(1.0f, 1.0f, 1.0f));
```

## Shape Interface

```cpp
ShapeType getType() const override
```

Returns `BOX`, identifying this as a box shape for collision detection dispatch.

## Collision Detection

### Box-Box Collision (SAT)

Box-box collision uses the **Separating Axis Theorem (SAT)**. The algorithm tests 15 potential separating axes:
- 3 face normals from box A (local axes)
- 3 face normals from box B (local axes)
- 9 edge-edge cross products

**Separating Axis Theorem**: Two convex objects are *not* colliding if there exists an axis along which their projections don't overlap.

**How it works**:

1. **Project both boxes onto each axis**: For an axis $\vec{a}$ and box vertices $\vec{v}_i$, compute:
   $$\text{proj}_{\vec{a}}(\text{box}) = [\min_i(\vec{v}_i \cdot \vec{a}), \max_i(\vec{v}_i \cdot \vec{a})]$$

2. **Check for overlap**: Two intervals $[min_A, max_A]$ and $[min_B, max_B]$ overlap if:
   $$\max_A > \min_B \text{ AND } \min_A < \max_B$$

3. **If any axis shows no overlap**: Objects are separated (no collision)

4. **If all axes show overlap**: Objects are colliding. The axis with minimum overlap indicates the collision normal and penetration depth.

**Mathematical formula for overlap amount**:
$$\text{overlap} = \min(\max_A, \max_B) - \max(\min_A, \min_B)$$

The implementation in `PhysicsWorld::checkSat()` performs this test efficiently.

### Sphere-Box Collision

Sphere-box collision finds the closest point on the box to the sphere center, then checks if it's within the sphere radius.

**Algorithm**:

1. **Transform sphere center to box's local space**:
   $$\vec{c}_{local} = R^T \times (\vec{c}_{sphere} - \vec{p}_{box})$$
   
   This uses `transformTranspose` since $R^T = R^{-1}$ for rotation matrices.

2. **Find closest point on AABB (in local space)**:
   $$\vec{closest} = (\text{clamp}(c_x, -h_x, h_x), \text{clamp}(c_y, -h_y, h_y), \text{clamp}(c_z, -h_z, h_z))$$
   
   where $\text{clamp}(v, min, max) = \max(min, \min(v, max))$

3. **Transform closest point back to world space**:
   $$\vec{closest}_{world} = R \times \vec{closest}_{local} + \vec{p}_{box}$$

4. **Check distance**:
   $$d^2 = |\vec{c}_{sphere} - \vec{closest}_{world}|^2$$
   
   Collision occurs if $d^2 < r^2$ where $r$ is the sphere radius.

5. **Compute collision data**:
   - **Normal**: $\vec{n} = \frac{\vec{c}_{sphere} - \vec{closest}_{world}}{d}$
   - **Penetration depth**: $d_{pen} = r - d$
   - **Contact point**: $\vec{closest}_{world}$

The implementation is in `PhysicsWorld::checkSphereBox()`.

## Computing 8 Corners

For SAT projection, we need all 8 corners in world space:

```cpp
Vector3 corners[8] = {
    Vector3( halfExtents.x,  halfExtents.y,  halfExtents.z),  // +X +Y +Z
    Vector3(-halfExtents.x,  halfExtents.y,  halfExtents.z),  // -X +Y +Z
    Vector3( halfExtents.x, -halfExtents.y,  halfExtents.z),  // +X -Y +Z
    Vector3( halfExtents.x,  halfExtents.y, -halfExtents.z),  // +X +Y -Z
    Vector3(-halfExtents.x, -halfExtents.y,  halfExtents.z),  // -X -Y +Z
    Vector3(-halfExtents.x,  halfExtents.y, -halfExtents.z),  // -X +Y -Z
    Vector3( halfExtents.x, -halfExtents.y, -halfExtents.z),  // +X -Y -Z
    Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z),  // -X -Y -Z
};

// Transform to world space
for (int i = 0; i < 8; i++) {
    Vector3 worldCorner = body->rotationMatrix.transform(corners[i]) + body->position;
}
```

## Usage Examples

### Creating a Box Rigid Body

```cpp
RigidBody* createBox(Vector3 position, Vector3 dimensions, float mass) {
    RigidBody* box = new RigidBody();
    
    // Set mass
    box->inverseMass = 1.0f / mass;
    box->position = position;
    
    // Create collision shape (dimensions are full size, convert to half-extents)
    Vector3 halfExtents = dimensions * 0.5f;
    box->shape = new BoundingBox(halfExtents);
    
    // Set inertia tensor
    box->inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, dimensions);
    
    return box;
}

// Create a 2×1×3 box
RigidBody* myBox = createBox(
    Vector3(0.0f, 5.0f, 0.0f),  // Position
    Vector3(2.0f, 1.0f, 3.0f),  // Dimensions
    10.0f                        // Mass
);
```

### Creating a Ground Platform

```cpp
RigidBody* ground = new RigidBody();
ground->inverseMass = 0.0f;  // Immovable
ground->position = Vector3(0.0f, -1.0f, 0.0f);

// Large flat platform: 100m wide, 1m tall, 100m deep
ground->shape = new BoundingBox(Vector3(50.0f, 0.5f, 50.0f));

world.addBody(ground);
```

### Creating a Wall

```cpp
RigidBody* wall = new RigidBody();
wall->inverseMass = 0.0f;  // Immovable
wall->position = Vector3(10.0f, 5.0f, 0.0f);

// Thin tall wall: 0.5m thick, 10m tall, 20m wide
wall->shape = new BoundingBox(Vector3(0.25f, 5.0f, 10.0f));

world.addBody(wall);
```

## Advantages and Limitations

### Advantages

1. **Better fit than spheres**: For box-shaped objects, much more accurate collision bounds
2. **Efficient**: SAT algorithm is fast for convex polyhedra
3. **Natural representation**: Most game objects are box-like
4. **Oriented**: Rotates naturally with the rigid body

### Limitations

1. **More complex than spheres**: SAT requires testing multiple axes
2. **Edge cases**: Edge-edge collisions can be tricky to handle perfectly
3. **Contact point approximation**: Current implementation uses box center as contact point (marked as TODO in code)
4. **Not ideal for curved objects**: Use spheres or capsules for round objects

## Performance Considerations

### Time Complexity
- **Sphere-Box**: $O(1)$ - constant time clamping and distance check
- **Box-Box (SAT)**: $O(1)$ - 15 axis tests, 8 vertices each, but constants are higher

### Optimization Tips

1. **Broad phase first**: Use bounding spheres for quick rejection before expensive box tests
2. **Spatial partitioning**: Use octrees/BVH to avoid testing distant boxes
3. **Sleep inactive bodies**: Don't test stationary objects against each other

## Integration with Physics Engine

```cpp
// In PhysicsWorld::findCollisionFeatures()
if (typeA == BOX && typeB == BOX) {
    checkSat(bodyA, bodyB);  // Use SAT algorithm
}
else if (typeA == BOX && typeB == SPHERE) {
    checkSphereBox(bodyB, bodyA);  // Order matters
}
else if (typeA == SPHERE && typeB == BOX) {
    checkSphereBox(bodyA, bodyB);  // Sphere first, box second
}
```

## Future Enhancements

1. **Better contact points**: Implement proper contact point generation (face-face, edge-edge, vertex-face)
2. **Contact manifolds**: Generate multiple contact points for stable stacking
3. **Continuous collision detection**: Prevent tunneling at high velocities
4. **Convex hull support**: Generalize to arbitrary convex shapes

## Mathematical Notes

### Volume
$$V = (2h_x) \times (2h_y) \times (2h_z) = 8h_xh_yh_z$$

### Surface Area
$$A = 2(4h_xh_y + 4h_xh_z + 4h_yh_z) = 8(h_xh_y + h_xh_z + h_yh_z)$$

### Diagonal Length
$$d = \sqrt{(2h_x)^2 + (2h_y)^2 + (2h_z)^2} = 2\sqrt{h_x^2 + h_y^2 + h_z^2}$$

This is the distance from one corner to the opposite corner.

