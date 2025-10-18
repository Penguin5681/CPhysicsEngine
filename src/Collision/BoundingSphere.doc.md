# BoundingSphere Class Documentation

The `BoundingSphere` class represents a spherical collision volume used for efficient collision detection in 3D physics simulations. It encapsulates an object with a simple sphere defined by a radius.

## Overview

Bounding spheres are one of the simplest and most efficient collision detection primitives. They are used to:
- Quickly determine if two objects might be colliding
- Provide a simple approximation of an object's volume
- Enable broad-phase collision detection before more complex narrow-phase checks

## Structure

The bounding sphere is defined by a single property:
- `radius` - The distance from the center to the surface of the sphere (in world units)

**Note**: The center position is stored separately in the `RigidBody` that owns this shape.

## Mathematical Representation

A sphere in 3D space is defined by:

$$S = \{\vec{p} \in \mathbb{R}^3 : |\vec{p} - \vec{c}| \leq r\}$$

where:
- $\vec{c}$ is the center position (stored in the rigid body)
- $r$ is the radius
- $\vec{p}$ is any point on or inside the sphere

## Core Operations

### Constructor

```cpp
BoundingSphere(float radius)
```

Creates a bounding sphere with the specified radius. The radius should be positive and large enough to encompass the entire object it represents.

### Overlap Detection

The `overlaps` method determines if two bounding spheres intersect. This is the fundamental operation for collision detection.

#### Mathematical Formula

Two spheres overlap if the distance between their centers is less than the sum of their radii:

$$\text{overlap} \iff |\vec{c}_2 - \vec{c}_1| < r_1 + r_2$$

where:
- $\vec{c}_1$ and $\vec{c}_2$ are the center positions of the two spheres
- $r_1$ and $r_2$ are their respective radii

#### Optimized Implementation

To avoid the expensive square root operation in distance calculation, we compare squared distances:

$$\text{overlap} \iff |\vec{c}_2 - \vec{c}_1|^2 < (r_1 + r_2)^2$$

Expanded:

$$d^2 = (x_2 - x_1)^2 + (y_2 - y_1)^2 + (z_2 - z_1)^2$$

$$\text{overlap} \iff d^2 < (r_1 + r_2)^2$$

This optimization significantly improves performance in physics simulations where thousands of collision checks may occur per frame.

#### Method Signature

```cpp
bool overlaps(Vector3 thisPosition, Vector3 otherPosition, BoundingSphere* other) const
```

**Parameters**:
- `thisPosition` - World position of this sphere's center
- `otherPosition` - World position of the other sphere's center
- `other` - Pointer to the other bounding sphere to check against

**Returns**: `true` if the spheres overlap, `false` otherwise

## Collision Detection Algorithm

The algorithm follows these steps:

1. **Calculate the vector between centers**:
   $$\vec{d} = \vec{c}_2 - \vec{c}_1$$

2. **Compute squared distance**:
   $$d^2 = \vec{d} \cdot \vec{d} = x_d^2 + y_d^2 + z_d^2$$

3. **Calculate total radius**:
   $$r_{total} = r_1 + r_2$$

4. **Compare squared values**:
   $$\text{if } d^2 < r_{total}^2 \text{ then collision detected}$$

## Advantages and Limitations

### Advantages

1. **Computational Efficiency**: Only requires a few arithmetic operations (no square roots in the optimized version)
2. **Simplicity**: Easy to understand and implement
3. **Rotation Invariant**: Spheres look the same from any angle, so no rotation updates needed
4. **Good for Broad Phase**: Excellent for quickly eliminating non-colliding pairs

### Limitations

1. **Poor Fit for Non-Spherical Objects**: Can leave large gaps for elongated or irregular shapes
2. **Conservative**: May report false positives for objects that are close but not actually touching
3. **No Contact Information**: Only provides yes/no answer, not contact points or normals

## Practical Usage

### Choosing Radius

For a given object, the bounding sphere radius should typically be:

$$r = \max_{i} |\vec{v}_i - \vec{c}|$$

where $\vec{v}_i$ are the vertices of the object and $\vec{c}$ is its center. This ensures the sphere fully contains the object.

For basic shapes:
- **Cube** (side length $s$): $r = \frac{s\sqrt{3}}{2}$
- **Cylinder** (radius $r_c$, height $h$): $r = \sqrt{r_c^2 + \frac{h^2}{4}}$
- **Sphere**: Use the actual radius

### Example Usage

```cpp
// Create two spheres
BoundingSphere sphere1(1.0f);  // 1 meter radius
BoundingSphere sphere2(0.5f);  // 0.5 meter radius

// Define their positions
Vector3 pos1(0.0f, 0.0f, 0.0f);
Vector3 pos2(1.2f, 0.0f, 0.0f);  // 1.2 meters away on X axis

// Check for collision
bool isColliding = sphere1.overlaps(pos1, pos2, &sphere2);
// Result: false (distance 1.2 > sum of radii 1.5)

// Move sphere2 closer
pos2 = Vector3(1.0f, 0.0f, 0.0f);  // 1.0 meters away
isColliding = sphere1.overlaps(pos1, pos2, &sphere2);
// Result: true (distance 1.0 < sum of radii 1.5)
```

## Integration with Physics Engine

In a typical physics simulation:

1. Each `RigidBody` has a `BoundingSphere* shape` member
2. The `PhysicsWorld::detectCollisions()` method checks all pairs using `overlaps()`
3. When overlap is detected, a `CollisionManifold` is created with detailed collision information
4. The `resolveCollisions()` method uses the manifold to apply impulses and resolve penetration

## Performance Considerations

- **Time Complexity**: O(1) per pair check
- **Space Complexity**: O(1) per sphere (just one float)
- **Cache Friendly**: Minimal memory footprint improves CPU cache utilization
- **SIMD Potential**: The squared distance calculation can be vectorized for further optimization

## Future Enhancements

For more complex collision detection, consider:
- **Oriented Bounding Boxes (OBB)**: Better fit for rectangular objects
- **Capsules**: Good for character controllers
- **Convex Hulls**: Accurate collision for arbitrary convex shapes
- **Spatial Partitioning**: Use with octrees or BVH for O(n log n) broad phase

