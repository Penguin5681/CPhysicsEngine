# CollisionShape Class Documentation

The `CollisionShape` class is an abstract base class that defines the interface for all collision geometry types in the physics engine. It uses polymorphism to allow different collision shapes to be handled uniformly.

## Overview

The collision system needs to work with various geometric shapes (spheres, boxes, capsules, etc.) without rigid bodies knowing the specific implementation details. `CollisionShape` provides this abstraction through a virtual interface.

**Design Pattern**: This is an example of the **Strategy Pattern** - different collision algorithms are selected based on the shape type at runtime.

## Structure

This is an abstract base class with:
- A virtual destructor for proper cleanup
- A pure virtual method to identify the shape type

## Shape Types

The `ShapeType` enumeration defines all supported collision geometries:

```cpp
enum ShapeType {
    SPHERE,
    BOX
};
```

Currently supports:
- **SPHERE** - Spherical collision volume (simplest, fastest)
- **BOX** - Oriented bounding box (rectangular prism)

**Extensibility**: Additional shapes can be added by:
1. Adding to the `ShapeType` enum
2. Creating a new class that inherits from `CollisionShape`
3. Implementing collision detection logic in `PhysicsWorld`

## Interface

### Virtual Destructor

```cpp
virtual ~CollisionShape() {}
```

**Purpose**: Ensures that when a `CollisionShape*` is deleted, the correct destructor is called for the derived class (sphere, box, etc.).

**Why virtual?** Without a virtual destructor:
```cpp
CollisionShape* shape = new BoundingSphere(1.0f);
delete shape;  // Would only call ~CollisionShape, leaking BoundingSphere resources
```

With virtual destructor:
```cpp
delete shape;  // Correctly calls ~BoundingSphere, then ~CollisionShape
```

### Get Type Method

```cpp
virtual ShapeType getType() const = 0;
```

**Pure virtual method** (= 0): Each derived class must implement this.

**Purpose**: Allows runtime type identification for collision dispatch.

**Returns**: The `ShapeType` enum value for this shape.

## Usage in Collision Detection

The physics engine uses shape types for **double dispatch** - selecting the correct collision algorithm based on both shapes:

```cpp
void PhysicsWorld::findCollisionFeatures(RigidBody* bodyA, RigidBody* bodyB) {
    ShapeType typeA = bodyA->shape->getType();
    ShapeType typeB = bodyB->shape->getType();
    
    // Dispatch to appropriate collision function
    if (typeA == SPHERE && typeB == SPHERE) {
        checkSphereSphere(bodyA, bodyB);
    }
    else if (typeA == BOX && typeB == BOX) {
        checkSat(bodyA, bodyB);  // Separating Axis Theorem
    }
    else if (typeA == BOX && typeB == SPHERE) {
        checkSphereBox(bodyB, bodyA);  // Order: sphere first
    }
    else if (typeA == SPHERE && typeB == BOX) {
        checkSphereBox(bodyA, bodyB);
    }
}
```

## Derived Classes

### BoundingSphere

Inherits from `CollisionShape` and implements:
```cpp
class BoundingSphere : public CollisionShape {
public:
    float radius;
    
    ShapeType getType() const override {
        return SPHERE;
    }
};
```

**Use case**: Round objects (balls, planets, particles)

### BoundingBox

Inherits from `CollisionShape` and implements:
```cpp
class BoundingBox : public CollisionShape {
public:
    Vector3 halfExtents;
    
    ShapeType getType() const override {
        return BOX;
    }
};
```

**Use case**: Rectangular objects (crates, walls, platforms)

## Usage Examples

### Creating Rigid Bodies with Different Shapes

```cpp
// Create a sphere
RigidBody* ball = new RigidBody();
ball->shape = new BoundingSphere(1.0f);  // Radius 1m
ball->inverseMass = 1.0f / 5.0f;         // Mass 5kg
world.addBody(ball);

// Create a box
RigidBody* crate = new RigidBody();
crate->shape = new BoundingBox(Vector3(0.5f, 0.5f, 0.5f));  // 1m cube
crate->inverseMass = 1.0f / 10.0f;  // Mass 10kg
world.addBody(crate);

// The physics engine handles collision between different types automatically
```

### Checking Shape Type

```cpp
if (body->shape->getType() == SPHERE) {
    BoundingSphere* sphere = static_cast<BoundingSphere*>(body->shape);
    std::cout << "Sphere with radius: " << sphere->radius << std::endl;
}
else if (body->shape->getType() == BOX) {
    BoundingBox* box = static_cast<BoundingBox*>(body->shape);
    std::cout << "Box with extents: " << box->halfExtents.x << ", " 
              << box->halfExtents.y << ", " << box->halfExtents.z << std::endl;
}
```

### Safe Casting

```cpp
void processShape(CollisionShape* shape) {
    switch (shape->getType()) {
        case SPHERE: {
            BoundingSphere* sphere = static_cast<BoundingSphere*>(shape);
            // Safe to use sphere-specific members
            float volume = (4.0f / 3.0f) * 3.14159f * sphere->radius * sphere->radius * sphere->radius;
            break;
        }
        case BOX: {
            BoundingBox* box = static_cast<BoundingBox*>(shape);
            // Safe to use box-specific members
            Vector3 dimensions = box->halfExtents * 2.0f;
            float volume = dimensions.x * dimensions.y * dimensions.z;
            break;
        }
    }
}
```

## Design Benefits

### Polymorphism Advantages

1. **Uniform Interface**: `RigidBody` doesn't need to know about specific shape types
   ```cpp
   class RigidBody {
       CollisionShape* shape;  // Works with any shape type
   };
   ```

2. **Easy Extension**: Add new shapes without modifying existing code:
   ```cpp
   enum ShapeType {
       SPHERE,
       BOX,
       CAPSULE,    // New!
       CYLINDER    // New!
   };
   ```

3. **Runtime Flexibility**: Change shapes dynamically:
   ```cpp
   delete body->shape;
   body->shape = new BoundingSphere(2.0f);  // Switch from box to sphere
   ```

### Collision Dispatch Table

The shape type enables efficient collision function selection:

| Type A | Type B | Function |
|--------|--------|----------|
| SPHERE | SPHERE | `checkSphereSphere()` |
| SPHERE | BOX | `checkSphereBox()` |
| BOX | SPHERE | `checkSphereBox()` (swapped) |
| BOX | BOX | `checkSat()` |

**Time Complexity**: $O(1)$ dispatch using enum comparison

## Memory Management

### Ownership Rules

1. **RigidBody owns its shape**: When a body is deleted, its shape should be deleted
2. **Manual deletion required**: Current implementation uses raw pointers

```cpp
// Proper cleanup
RigidBody* body = new RigidBody();
body->shape = new BoundingSphere(1.0f);

// Later, clean up
delete body->shape;  // Delete shape first
delete body;         // Then delete body
```

### Recommended: Smart Pointers

For safer memory management, consider using `std::unique_ptr`:

```cpp
class RigidBody {
    std::unique_ptr<CollisionShape> shape;
};

// Automatic cleanup
body->shape = std::make_unique<BoundingSphere>(1.0f);
// No manual delete needed!
```

## Future Extensions

### Potential New Shapes

1. **Capsule**: Cylinder with hemispherical ends
   - Great for character controllers
   - Smooth collisions without edge catching

2. **Cylinder**: Circular cross-section
   - Wheels, pillars, barrels

3. **Convex Hull**: Arbitrary convex polyhedron
   - Complex shapes with exact collision

4. **Compound Shape**: Multiple sub-shapes
   - Complex objects (cars, furniture)

### Implementation Pattern

```cpp
// Add to enum
enum ShapeType {
    SPHERE,
    BOX,
    CAPSULE
};

// Create new class
class BoundingCapsule : public CollisionShape {
public:
    float radius;
    float height;
    
    BoundingCapsule(float r, float h) : radius(r), height(h) {}
    
    ShapeType getType() const override {
        return CAPSULE;
    }
};

// Add collision detection
void PhysicsWorld::checkSphereCapsule(RigidBody* sphere, RigidBody* capsule) {
    // Implementation...
}
```

## Best Practices

1. **Always check for nullptr**: Before using `shape`, verify it's not null
   ```cpp
   if (body->shape != nullptr) {
       ShapeType type = body->shape->getType();
   }
   ```

2. **Use consistent ordering**: For symmetric collision pairs (A vs B), establish a convention
   ```cpp
   // Always pass sphere first, box second
   checkSphereBox(sphereBody, boxBody);
   ```

3. **Validate casts**: After `static_cast`, the shape type is guaranteed by `getType()`
   ```cpp
   if (shape->getType() == SPHERE) {
       auto* sphere = static_cast<BoundingSphere*>(shape);
       // Safe to use sphere->radius
   }
   ```

## Performance Considerations

- **Virtual call overhead**: `getType()` is a virtual function call, but modern CPUs handle this efficiently
- **Branch prediction**: Collision dispatch branches are consistent, helping CPU branch prediction
- **Cache locality**: Shape type determines memory access patterns for collision data

## Summary

`CollisionShape` provides a clean, extensible interface for collision geometry:
- Abstract base class with virtual destructor
- Pure virtual `getType()` for runtime type identification
- Enables polymorphic collision detection
- Easy to extend with new shape types
- Forms the foundation of the collision system

