# Vector3 Class Documentation

The `Vector3` class represents a three-dimensional vector with x, y, and z components. This class provides essential vector operations used for physics simulations and 3D mathematics.

## Overview

A 3D vector can represent positions, directions, velocities, forces, and other physical quantities in three-dimensional space.

## Structure

The vector is stored as three float values:
- `x` - X-component
- `y` - Y-component  
- `z` - Z-component

## Basic Operations

### Constructors

- **Default constructor**: Creates a zero vector (0, 0, 0)
- **Parameterized constructor**: Creates a vector with specified x, y, and z components

```cpp
Vector3 zero;                        // (0, 0, 0)
Vector3 position(1.0f, 2.0f, 3.0f); // (1, 2, 3)
```

### Arithmetic Operations

#### Addition
Vector addition combines two vectors component-wise:

$$\vec{a} + \vec{b} = (a_x + b_x, a_y + b_y, a_z + b_z)$$

#### Subtraction
Vector subtraction subtracts components:

$$\vec{a} - \vec{b} = (a_x - b_x, a_y - b_y, a_z - b_z)$$

#### Scalar Multiplication
Multiplying by a scalar scales all components:

$$\vec{a} \times s = (a_x \times s, a_y \times s, a_z \times s)$$

#### Scalar Division
Dividing by a scalar divides all components:

$$\vec{a} \div s = (a_x \div s, a_y \div s, a_z \div s)$$

**Warning**: Division by zero will result in undefined behavior. Ensure the scalar is non-zero.

### In-Place Operations

The class provides in-place versions of arithmetic operations for better performance:

- `operator+=` - Add to this vector
- `operator-=` - Subtract from this vector
- `operator*=` - Multiply this vector by a scalar
- `operator/=` - Divide this vector by a scalar

These modify the vector directly instead of creating a new one:

```cpp
Vector3 velocity(1.0f, 0.0f, 0.0f);
Vector3 acceleration(0.0f, -9.81f, 0.0f);
velocity += acceleration * 0.016f;  // Update velocity in-place
```

## Vector Mathematics

### Magnitude

The magnitude (length) of a vector is calculated using the Euclidean distance formula:

$$|\vec{v}| = \sqrt{x^2 + y^2 + z^2}$$

```cpp
Vector3 v(3.0f, 4.0f, 0.0f);
float length = v.magnitude();  // Returns 5.0
```

### Magnitude Squared

For performance-critical code, comparing squared magnitudes avoids the expensive square root operation:

$$|\vec{v}|^2 = x^2 + y^2 + z^2$$

**Use case**: When comparing distances, use `magnitudeSquared()` instead of `magnitude()` for better performance.

```cpp
// Fast: No square root
if (vector.magnitudeSquared() < maxDistSquared) {
    // Vector is within range
}

// Slower: Requires square root
if (vector.magnitude() < maxDist) {
    // Same check, but slower
}
```

### Normalization

A normalized (unit) vector has the same direction but a magnitude of 1:

$$\hat{v} = \frac{\vec{v}}{|\vec{v}|} = \left(\frac{x}{|\vec{v}|}, \frac{y}{|\vec{v}|}, \frac{z}{|\vec{v}|}\right)$$

**Two methods are provided:**

1. `normalized()` - Returns a new normalized vector (const method)
2. `normalize()` - Normalizes the vector in-place

**Zero vector handling**: If the magnitude is zero, both methods return/produce a zero vector to avoid division by zero.

```cpp
Vector3 direction(3.0f, 4.0f, 0.0f);
Vector3 unitDir = direction.normalized();  // (0.6, 0.8, 0)
direction.normalize();  // Now direction is (0.6, 0.8, 0)
```

### Dot Product

The dot product between two vectors produces a scalar value:

$$\vec{a} \cdot \vec{b} = a_x \times b_x + a_y \times b_y + a_z \times b_z$$

**Alternative formula**:

$$\vec{a} \cdot \vec{b} = |\vec{a}| \times |\vec{b}| \times \cos(\theta)$$

where $\theta$ is the angle between the vectors.

**Properties and applications**:

- If $\vec{a} \cdot \vec{b} > 0$: Vectors form an acute angle (< 90°)
- If $\vec{a} \cdot \vec{b} = 0$: Vectors are perpendicular (orthogonal)
- If $\vec{a} \cdot \vec{b} < 0$: Vectors form an obtuse angle (> 90°)

**Uses in physics**:
- Calculating work: $W = \vec{F} \cdot \vec{d}$ (force dot displacement)
- Projecting one vector onto another
- Determining if objects are moving toward or away from each other

```cpp
Vector3 velocity(5.0f, 0.0f, 0.0f);
Vector3 normal(1.0f, 0.0f, 0.0f);
float velocityAlongNormal = velocity.dot(normal);  // 5.0
```

### Cross Product

The cross product between two vectors produces a new vector perpendicular to both:

$$\vec{a} \times \vec{b} = \begin{pmatrix}
a_y \times b_z - a_z \times b_y \\
a_z \times b_x - a_x \times b_z \\
a_x \times b_y - a_y \times b_x
\end{pmatrix}$$

**Properties**:
- The result is perpendicular to both input vectors
- The magnitude equals the area of the parallelogram formed by the vectors: $|\vec{a} \times \vec{b}| = |\vec{a}| \times |\vec{b}| \times \sin(\theta)$
- **Not commutative**: $\vec{a} \times \vec{b} = -(\vec{b} \times \vec{a})$
- Follows the right-hand rule for direction

**Uses in physics**:
- Calculating torque: $\vec{\tau} = \vec{r} \times \vec{F}$ (lever arm cross force)
- Computing angular velocity effects: $\vec{v}_{point} = \vec{\omega} \times \vec{r}$
- Finding surface normals from two edge vectors

```cpp
Vector3 r(1.0f, 0.0f, 0.0f);      // Lever arm
Vector3 force(0.0f, 0.0f, 10.0f);  // Force
Vector3 torque = r.cross(force);   // (0, -10, 0)
```

## Usage Examples

```cpp
// Create vectors
Vector3 position(1.0f, 2.0f, 3.0f);
Vector3 velocity(0.0f, 1.0f, 0.0f);
float dt = 0.016f;

// Update position
position += velocity * dt;

// Calculate distance from origin
float distance = position.magnitude();

// Get normalized direction
Vector3 direction = position.normalized();

// Check if vectors are perpendicular
Vector3 up(0.0f, 1.0f, 0.0f);
float dotProduct = velocity.dot(up);
bool isPerpendicular = (dotProduct == 0.0f);

// Find vector perpendicular to both
Vector3 perpendicular = position.cross(velocity);

// Physics: Apply force
Vector3 force(0.0f, -9.81f, 0.0f);  // Gravity
Vector3 acceleration = force * (1.0f / mass);
velocity += acceleration * dt;
```

## Implementation Notes

- All methods are defined inline in the header for optimal performance
- `[[nodiscard]]` attributes remind users to use return values
- Zero vector checks prevent division by zero in normalization
- `const` methods guarantee they don't modify the vector
