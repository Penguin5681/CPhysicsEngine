# Vector3 Class Documentation

The `Vector3` class represents a three-dimensional vector with x, y, and z components. This class provides essential vector operations used for physics simulations and 3D mathematics.

## Overview

A 3D vector can represent positions, directions, velocities, forces, and other physical quantities in three-dimensional space.

## Basic Operations

### Constructors

- Default constructor: Creates a zero vector (0, 0, 0)
- Parameterized constructor: Creates a vector with specified x, y, and z components

### Arithmetic Operations

- **Addition**: $\vec{a} + \vec{b} = (a_x + b_x, a_y + b_y, a_z + b_z)$
- **Subtraction**: $\vec{a} - \vec{b} = (a_x - b_x, a_y - b_y, a_z - b_z)$
- **Component-wise multiplication**: $\vec{a} * \vec{b} = (a_x \times b_x, a_y \times b_y, a_z \times b_z)$
- **Scalar multiplication**: $\vec{a} \times s = (a_x \times s, a_y \times s, a_z \times s)$
- **Scalar division**: $\vec{a} \div s = (a_x \div s, a_y \div s, a_z \div s)$

The class also provides in-place versions (`+=`, `-=`, `*=`, `/=`) of these operations.

## Vector Mathematics

### Magnitude

The magnitude (length) of a vector is calculated as:

$$|\vec{v}| = \sqrt{x^2 + y^2 + z^2}$$

### Normalization

A normalized vector has the same direction but unit length (magnitude of 1):

$$\hat{v} = \frac{\vec{v}}{|\vec{v}|}$$

Two methods are provided:
- `normalized()`: Returns a new normalized vector
- `normalize()`: Normalizes the vector in-place

### Dot Product

The dot product between two vectors produces a scalar value:

$$\vec{a} \cdot \vec{b} = a_x \times b_x + a_y \times b_y + a_z \times b_z$$

Properties:
- If $\vec{a} \cdot \vec{b} > 0$: The vectors form an acute angle
- If $\vec{a} \cdot \vec{b} = 0$: The vectors are perpendicular
- If $\vec{a} \cdot \vec{b} < 0$: The vectors form an obtuse angle

### Cross Product

The cross product between two vectors produces a new vector perpendicular to both:

$$\vec{a} \times \vec{b} = (a_y \times b_z - a_z \times b_y, a_z \times b_x - a_x \times b_z, a_x \times b_y - a_y \times b_x)$$

The magnitude of the cross product equals the area of the parallelogram formed by the two vectors.

## Usage Examples

```cpp
Vector3 position(1.0f, 2.0f, 3.0f);
Vector3 direction(0.0f, 1.0f, 0.0f);

// Calculate distance
float distance = position.magnitude();

// Get normalized direction
Vector3 unitDirection = position.normalized();

// Check if vectors are perpendicular
float dotProduct = position.dot(direction);
bool isPerpendicular = (dotProduct == 0.0f);

// Find vector perpendicular to both
Vector3 perpendicular = position.cross(direction);
```