# Quaternion Class Documentation

The `Quaternion` class represents rotations in 3D space using quaternion algebra. Quaternions provide a compact, numerically stable way to represent and interpolate rotations without the gimbal lock issues inherent to Euler angles.

## Overview

A quaternion is a four-dimensional number system that extends complex numbers. In 3D graphics and physics, quaternions are primarily used to represent rotations. A quaternion consists of one real (scalar) component and three imaginary (vector) components.

## Mathematical Representation

A quaternion $q$ is represented as:

$$q = r + i\mathbf{i} + j\mathbf{j} + k\mathbf{k}$$

where:
- $r$ is the real (scalar) part
- $i$, $j$, $k$ are the imaginary (vector) parts
- $\mathbf{i}$, $\mathbf{j}$, $\mathbf{k}$ are the imaginary units satisfying:

$$\mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = \mathbf{ijk} = -1$$

## Structure

The quaternion is stored as four float values:
- `r` - Real/scalar component
- `i` - First imaginary component (x-axis)
- `j` - Second imaginary component (y-axis)
- `k` - Third imaginary component (z-axis)

## Core Operations

### Constructor

```cpp
Quaternion(float r, float i, float j, float k)
```

Creates a quaternion with specified components. For a rotation quaternion representing an identity rotation (no rotation), use:

$$q_{identity} = (1, 0, 0, 0)$$

### Normalization

A unit quaternion (magnitude 1) represents a valid rotation. The magnitude of a quaternion is:

$$|q| = \sqrt{r^2 + i^2 + j^2 + k^2}$$

The normalized quaternion is:

$$\hat{q} = \frac{q}{|q|} = \left(\frac{r}{|q|}, \frac{i}{|q|}, \frac{j}{|q|}, \frac{k}{|q|}\right)$$

The `normalize()` method ensures the quaternion remains a unit quaternion, which is essential for representing rotations accurately. Small numerical errors can accumulate during calculations, so normalization should be performed regularly.

### Quaternion Multiplication

Quaternion multiplication combines two rotations. Given quaternions $q_1 = (r_1, i_1, j_1, k_1)$ and $q_2 = (r_2, i_2, j_2, k_2)$, their product is:

$$q_1 \times q_2 = \begin{pmatrix}
r_1 \cdot r_2 - i_1 \cdot i_2 - j_1 \cdot j_2 - k_1 \cdot k_2 \\
r_1 \cdot i_2 + i_1 \cdot r_2 + j_1 \cdot k_2 - k_1 \cdot j_2 \\
r_1 \cdot j_2 - i_1 \cdot k_2 + j_1 \cdot r_2 + k_1 \cdot i_2 \\
r_1 \cdot k_2 + i_1 \cdot j_2 - j_1 \cdot i_2 + k_1 \cdot r_2
\end{pmatrix}$$

**Important**: Quaternion multiplication is **not commutative**, meaning $q_1 \times q_2 \neq q_2 \times q_1$ in general. The order matters when combining rotations.

Both `operator*` (creates a new quaternion) and `operator*=` (modifies in place) are provided.

### Scalar Multiplication

Multiplying a quaternion by a scalar scales all components:

$$s \cdot q = (s \cdot r, s \cdot i, s \cdot j, s \cdot k)$$

This is used internally for integration and interpolation operations.

### Quaternion Addition

The `operator+=` adds two quaternions component-wise:

$$q_1 + q_2 = (r_1 + r_2, i_1 + i_2, j_1 + j_2, k_1 + k_2)$$

This operation is used during numerical integration of angular velocity.

## Physics Integration

### Updating Orientation from Angular Velocity

The `updateByAngularVelocity` method integrates angular velocity to update the orientation quaternion over time. This is crucial for simulating rotating rigid bodies.

Given angular velocity $\vec{\omega} = (\omega_x, \omega_y, \omega_z)$ and time step $\Delta t$, we first construct a "spin" quaternion:

$$q_{spin} = (0, \omega_x, \omega_y, \omega_z)$$

The rate of change of the orientation quaternion is:

$$\dot{q} = \frac{1}{2} q_{spin} \times q$$

Using Euler integration, we update the quaternion:

$$q_{new} = q_{old} + \dot{q} \cdot \Delta t$$

Expanded:

$$q_{new} = q_{old} + \frac{1}{2}(q_{spin} \times q_{old}) \cdot \Delta t$$

**Important**: After this integration step, the quaternion must be normalized to maintain unit length and remain a valid rotation.

### Physical Interpretation

- **Angular velocity** $\vec{\omega}$ represents how fast and around which axis an object is rotating
- The magnitude $|\vec{\omega}|$ is the rotation speed in radians per second
- The direction of $\vec{\omega}$ is the axis of rotation (right-hand rule)
- Multiplying by $\frac{1}{2}$ is mathematically required for quaternion derivative calculation

## Rotation Representation

To represent a rotation of angle $\theta$ around a unit axis $\vec{a} = (a_x, a_y, a_z)$, the quaternion is:

$$q = \left(\cos\frac{\theta}{2}, a_x\sin\frac{\theta}{2}, a_y\sin\frac{\theta}{2}, a_z\sin\frac{\theta}{2}\right)$$

Notice the half-angles: this is a fundamental property of quaternions representing rotations.

## Advantages Over Euler Angles

1. **No Gimbal Lock**: Quaternions don't suffer from the singularity that occurs when two rotation axes align
2. **Smooth Interpolation**: Spherical linear interpolation (SLERP) provides smooth rotation transitions
3. **Numerical Stability**: Less susceptible to floating-point errors than matrix representations
4. **Compact**: Only 4 values instead of 9 for a 3×3 rotation matrix
5. **Easy Composition**: Combining rotations is simple quaternion multiplication

## Usage Example

```cpp
// Create an identity quaternion (no rotation)
Quaternion orientation(1.0f, 0.0f, 0.0f, 0.0f);

// Define angular velocity (rotating around Y axis at 2 rad/s)
Vector3 angularVelocity(0.0f, 2.0f, 0.0f);

// Integration time step
float dt = 1.0f / 60.0f; // 60 FPS

// Update orientation based on angular velocity
orientation.updateByAngularVelocity(angularVelocity, dt);
orientation.normalize(); // Keep it as a unit quaternion

// Combine two rotations
Quaternion rotation1(0.707f, 0.707f, 0.0f, 0.0f); // 90° around X
Quaternion rotation2(0.707f, 0.0f, 0.707f, 0.0f); // 90° around Y
Quaternion combined = rotation1 * rotation2; // Apply rotation1, then rotation2
```

## Implementation Notes

- Always normalize after integration to prevent drift from numerical errors
- The quaternion must be a unit quaternion to represent a valid rotation
- When converting to other representations (e.g., rotation matrices), ensure normalization first
- For interpolation between orientations, use SLERP (Spherical Linear Interpolation) for smooth transitions

