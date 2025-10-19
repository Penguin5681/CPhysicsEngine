# Matrix3x3 Class Documentation

The `Matrix3x3` class represents a 3×3 matrix primarily used for rotations, transformations, and inertia tensor calculations in 3D physics simulations.

## Overview

A 3×3 matrix is a mathematical construct that can represent linear transformations, rotations, and other operations in three-dimensional space. In physics engines, these matrices are particularly important for:

1. Representing orientations of rigid bodies
2. Computing inertia tensors and their inverses
3. Transforming vectors from one coordinate space to another
4. Converting quaternion rotations to matrix form

## Structure

The matrix is stored as an array of 9 float values in row-major order:

$$
M = \begin{bmatrix} 
m_0 & m_1 & m_2 \\
m_3 & m_4 & m_5 \\
m_6 & m_7 & m_8 
\end{bmatrix}
$$

where `data[i]` corresponds to $m_i$.

## Core Operations

### Constructor

The default constructor initializes the matrix as an identity matrix:

$$
I = \begin{bmatrix} 
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1 
\end{bmatrix}
$$

**Property**: Multiplying any vector by the identity matrix returns the same vector: $I \times \vec{v} = \vec{v}$

### Vector Transformation

The `transform` method applies the matrix to a vector, performing matrix-vector multiplication:

$$\vec{v}' = M \times \vec{v}$$

In component form:

$$
\begin{bmatrix} 
x' \\
y' \\
z' 
\end{bmatrix} = 
\begin{bmatrix} 
m_0 & m_1 & m_2 \\
m_3 & m_4 & m_5 \\
m_6 & m_7 & m_8 
\end{bmatrix} \times
\begin{bmatrix} 
x \\
y \\
z 
\end{bmatrix}
$$

This expands to:
$$x' = m_0 \times x + m_1 \times y + m_2 \times z$$
$$y' = m_3 \times x + m_4 \times y + m_5 \times z$$
$$z' = m_6 \times x + m_7 \times y + m_8 \times z$$

**Use case**: Rotating vectors from local space to world space.

```cpp
Matrix3x3 rotation;
rotation.setOrientation(bodyOrientation);
Vector3 localDirection(1.0f, 0.0f, 0.0f);
Vector3 worldDirection = rotation.transform(localDirection);
```

### Transpose Transformation

The `transformTranspose` method transforms a vector by the transpose of the matrix:

$$\vec{v}' = M^T \times \vec{v}$$

The transpose of a matrix swaps rows and columns:

$$
M^T = \begin{bmatrix} 
m_0 & m_3 & m_6 \\
m_1 & m_4 & m_7 \\
m_2 & m_5 & m_8 
\end{bmatrix}
$$

This expands to:
$$x' = m_0 \times x + m_3 \times y + m_6 \times z$$
$$y' = m_1 \times x + m_4 \times y + m_7 \times z$$
$$z' = m_2 \times x + m_5 \times y + m_8 \times z$$

**Use case**: For rotation matrices, the transpose equals the inverse. This efficiently transforms vectors from world space to local space.

**Mathematical property**: For rotation matrices (orthogonal matrices), $M^T = M^{-1}$

```cpp
// Transform world position to local space
Vector3 worldPos = sphereBody->position;
Vector3 localPos = boxBody->rotationMatrix.transformTranspose(
    worldPos - boxBody->position
);
```

### Get Column

The `getColumn` method extracts a column vector from the matrix:

$$\text{getColumn}(i) = \begin{bmatrix} 
m_i \\
m_{i+3} \\
m_{i+6}
\end{bmatrix}$$

where $i \in \{0, 1, 2\}$.

**Use case**: When a matrix represents a rotation, the columns are the basis vectors (axes) of the rotated coordinate system:
- Column 0: X-axis direction in world space
- Column 1: Y-axis direction in world space
- Column 2: Z-axis direction in world space

```cpp
Vector3 xAxis = rotationMatrix.getColumn(0);
Vector3 yAxis = rotationMatrix.getColumn(1);
Vector3 zAxis = rotationMatrix.getColumn(2);
```

### Inverse Inertia Tensor for Cuboid

The `setInverseInertiaTensorCuboid` method configures the matrix to represent the inverse inertia tensor of a cuboid (rectangular prism) with given mass and dimensions.

For a cuboid with dimensions $(w, h, d)$ along the x, y, and z axes respectively, the moments of inertia about each axis are:

$$I_x = \frac{1}{12} \times m \times (h^2 + d^2)$$
$$I_y = \frac{1}{12} \times m \times (w^2 + d^2)$$
$$I_z = \frac{1}{12} \times m \times (w^2 + h^2)$$

**Physical meaning**: These values represent the object's resistance to rotation around each axis. Larger values mean more resistance.

The inverse inertia tensor for a cuboid aligned with the coordinate axes is a diagonal matrix:

$$
I^{-1} = \begin{bmatrix} 
\frac{1}{I_x} & 0 & 0 \\
0 & \frac{1}{I_y} & 0 \\
0 & 0 & \frac{1}{I_z} 
\end{bmatrix}
$$

**Safety check**: The implementation checks if $I_i > 0$ before computing $\frac{1}{I_i}$, setting it to 0 if the moment of inertia is zero or negative.

**Why inverse?** In physics simulations, we use the inverse inertia tensor directly in calculations:

$$\vec{\alpha} = I^{-1} \times \vec{\tau}$$

where $\vec{\alpha}$ is angular acceleration and $\vec{\tau}$ is torque. This avoids expensive matrix inversions during runtime.

```cpp
Matrix3x3 inverseInertia;
float mass = 10.0f;
Vector3 dimensions(2.0f, 1.0f, 3.0f);  // width, height, depth
inverseInertia.setInverseInertiaTensorCuboid(mass, dimensions);
```

### Orientation from Quaternion

The `setOrientation` method creates a rotation matrix from a quaternion. For a quaternion $q = (r, i, j, k)$ where $r$ is the real part and $(i, j, k)$ are the imaginary components, the corresponding rotation matrix is:

First, compute intermediate values:
$$xx = i^2, \quad yy = j^2, \quad zz = k^2$$
$$xy = i \times j, \quad xz = i \times k, \quad yz = j \times k$$
$$sx = r \times i, \quad sy = r \times j, \quad sz = r \times k$$

Then construct the matrix:

$$
R = \begin{bmatrix} 
1-2(yy+zz) & 2(xy-sz) & 2(xz+sy) \\
2(xy+sz) & 1-2(xx+zz) & 2(yz-sx) \\
2(xz-sy) & 2(yz+sx) & 1-2(xx+yy)
\end{bmatrix}
$$

**Important**: The quaternion must be normalized (unit quaternion) to produce a valid rotation matrix.

**Use case**: After updating a rigid body's orientation quaternion, convert it to a rotation matrix for efficient vector transformations.

```cpp
Quaternion orientation(1.0f, 0.0f, 0.0f, 0.0f);
orientation.updateByAngularVelocity(angularVelocity, dt);
orientation.normalize();

Matrix3x3 rotationMatrix;
rotationMatrix.setOrientation(orientation);
```

## Usage Examples

```cpp
// Create a matrix (initialized as identity)
Matrix3x3 transform;

// Set it as a rotation from a quaternion
Quaternion orientation(0.707f, 0.707f, 0.0f, 0.0f);  // 90° around X
transform.setOrientation(orientation);

// Transform a vector from local to world space
Vector3 localVector(0.0f, 1.0f, 0.0f);
Vector3 worldVector = transform.transform(localVector);

// Transform from world to local space
Vector3 worldPos(5.0f, 3.0f, 2.0f);
Vector3 localPos = transform.transformTranspose(worldPos);

// Create an inverse inertia tensor for a box
float mass = 10.0f;
Vector3 dimensions(2.0f, 1.0f, 3.0f);  // width, height, depth
Matrix3x3 inverseInertiaTensor;
inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, dimensions);

// Use it to calculate angular acceleration
Vector3 torque(0.0f, 5.0f, 0.0f);
Vector3 angularAcceleration = inverseInertiaTensor.transform(torque);

// Extract the coordinate axes
Vector3 rightAxis = transform.getColumn(0);
Vector3 upAxis = transform.getColumn(1);
Vector3 forwardAxis = transform.getColumn(2);
```

## Mathematical Properties

### Rotation Matrices

When representing rotations, the matrix is orthogonal, meaning:
- All columns are unit vectors: $|c_i| = 1$
- All columns are mutually perpendicular: $c_i \cdot c_j = 0$ for $i \neq j$
- The determinant is 1: $\det(R) = 1$
- The transpose equals the inverse: $R^T = R^{-1}$

### Performance Considerations

- Matrix-vector multiplication: $O(9)$ operations (9 multiplications, 6 additions)
- Transpose transformation: Same cost as regular transformation
- Column extraction: $O(1)$ - just memory access

## Implementation Notes

- The matrix uses row-major storage for cache-friendly access patterns
- All transformation methods are marked `[[nodiscard]]` to prevent unused results
- The identity matrix is the default state, making it safe to use without initialization
- Methods are defined inline for optimal performance
