# Matrix3x3 Class Documentation

The `Matrix3x3` class represents a 3×3 matrix primarily used for rotations, transformations, and inertia tensor calculations in 3D physics simulations.

## Overview

A 3×3 matrix is a mathematical construct that can represent linear transformations, rotations, and other operations in three-dimensional space. In physics engines, these matrices are particularly important for:

1. Representing orientations of rigid bodies
2. Computing inertia tensors
3. Transforming vectors from one coordinate space to another

## Structure

The matrix is stored as an array of 9 float values in row-major order:

$$
\begin{bmatrix} 
m_0 & m_1 & m_2 \\
m_3 & m_4 & m_5 \\
m_6 & m_7 & m_8 
\end{bmatrix}
$$

## Core Operations

### Constructor

The default constructor initializes the matrix. In a typical implementation, this would create an identity matrix:

$$
\begin{bmatrix} 
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1 
\end{bmatrix}
$$

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

### Inertia Tensor for Cuboid

The `setInverseInertiaTensorCuboid` method configures the matrix to represent the inverse inertia tensor of a cuboid (rectangular prism) with given mass and dimensions.

For a cuboid with dimensions $w$ (width), $h$ (height), and $d$ (depth) along the x, y, and z axes respectively, the moments of inertia are:

$$I_x = \frac{1}{12} \times mass \times (h^2 + d^2)$$
$$I_y = \frac{1}{12} \times mass \times (w^2 + d^2)$$
$$I_z = \frac{1}{12} \times mass \times (w^2 + h^2)$$

The inverse inertia tensor for a cuboid aligned with the coordinate axes is a diagonal matrix:

$$
\begin{bmatrix} 
\frac{1}{I_x} & 0 & 0 \\
0 & \frac{1}{I_y} & 0 \\
0 & 0 & \frac{1}{I_z} 
\end{bmatrix}
$$

### Orientation from Quaternion

The `setOrientation` method creates a rotation matrix from a quaternion. For a quaternion $q = [s, \vec{v}] = [s, x, y, z]$ (where $s$ is the scalar component and $\vec{v}$ is the vector component), the corresponding rotation matrix is:

$$
\begin{bmatrix} 
1-2(y^2+z^2) & 2(xy-sz) & 2(xz+sy) \\
2(xy+sz) & 1-2(x^2+z^2) & 2(yz-sx) \\
2(xz-sy) & 2(yz+sx) & 1-2(x^2+y^2)
\end{bmatrix}
$$

This conversion allows us to rotate vectors using matrix multiplication after defining orientations with more intuitive quaternions.

## Usage Examples

```cpp
// Create a matrix
Matrix3x3 rotationMatrix;

// Set it as a rotation from a quaternion
Quaternion orientation;
// ... initialize orientation ...
rotationMatrix.setOrientation(orientation);

// Transform a vector
Vector3 position(1.0f, 2.0f, 3.0f);
Vector3 rotatedPosition = rotationMatrix.transform(position);

// Create an inverse inertia tensor for a cuboid
float mass = 10.0f;
Vector3 dimensions(2.0f, 1.0f, 3.0f);  // width, height, depth
Matrix3x3 inverseInertiaTensor;
inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, dimensions);
```
