# PhysicsWorld Class Documentation

The `PhysicsWorld` class is the central orchestrator of the physics simulation. It manages all rigid bodies, detects collisions, resolves interpenetration, and applies realistic physical responses based on Newtonian mechanics.

## Overview

The `PhysicsWorld` represents the entire simulated physical environment. It:
- Maintains a list of all rigid bodies in the simulation
- Applies global forces like gravity
- Detects collisions between objects
- Resolves collisions with impulse-based physics
- Steps the simulation forward in time

## Structure

### Private Members

- `bodies` - std::vector of pointers to all `RigidBody` objects in the world
- `gravity` - Global gravity acceleration vector (typically pointing downward)
- `collisions` - std::vector of `CollisionManifold` objects detected in the current frame

### Public Interface

- `PhysicsWorld(Vector3 gravity)` - Constructor that sets the gravity vector
- `addBody(RigidBody* body)` - Adds a rigid body to the simulation
- `step(float dt)` - Advances the simulation by time step `dt`

## Physics Simulation Loop

The `step` method executes the complete physics pipeline for one time step:

### Step 1: Clear Previous Collisions
```cpp
collisions.clear();
```
Each frame starts fresh with no collision data.

### Step 2: Apply Forces and Integrate

For each body with finite mass ($m^{-1} > 0$):

#### Apply Gravity
$$\vec{F}_{gravity} = m \cdot \vec{g} = \frac{\vec{g}}{m^{-1}}$$

where:
- $\vec{g}$ is the gravity acceleration (e.g., $(0, -9.81, 0)$ m/s²)
- $m^{-1}$ is the inverse mass

#### Numerical Integration

The `integrate(dt)` method performs Euler integration:

**Linear Motion:**
$$\vec{a} = \vec{F}_{total} \cdot m^{-1}$$
$$\vec{v}_{new} = \vec{v}_{old} + \vec{a} \cdot \Delta t$$
$$\vec{p}_{new} = \vec{p}_{old} + \vec{v}_{new} \cdot \Delta t$$

**Angular Motion:**
$$\vec{\alpha} = \mathbf{I}^{-1} \vec{\tau}$$
$$\vec{\omega}_{new} = \vec{\omega}_{old} + \vec{\alpha} \cdot \Delta t$$
$$q_{new} = q_{old} + \frac{1}{2}(q_{spin} \times q_{old}) \cdot \Delta t$$

where:
- $\vec{a}$ is linear acceleration
- $\vec{v}$ is velocity
- $\vec{p}$ is position
- $\vec{\alpha}$ is angular acceleration
- $\vec{\omega}$ is angular velocity
- $q$ is orientation quaternion
- $\mathbf{I}^{-1}$ is inverse inertia tensor
- $\vec{\tau}$ is torque

### Step 3: Detect Collisions
```cpp
detectCollisions();
```

### Step 4: Resolve Collisions
```cpp
resolveCollisions();
```

## Collision Detection Algorithm

The `detectCollisions()` method performs broad-phase collision detection using bounding spheres.

### N-Squared Detection

For simplicity, we check all pairs of bodies:

```cpp
for (size_t i = 0; i < bodies.size(); i++) {
    for (size_t j = i + 1; j < bodies.size(); j++) {
        // Check collision between bodies[i] and bodies[j]
    }
}
```

**Time Complexity**: $O(n^2)$ where $n$ is the number of bodies.

**Note**: For large simulations (>100 objects), spatial partitioning (octrees, spatial hashing, or BVH) should be used to reduce this to $O(n \log n)$ or even $O(n)$.

### Sphere-Sphere Collision Test

For two spheres with centers $\vec{c}_1$, $\vec{c}_2$ and radii $r_1$, $r_2$:

1. **Calculate direction vector:**
   $$\vec{d} = \vec{c}_2 - \vec{c}_1$$

2. **Calculate distance:**
   $$d = |\vec{d}| = \sqrt{d_x^2 + d_y^2 + d_z^2}$$

3. **Calculate total radius:**
   $$r_{total} = r_1 + r_2$$

4. **Check overlap:**
   $$\text{collision} \iff d < r_{total}$$

### Computing Collision Data

When a collision is detected:

#### Contact Normal
$$\vec{n} = \begin{cases}
\frac{\vec{d}}{d} & \text{if } d > 0 \\
(0, 1, 0) & \text{if } d = 0
\end{cases}$$

The fallback $(0, 1, 0)$ handles the edge case where spheres have identical centers.

#### Penetration Depth
$$d_{pen} = r_{total} - d$$

This represents how much the spheres are overlapping.

#### Contact Point
$$\vec{p}_{contact} = \vec{c}_1 + \vec{n} \cdot \left(r_1 - \frac{d_{pen}}{2}\right)$$

This places the contact point at the midpoint of the overlapping region.

### Creating the Manifold

```cpp
collisions.push_back(CollisionManifold(body1, body2, normal, penetration, contact));
```

## Collision Resolution Algorithm

The `resolveCollisions()` method applies impulse-based physics to realistically respond to collisions.

### For Each Collision Manifold

#### 1. Extract Collision Data
```cpp
RigidBody* A = manifold.bodyA;
RigidBody* B = manifold.bodyB;
Vector3 normal = manifold.contactNormal;
Vector3 contactPoint = manifold.contactPoint;
float penetration = manifold.penetrationDepth;
```

#### 2. Calculate Lever Arms

The lever arm is the vector from the center of mass to the contact point:

$$\vec{r}_A = \vec{p}_{contact} - \vec{p}_A$$
$$\vec{r}_B = \vec{p}_{contact} - \vec{p}_B$$

**Physical Meaning**: Determines the torque created by the impulse.

#### 3. Calculate Contact Point Velocities

The velocity at a point on a rotating body includes both linear and angular components:

$$\vec{v}_{point} = \vec{v}_{linear} + \vec{\omega} \times \vec{r}$$

For both bodies:
$$\vec{v}_A = \vec{v}_{A,linear} + \vec{\omega}_A \times \vec{r}_A$$
$$\vec{v}_B = \vec{v}_{B,linear} + \vec{\omega}_B \times \vec{r}_B$$

The relative velocity at the contact point:
$$\vec{v}_{rel} = \vec{v}_B - \vec{v}_A$$

#### 4. Calculate Velocity Along Normal

$$v_n = \vec{v}_{rel} \cdot \vec{n}$$

**Early Exit**: If $v_n > 0$, the bodies are separating, so skip this collision:
```cpp
if (velAlongNormal > 0.0f) {
    continue;
}
```

#### 5. Calculate Coefficient of Restitution

$$e = \min(e_A, e_B)$$

where $e \in [0, 1]$:
- $e = 0$: Perfectly inelastic (objects stick together)
- $e = 1$: Perfectly elastic (no energy loss)
- $e = 0.7$: Typical value for moderately bouncy objects

#### 6. Calculate Impulse Magnitude (The Core Formula)

This is the most complex calculation in the entire physics engine:

$$j = \frac{-(1 + e) \cdot v_n}{m_A^{-1} + m_B^{-1} + [(\mathbf{I}_A^{-1}(\vec{r}_A \times \vec{n})) \times \vec{r}_A + (\mathbf{I}_B^{-1}(\vec{r}_B \times \vec{n})) \times \vec{r}_B] \cdot \vec{n}}$$

**Breaking it down:**

**Numerator:**
$$-(1 + e) \cdot v_n$$
This is the desired change in velocity, including restitution.

**Denominator - Linear Part:**
$$m_A^{-1} + m_B^{-1}$$
The combined "effective mass" for linear motion.

**Denominator - Rotational Part:**
$$\text{term}_A = (\mathbf{I}_A^{-1}(\vec{r}_A \times \vec{n})) \times \vec{r}_A$$
$$\text{term}_B = (\mathbf{I}_B^{-1}(\vec{r}_B \times \vec{n})) \times \vec{r}_B$$

These terms account for rotational inertia. The calculation process:
1. $\vec{r} \times \vec{n}$ gives the torque direction per unit impulse
2. $\mathbf{I}^{-1}$ converts torque to angular acceleration
3. The outer cross product $\times \vec{r}$ converts back to linear velocity change at the contact point

**Final Denominator:**
$$denominator = m_A^{-1} + m_B^{-1} + (\text{term}_A + \text{term}_B) \cdot \vec{n}$$

**Safety Check:**
```cpp
if (denominator <= 0.0f) {
    continue; // Avoid division by zero
}
```

#### 7. Apply Linear Impulse

The impulse vector:
$$\vec{J} = j \cdot \vec{n}$$

Apply to velocities:
$$\vec{v}_A \leftarrow \vec{v}_A - \vec{J} \cdot m_A^{-1}$$
$$\vec{v}_B \leftarrow \vec{v}_B + \vec{J} \cdot m_B^{-1}$$

**Physical Meaning**: Newton's third law - equal and opposite forces.

#### 8. Apply Angular Impulse (Torque)

Calculate torque impulses:
$$\vec{\tau}_A = -(\vec{r}_A \times \vec{J})$$
$$\vec{\tau}_B = \vec{r}_B \times \vec{J}$$

Apply to angular velocities:
$$\vec{\omega}_A \leftarrow \vec{\omega}_A + \mathbf{I}_A^{-1} \vec{\tau}_A$$
$$\vec{\omega}_B \leftarrow \vec{\omega}_B + \mathbf{I}_B^{-1} \vec{\tau}_B$$

**Physical Meaning**: Off-center collisions create rotation.

#### 9. Positional Correction (Baumgarte Stabilization)

To prevent objects from slowly sinking into each other due to numerical drift:

$$\vec{correction} = \vec{n} \cdot \frac{\max(d_{pen} - slop, 0)}{m_A^{-1} + m_B^{-1}} \cdot percent$$

where:
- $slop = 0.01$ - Small tolerance to allow minor penetration without correction
- $percent = 0.2$ - Correction strength (20% per frame)

Apply position corrections:
$$\vec{p}_A \leftarrow \vec{p}_A - \vec{correction} \cdot m_A^{-1}$$
$$\vec{p}_B \leftarrow \vec{p}_B + \vec{correction} \cdot m_B^{-1}$$

**Why This Works:**
- Only corrects significant penetration (> slop)
- Gradual correction (20% per frame) prevents jittering
- Mass-proportional correction is more realistic
- Prevents the "sinking" artifact in long-running simulations

## Physical Principles

### Conservation of Momentum

Linear momentum before collision:
$$\vec{p}_{before} = m_A\vec{v}_A + m_B\vec{v}_B$$

Linear momentum after collision:
$$\vec{p}_{after} = m_A\vec{v}'_A + m_B\vec{v}'_B$$

The impulse-based resolution ensures: $\vec{p}_{before} = \vec{p}_{after}$ (momentum is conserved).

### Conservation of Angular Momentum

Angular momentum is also conserved through the torque impulse calculations.

### Energy Dissipation

The coefficient of restitution $e$ controls energy loss:
- Kinetic energy after collision: $KE_{after} = e^2 \cdot KE_{before}$
- Energy lost: $E_{lost} = (1 - e^2) \cdot KE_{before}$

## Usage Example

```cpp
// Create a physics world with Earth gravity
Vector3 gravity(0.0f, -9.81f, 0.0f);
PhysicsWorld world(gravity);

// Create rigid bodies
RigidBody* ball1 = new RigidBody();
ball1->position = Vector3(0, 10, 0);
ball1->inverseMass = 1.0f; // 1 kg
ball1->shape = new BoundingSphere(0.5f);

RigidBody* ground = new RigidBody();
ground->position = Vector3(0, 0, 0);
ground->inverseMass = 0.0f; // Infinite mass (immovable)
ground->shape = new BoundingSphere(100.0f);

// Add to world
world.addBody(ball1);
world.addBody(ground);

// Simulation loop (60 FPS)
float dt = 1.0f / 60.0f;
for (int frame = 0; frame < 600; frame++) {
    world.step(dt);
    // Render or log positions
}
```

## Performance Characteristics

- **Time Complexity**: $O(n^2)$ for collision detection, $O(c)$ for resolution where $c$ is collision count
- **Memory**: $O(n)$ for bodies, temporary $O(c)$ for manifolds
- **Frame Budget**: Aim for < 2ms total physics time at 60 FPS

## Limitations and Future Improvements

### Current Limitations
1. **No spatial partitioning**: $O(n^2)$ doesn't scale beyond ~100 objects
2. **Single contact point**: Can cause instability for box-box collisions
3. **Euler integration**: Accumulates energy over time
4. **No sleeping**: Inactive objects still consume CPU
5. **No constraints**: No joints, springs, or other constraints

### Potential Improvements
1. **Broad Phase**: Implement sweep-and-prune or spatial hashing
2. **Better Integration**: Use Verlet or RK4 for stability
3. **Sleeping**: Put stationary objects to sleep
4. **Warm Starting**: Cache impulses for faster convergence
5. **Constraint Solver**: Support joints and springs
6. **Friction**: Add tangential impulse for realistic friction
7. **Continuous Collision Detection**: Prevent tunneling at high speeds

## Mathematical Derivation Notes

The impulse formula comes from solving the constraint:

$$v'_n = -e \cdot v_n$$

where $v'_n$ is the relative velocity after collision along the normal.

Combined with Newton's laws and the definition of angular velocity, this yields the denominator terms that account for both linear and rotational inertia.

The derivation involves:
1. Defining the constraint (desired separating velocity)
2. Expressing velocities in terms of impulse $j$
3. Solving for $j$ that satisfies the constraint
4. Including both linear ($m^{-1}$) and rotational ($\mathbf{I}^{-1}$) contributions

This is known as the **sequential impulse** method, popularized by Erin Catto (Box2D creator).

