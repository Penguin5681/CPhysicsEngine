#include "./src/Core/Vector3.h"
#include "./src/Core/Quaternion.h" // <-- Need this for rotation
#include "./src/Dynamics/PhysicsWorld.h"
#include "./src/Dynamics/RigidBody.h"
#include "./src/Collision/BoundingSphere.h" // <-- Need Sphere
#include "./src/Collision/BoundingBox.h"    // <-- Need Box
#include <iostream>
#include <iomanip>
#include <cmath> // For M_PI

// Define PI if not available (Windows)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    // 1. Setup Simulation with gravity
    Vector3 gravity = Vector3(0.0f, -9.81f, 0.0f);
    PhysicsWorld world(gravity);

    constexpr float timeStep = 1.0f / 60.0f;
    constexpr float simulationDuration = 6.0f;

    // 2. Create the "Ramp" (a rotated box)
    auto* rampBody = new RigidBody();
    rampBody->position = Vector3(0.0f, -5.0f, 0.0f);
    rampBody->inverseMass = 0.0f; // Immovable
    rampBody->restitution = 0.5f;

    // Create a long, flat box shape
    auto* rampShape = new BoundingBox(Vector3(20.0f, 1.0f, 20.0f));
    rampBody->shape = rampShape;
    rampBody->inverseInertiaTensor.setInverseInertiaTensorCuboid(1.0f, Vector3(40.0f, 2.0f, 40.0f));

    // --- Give the ramp its rotation ---
    // Rotate it 20 degrees "up" around the Z-axis
    float angle = 20.0f * (float)M_PI / 180.0f;
    Vector3 axis(0.0f, 0.0f, 1.0f);
    rampBody->orientation = Quaternion(cosf(angle / 2.0f),
                                       axis.x * sinf(angle / 2.0f),
                                       axis.y * sinf(angle / 2.0f),
                                       axis.z * sinf(angle / 2.0f));

    // 3. Create the "Ball"
    auto* ballBody = new RigidBody();
    ballBody->position = Vector3(-10.0f, 10.0f, 0.0f); // Start high and to the left
    ballBody->inverseMass = 1.0f; // 1kg mass
    ballBody->restitution = 0.5f;

    auto* ballShape = new BoundingSphere(1.0f); // 1m radius
    ballBody->shape = ballShape;
    // (Inertia tensor for a sphere is different, but our cuboid one is a fine approximation for now)
    ballBody->inverseInertiaTensor.setInverseInertiaTensorCuboid(1.0f, Vector3(2.0f, 2.0f, 2.0f));


    // 4. Add bodies to the world
    world.addBody(rampBody);
    world.addBody(ballBody);

    // 5. Run the Simulation Loop
    float totalTime = 0.0;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "--- Sphere-on-Box (Ramp) Test ---" << std::endl;
    std::cout << "---------------------------------" << std::endl;

    while (totalTime < simulationDuration) {
        // Print the state of the falling ball
        std::cout << "Time: " << std::setw(6) << totalTime << "s, "
                  << "Ball Position: ("
                  << std::setw(8) << ballBody->position.x << ", "
                  << std::setw(8) << ballBody->position.y << ", "
                  << std::setw(8) << ballBody->position.z << ")" << std::endl;

        world.step(timeStep);

        totalTime += timeStep;
    }

    std::cout << "--- Test Complete ---" << std::endl;

    // 6. Cleanup
    delete rampShape;
    delete ballShape;
    delete rampBody;
    delete ballBody;

    return 0;
}