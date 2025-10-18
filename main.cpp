#include "./src/Core/Vector3.h"
	#include "./src/Dynamics/PhysicsWorld.h"
	#include "./src/Dynamics/RigidBody.h"
	#include "./src/Collision/BoundingSphere.h"
	#include <iostream>
	#include <iomanip>

	int main() {
	    // Create a physics world with no gravity
	    Vector3 gravity = Vector3(0, 0, 0);
	    PhysicsWorld world(gravity);

	    constexpr float timeStep = 1.0f / 60.0f; // 60 FPS
	    constexpr float simulationDuration = 3.0f; // 3 seconds simulation

	    // Create the first ball
	    auto* ball1 = new RigidBody();
	    ball1->position = Vector3(-5.0f, 0.0f, 0.0f); // Start left of origin
	    ball1->velocity = Vector3(2.0f, 0.0f, 0.0f);  // Moving right
	    ball1->inverseMass = 1.0f; // 1kg ball

	    // Create the second ball
	    auto* ball2 = new RigidBody();
	    ball2->position = Vector3(5.0f, 0.0f, 0.0f); // Start right of origin
	    ball2->velocity = Vector3(-2.0f, 0.0f, 0.0f); // Moving left
	    ball2->inverseMass = 1.0f; // 1kg ball

	    // Create collision shapes for the balls
	    auto* sphere1 = new BoundingSphere(1.0f); // 1m radius
	    auto* sphere2 = new BoundingSphere(1.0f); // 1m radius

	    // Assign shapes to the rigid bodies
	    ball1->shape = sphere1;
	    ball2->shape = sphere2;

	    // Add bodies to the world
	    world.addBody(ball1);
	    world.addBody(ball2);

	    // Run simulation
	    float totalTime = 0.0f;
	    std::cout << "--- Collision Detection Test ---" << std::endl;
	    std::cout << "Two balls moving toward each other" << std::endl;
	    std::cout << "----------------------------------" << std::endl;

	    while (totalTime < simulationDuration) {
	        std::cout << "Time: " << std::fixed << std::setprecision(4) << totalTime
	                  << " | Ball 1 Pos: (" << ball1->position.x << ", " << ball1->position.y << ", " << ball1->position.z << ")"
	                  << " | Ball 2 Pos: (" << ball2->position.x << ", " << ball2->position.y << ", " << ball2->position.z << ")"
	                  << std::endl;

	        world.step(timeStep);
	        totalTime += timeStep;
	    }

	    std::cout << "Simulation complete" << std::endl;

	    // Cleanup
	    delete sphere1;
	    delete sphere2;
	    delete ball1;
	    delete ball2;

	    return 0;
	}