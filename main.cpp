#include "./src/Core/Vector3.h"
#include "./src/Dynamics/PhysicsWorld.h"
#include "./src/Dynamics/RigidBody.h"
#include <iostream>
#include <iomanip>

int main() {
	Vector3 gravity = Vector3(0, 0, 0);
	PhysicsWorld world(gravity);

	constexpr float timeStep = 1.0 / 60.0; // 60 FPS
	constexpr float simulationDuration = 5.0;

	// auto* box = new RigidBody();
	//
	// constexpr float mass = 10.0f;                       // 10 kg
	// const Vector3 dimensions(2.0f, 2.0f, 2.0f);     // A 2x2x2 cube
	// const Vector3 constantTorque(0.0f, 1.0f, 0.0f); // 1 N-m of torque around the Y-axis
	//
	// box->position = Vector3(0, 0, 0);
	// box->velocity = Vector3(0, 0, 0);
	// box->inverseMass = 1.0f / mass;
	// box->inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, dimensions);
	//
	// world.addBody(box);
	//
	// float totalTime = 0.0;
	//
	// std::cout << std::fixed << std::setprecision(4);
	// std::cout << "--- Starting Rotational Dynamics Test ---" << std::endl;
	// std::cout << "Applying constant torque: (0.0000, 1.0000, 0.0000)" << std::endl;
	// std::cout << "Body Mass: " << mass << "kg, Dimensions: (2x2x2)" << std::endl;
	// std::cout << "------------------------------------------" << std::endl;
	//
	//
	// while (totalTime < simulationDuration) {
	//     Vector3 angVel = box->angularVelocity;
	//
	//     std::cout << "Time: " << std::setw(6) << totalTime << "s, "
	//               << "Angular Velocity: ("
	//               << std::setw(8) << angVel.x << ", "
	//               << std::setw(8) << angVel.y << ", "
	//               << std::setw(8) << angVel.z << ")" << std::endl;
	//
	//     box->addTorque(constantTorque);
	//
	//     world.step(timeStep);
	//
	//     totalTime += timeStep;
	// }
	//
	// std::cout << "Test complete" << std::endl;
	//
	// delete box;
	// box = nullptr;
	auto* plank = new RigidBody();
	constexpr float mass = 1.0f; // kg
	plank->inverseMass = 1.0 / mass;
	plank->position = Vector3(0, 0, 0);
	plank->inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, Vector3(1, 10, 0));
	world.addBody(plank);

	auto kickForce = Vector3(10.0, 0.0, 0.0);
	auto kickPoint = Vector3(0.0, 5.0, 0.0);

	float totalTime = 0.0;
	std::cout << "--- Kicking a plank off-center ---" << std::endl;
	std::cout << "Applying force (10,0,0) at point (0,5,0)" << std::endl;

	while (totalTime < simulationDuration) {
		if (totalTime == 0.0f) {
			plank->addForceAtPoint(kickForce, kickPoint);
		}

		std::cout << "Time: " << std::fixed << std::setprecision(4) << totalTime
			<< " | Pos: (" << plank->position.x << ", " << plank->position.y << ")"
			<< " | AngVel.z: " << plank->angularVelocity.z << std::endl;

		world.step(timeStep);
		totalTime += timeStep;
	}

	delete plank;
	return 0;
}
