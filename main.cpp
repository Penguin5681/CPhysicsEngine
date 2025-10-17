#include "./src/Core/Vector3.h"
#include "./src/Dynamics/PhysicsWorld.h"
#include "./src/Dynamics/RigidBody.h"
#include <iostream>

int main(int argc, char* argv[]) {
	Vector3 gravity = Vector3(0, -9.81, 0);
	PhysicsWorld world(gravity);
	float timeStep = 1.0 / 60.0; // this would be 60 FPS simulation
	float simulationDuration = 5.0; // run this thing for 5 seconds

	auto* ball = new RigidBody();
	ball->position = Vector3(0, 100, 0);
	ball->velocity = Vector3(10, 0, 0);
	ball->inverseMass = 1.0 / 5.0;

	world.addBody(ball);

	float totalTime = 0.0;
	while (totalTime < simulationDuration) {
		std::cout << "Time: " << totalTime << "s, Position: (" << ball->position.x << ", " << ball->position.y << ", " <<
			ball->position.z << ")" << std::endl;

		world.step(timeStep);
		totalTime += timeStep;
	}

	delete ball;

	return 0;
}
