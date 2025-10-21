//
// Created by penguin on 10/20/25.
//

#ifndef CPHYSICSENGINE_UNIFORMGRID_H
#define CPHYSICSENGINE_UNIFORMGRID_H
#pragma once
#include "../Dynamics/RigidBody.h"
#include "../Core/Vector3.h"

#include <vector>
#include <unordered_map>
#include <set>

struct Vector3i {
	int x, y, z;

	bool operator==(const Vector3i& other) const {
		return x == other.x and y == other.y and z == other.z;
	}
};

struct Vector3iHash {
	size_t operator()(const Vector3i& v) const {
		return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1) ^ (std::hash<int>()(v.z) << 2);
	}
};

class UniformGrid {
	float cellSize;
	std::unordered_map<Vector3i, std::vector<RigidBody*>, Vector3iHash> grid;
	std::set<std::pair<RigidBody*, RigidBody*>> checkedPairs;

public:
	UniformGrid(float cellSize) : cellSize(cellSize) {}

	Vector3i getCellIndex(const Vector3& point) const {
		return {
			static_cast<int>(std::floor(point.x / cellSize)),
			static_cast<int>(std::floor(point.y / cellSize)),
			static_cast<int>(std::floor(point.z / cellSize))
		};
	}

	void clear() {
		grid.clear();
		checkedPairs.clear();
	}

	void addBody(RigidBody* body) {
		Vector3i minIndex = getCellIndex(body->worldAABBMin);
		Vector3i maxIndex = getCellIndex(body->worldAABBMax);

		// NOTE: This could create a problem at some point
		for (int ix = minIndex.x; ix <= maxIndex.x; ix++) {
			for (int iy = minIndex.y; iy <= maxIndex.y; iy++) {
				for (int iz = minIndex.z; iz <= maxIndex.z; iz++) {
					Vector3i cellIndex = {ix, iy, iz};
					grid[cellIndex].push_back(body);
				}
			}
		}
	}

	std::vector<std::pair<RigidBody*, RigidBody*>> getPotentialCollisions() {
		std::vector<std::pair<RigidBody*, RigidBody*>> potentialPairs;
		checkedPairs.clear();

		for (const auto pair : grid) {
			const Vector3i& cellIndex = pair.first;
			const std::vector<RigidBody*>& cellBodies = pair.second;

			for (int i = 0; i < cellBodies.size(); i++) {
				for (int j = i + 1; j < cellBodies.size(); j++) {
					addPair(cellBodies[i], cellBodies[j], potentialPairs);
				}
			}

			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					for (int dz = -1; dz <= 1; dz++) {
						if (dx == 0 and dy == 0 and dz == 0) {
							continue;
						}

						Vector3i neighbourIndex = {cellIndex.x + dx, cellIndex.y + dy, cellIndex.z + dz};

						if (auto it = grid.find(neighbourIndex); it != grid.end()) {
							const std::vector<RigidBody*>& neighbourBodies = it->second;
							for (RigidBody* body1 : cellBodies) {
								for (RigidBody* body2 : neighbourBodies) {
									if (body2 > body1) {
										addPair(body1, body2, potentialPairs);
									}
								}
							}
						}
					}
				}
			}
		}
		return potentialPairs;
	}

private:
	void addPair(RigidBody* bodyA, RigidBody* bodyB, std::vector<std::pair<RigidBody*, RigidBody*>>& pairs) {
		if (bodyA > bodyB) {
			std::swap(bodyA, bodyB);
		}

		if (checkedPairs.insert({bodyA, bodyB}).second) {
			pairs.push_back({bodyA, bodyB});
		}
	}
};


#endif //CPHYSICSENGINE_UNIFORMGRID_H
