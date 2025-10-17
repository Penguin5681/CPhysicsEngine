//
// Created by penguin on 10/17/25.
//
#pragma once

#ifndef CPHYSICSENGINE_VECTOR3_H
#define CPHYSICSENGINE_VECTOR3_H
#include <cmath>

class Vector3 {
public:
	float x, y, z;

	Vector3() : x(0), y(0), z(0) {}

	Vector3(const float x, const float y, const float z) : x(x), y(y), z(z) {}

	// standard vector operation definitions

	Vector3 operator+(const Vector3& other) const {
		return {x + other.x, y + other.y, z + other.z};
	}

	Vector3 operator*(const float scalerValue) const {
		return {x * scalerValue, y * scalerValue, z * scalerValue};
	}

	Vector3 operator-(const Vector3& other) const {
		return {x - other.x, y - other.y, z - other.z};
	}

	Vector3 operator/(const float scalerValue) const {
		return {x / scalerValue, y / scalerValue, z / scalerValue};
	}

	// In-place operation definitions

	Vector3& operator+=(const Vector3& other) {
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}

	Vector3& operator-=(const Vector3& other) {
		x -= other.x;
		y -= other.y;
		z -= other.z;
		return *this;
	}

	Vector3& operator*=(const float scaler) {
		x *= scaler;
		y *= scaler;
		z *= scaler;
		return *this;
	}

	Vector3& operator/=(const float scaler) {
		x /= scaler;
		y /= scaler;
		z /= scaler;
		return *this;
	}

	// utility methods to perform calculations

	[[nodiscard]] float magnitude() const {
		return std::sqrt((x * x) + (y * y) + (z * z));
	}

	[[nodiscard]] Vector3 normalized() const {
		float vec3Magnitude = magnitude();
		if (vec3Magnitude == 0.0f) {
			return {0.0f, 0.0f, 0.0f};
		}
		return {x / vec3Magnitude, y / vec3Magnitude, z / vec3Magnitude};
	}

	// in-place vector normalization
	void normalize() {
		float vec3Magnitude = magnitude();
		if (vec3Magnitude != 0.0f) {
			x /= vec3Magnitude;
			y /= vec3Magnitude;
			z /= vec3Magnitude;
		}
	}

	// Calculate dot product with another vector
	[[nodiscard]] float dot(const Vector3& other) const {
		return (x * other.x) + (y * other.y) + (z * other.z);
	}

	// Calculate cross product with another vector
	[[nodiscard]] Vector3 cross(const Vector3& other) const {
		return {
			(y * other.z) - (z * other.y),
			(z * other.x) - (x * other.z),
			(x * other.y) - (y * other.x)
		};
	}
};

#endif //CPHYSICSENGINE_VECTOR3_H
