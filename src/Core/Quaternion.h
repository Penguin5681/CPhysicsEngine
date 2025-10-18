//
// Created by penguin on 10/18/25.
//

#ifndef CPHYSICSENGINE_QUATERNION_H
#define CPHYSICSENGINE_QUATERNION_H

#include <cmath>

class Quaternion {
public:
	float r, i, j, k;

	Quaternion(const float r, const float i, const float j, const float k) : r(r), i(i), j(j), k(k) {}

	void normalize() {
		float mag = std::sqrt(r * r + i * i + j * j + k * k);

		if (mag > 0.0f) {
			r /= mag;
			i /= mag;
			j /= mag;
			k /= mag;
		}
		else {
			r = 0.0f;
			i = 0.0f;
			j = 0.0f;
			k = 0.0f;
		}
	}

	Quaternion operator*(const Quaternion& other) const {
		return Quaternion(
			r * other.r - i * other.i - j * other.j - k * other.k,
			r * other.i + i * other.r + j * other.k - k * other.j,
			r * other.j - i * other.k + j * other.r + k * other.i,
			r * other.k + i * other.j - j * other.i + k * other.r
		);
	}

	Quaternion operator*=(const Quaternion& other) {
		const float old_r = r;
		const float old_i = i;
		const float old_j = j;
		const float old_k = k;

		r = old_r * other.r - old_i * other.i - old_j * other.j - old_k * other.k;
		i = old_r * other.i + old_i * other.r + old_j * other.k - old_k * other.j;
		j = old_r * other.j - old_i * other.k + old_j * other.r + old_k * other.i;
		k = old_r * other.k + old_i * other.j - old_j * other.i + old_k * other.r;

		return *this;
	}

	Quaternion& operator+=(const Quaternion& other) {
		r += other.r;
		i += other.i;
		j += other.j;
		k += other.k;
		return *this;
	}

	[[nodiscard]] Quaternion operator*(const float scalar) const {
		return Quaternion(r * scalar, i * scalar, j * scalar, k * scalar);
	}

	void updateByAngularVelocity(const Vector3& angularVelocity, const float dt) {
		const Quaternion spin(0.0f,
													angularVelocity.x,
													angularVelocity.y,
													angularVelocity.z);

		Quaternion q_dot = (spin * (*this)) * 0.5f;

		*this += q_dot * dt;
	}
};

#endif //CPHYSICSENGINE_QUATERNION_H
