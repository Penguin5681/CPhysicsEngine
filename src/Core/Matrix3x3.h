//
// Created by penguin on 10/18/25.
//

#ifndef CPHYSICSENGINE_MATRIX3X3_H
#define CPHYSICSENGINE_MATRIX3X3_H
#include "Vector3.h"
#include "Quaternion.h"

class Matrix3x3 {
public:
	float data[9]{};

	Matrix3x3() {
		// here i am initializing this as a identity matrix
		data[0] = 1.0f;
		data[1] = 0.0f;
		data[2] = 0.0f;
		data[3] = 0.0f;
		data[4] = 1.0f;
		data[5] = 0.0f;
		data[6] = 0.0f;
		data[7] = 0.0f;
		data[8] = 1.0f;

	}

	[[nodiscard]] Vector3 transform(const Vector3& vector) const {
		return Vector3(
			data[0] * vector.x + data[1] * vector.y + data[2] * vector.z,
			data[3] * vector.x + data[4] * vector.y + data[5] * vector.z,
			data[6] * vector.x + data[7] * vector.y + data[8] * vector.z
		);
	}

	void setInverseInertiaTensorCuboid(const float mass, const Vector3 dimension) {
		const float ix = 1.0f / 12.0f * mass * (dimension.y * dimension.y + dimension.z * dimension.z);
		const float iy = 1.0f / 12.0f * mass * (dimension.x * dimension.x + dimension.z * dimension.z);
		const float iz = 1.0f / 12.0f * mass * (dimension.x * dimension.x + dimension.y * dimension.y);

		data[0] = (ix > 0.0f) ? 1.0f / ix : 0.0f;
		data[1] = 0.0f;
		data[2] = 0.0f;
		data[3] = 0.0f;
		data[4] = (iy > 0.0f) ? 1.0f / iy : 0.0f;
		data[5] = 0.0f;
		data[6] = 0.0f;
		data[7] = 0.0f;
		data[8] = (iz > 0.0f) ? 1.0f / iz : 0.0f;
	}

	void setOrientation(const Quaternion quaternion) {
		const float xx = quaternion.i * quaternion.i;
		const float yy = quaternion.j * quaternion.j;
		const float zz = quaternion.k * quaternion.k;

		const float xy = quaternion.i * quaternion.j;
		const float xz = quaternion.i * quaternion.k;
		const float yz = quaternion.j * quaternion.k;
		const float sz = quaternion.r * quaternion.k;
		const float sy = quaternion.r * quaternion.j;
		const float sx = quaternion.r * quaternion.i;

		data[0] = 1.0f - 2.0f * (yy + zz);
		data[1] = 2.0f * (xy - sz);
		data[2] = 2.0f * (xz + sy);
		data[3] = 2.0f * (xy + sz);
		data[4] = 1.0f - 2.0f * (xx + zz);
		data[5] = 2.0f * (yz - sx);
		data[6] = 2.0f * (xz - sy);
		data[7] = 2.0f * (yz + sx);
		data[8] = 1.0f - 2.0f * (xx + yy);
	}
};

#endif //CPHYSICSENGINE_MATRIX3X3_H