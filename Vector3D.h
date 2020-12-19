#pragma once

#include <cmath>

struct Vector3D
{
	Vector3D () : x(0), y(0), z(0) {}
	Vector3D (float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

	float x;
	float y; 
	float z;

	static double Dist(const Vector3D& a, const Vector3D& b)
	{
		const float diffX = a.x - b.x;
		const float diffY = a.y - b.y;
		const float diffZ = a.z - b.z;
		return sqrt((diffX * diffX) + (diffY * diffY) + (diffZ * diffZ)); // One day cmath::sqrt will be constexpr...	
	}
};
