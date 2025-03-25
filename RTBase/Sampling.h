#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	//static Vec3 cosineSampleHemisphere(float r1, float r2)
	//{
	//	// Add code here
	//	//return Vec3(0, 0, 1);
	//	float theta = acos(sqrt(1.0f - r1)); // 计算余弦加权分布的 θ
	//	float phi = 2.0f * M_PI * r2;        // 均匀分布的方位角 φ

	//	// 转换为直角坐标系
	//	float x = cos(phi) * sin(theta);
	//	float y = sin(phi) * sin(theta);
	//	float z = cos(theta);

	//	return Vec3(x, y, z);
	//}
	//static float cosineHemispherePDF(const Vec3 wi)
	//{
	//	// Add code here
	//	//return 1.0f;
	//	return (wi.z > 0) ? wi.z / M_PI : 0.0f;
	//}
	//static Vec3 uniformSampleSphere(float r1, float r2)
	//{
	//	// Add code here
	//	//return Vec3(0, 0, 1);
	//	// 计算极坐标参数
	//	float theta = acos(1.0f - 2.0f * r1); // 均匀分布 θ
	//	float phi = 2.0f * M_PI * r2;         // 均匀分布 φ

	//	// 转换为直角坐标
	//	float x = cos(phi) * sin(theta);
	//	float y = sin(phi) * sin(theta);
	//	float z = cos(theta);

	//	return Vec3(x, y, z);
	//}
	//static float uniformSpherePDF(const Vec3& wi)
	//{
	//	// Add code here
	//	return 1.0f / (4.0f * M_PI);
	//}
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		float theta = acosf(r1);
		float phi = 2.0f * M_PI * r2;

		return SphericalCoordinates::sphericalToWorld(theta, phi);

	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		return (wi.z > 0.0f) ? (1.0f / (2.0f * M_PI)) : 0.0f;
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float theta = acosf(sqrtf(r1));
		float phi = 2.0f * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		return (wi.z > 0.0f) ? (wi.z / M_PI) : 0.0f;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		float theta = acosf(1 - 2 * r1);
		float phi = 2 * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		return 1.0f / (4.0f * M_PI);
	}
};