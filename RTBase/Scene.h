#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"

class Camera
{
public:
	Matrix projectionMatrix;
	Matrix inverseProjectionMatrix;
	Matrix camera;
	float width = 0;
	float height = 0;
	Vec3 origin;
	Vec3 viewDirection;
	float Alens;
	float Wlens = 1.0f;  // 添加镜头宽度成员变量，默认值为1.0f
	void init(Matrix ProjectionMatrix, int screenwidth, int screenheight)
	{
		projectionMatrix = ProjectionMatrix;
		inverseProjectionMatrix = ProjectionMatrix.invert();
		width = (float)screenwidth;
		height = (float)screenheight;
		float aspect = ProjectionMatrix.a[0][0] / ProjectionMatrix.a[1][1];
		float Hlens = Wlens * aspect;
		Alens = Wlens * Hlens;
	}
	void updateView(Matrix V)
	{
		camera = V;
		origin = camera.mulPoint(Vec3(0, 0, 0));
		viewDirection = inverseProjectionMatrix.mulPointAndPerspectiveDivide(Vec3(0, 0, 1));
		viewDirection = camera.mulVec(viewDirection);
		viewDirection = viewDirection.normalize();
	}
	// Add code here
	Ray generateRay(float x, float y)
	{
		//float ndcX = (2.0f * x) / width - 1.0f;
		//float ndcY = 1.0f - (2.0f * y) / height; // 反转 Y 轴，符合 NDC 规范

		//// 2. 在相机空间构造齐次坐标 (NDC 变换后坐标，Z 设为 -1, W 设为 1)
		//Vec3 pointNDC(ndcX, ndcY, -1.0f); // 远平面 Z = -1
		//Vec3 pointCamera = inverseProjectionMatrix.mulPointAndPerspectiveDivide(pointNDC);

		//// 3. 变换到世界坐标 (V^-1 变换)
		//Vec3 pointWorld = camera.mulPoint(pointCamera);

		//// 4. 计算光线方向
		//Vec3 dir = (pointWorld - origin).normalize();

		//return Ray(origin, dir);
		float xprime = x / width;
		float yprime = 1.0f - (y / height);
		xprime = (xprime * 2.0f) - 1.0f;
		yprime = (yprime * 2.0f) - 1.0f;
		Vec3 dir(xprime, yprime, 1.0f);
		dir = inverseProjectionMatrix.mulPoint(dir);
		dir = camera.mulVec(dir);
		dir = dir.normalize();
		return Ray(origin, dir);
	}
};

class Scene
{
public:
	std::vector<Triangle> triangles;
	std::vector<BSDF*> materials;
	std::vector<Light*> lights;
	Light* background = NULL;
	BVHNode* bvh = NULL;
	Camera camera;
	AABB bounds;
	void build()
	{
		//// 构建BVH
		//if (bvh) {
		//	delete bvh;  // 如果已存在，先删除
		//}
		//
		//// 创建三角形索引数组
		//std::vector<Triangle> sortedTriangles = triangles;  // 复制三角形数组
		//bvh = new BVHNode();
		//bvh->build(sortedTriangles);  // 传入可排序的三角形数组
		//
		//// 更新原始三角形数组
		//triangles = sortedTriangles;
		//
		std::vector<Triangle> inputTriangles;
		for (int i = 0; i < triangles.size(); i++)
		{
			inputTriangles.push_back(triangles[i]);
		}
		triangles.clear();
		bvh = new BVHNode();
		bvh->build(inputTriangles, triangles);

		//// Do not touch the code below this line!
		//// Build light list
		for (int i = 0; i < triangles.size(); i++)
		{
			if (materials[triangles[i].materialIndex]->isLight())
			{
				AreaLight* light = new AreaLight();
				light->triangle = &triangles[i];
				light->emission = materials[triangles[i].materialIndex]->emission;
				lights.push_back(light);
			}
		}
	}
	IntersectionData traverse(const Ray& ray)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		////for (int i = 0; i < triangles.size(); i++) {
		////	float t, u, v;
		////	if (triangles[i].rayIntersect(ray, t, u, v)) {
		////		if (t > 0 && t < intersection.t) {
		////			intersection.t = t;
		////			intersection.ID = i;
		////			intersection.alpha = u;
		////			intersection.beta = v;
		////			intersection.gamma = 1.0f - (u + v);
		////		}
		////	}
		////}
		// //使用BVH进行射线相交测试
		//if (bvh != NULL) {
		//	intersection = bvh->traverse(ray, triangles);
		//} else {
		//	// 如果BVH未构建，使用暴力遍历
		//	for (int i = 0; i < triangles.size(); i++) {
		//		float t, u, v;
		//		if (triangles[i].rayIntersect(ray, t, u, v)) {
		//			if (t > 0 && t < intersection.t) {
		//				intersection.t = t;
		//				intersection.ID = i;
		//				intersection.alpha = u;
		//				intersection.beta = v;
		//				intersection.gamma = 1.0f - (u + v);
		//			}
		//		}
		//	}
		//}

		//// 确保相交数据有效
		//if (intersection.t == FLT_MAX) {
		//	intersection.ID = -1;  // 标记为未相交
		//}
		//
		//return intersection;
		return bvh->traverse(ray, triangles);
	}
	Light* sampleLight(Sampler* sampler, float& pmf)
	{
		float r1 = sampler->next();
		pmf = 1.0f / (float)lights.size();
		return lights[std::min((int)(r1 * lights.size()), (int)(lights.size() - 1))];
		//return NULL;
	}
	// Do not modify any code below this line
	void init(std::vector<Triangle> meshTriangles, std::vector<BSDF*> meshMaterials, Light* _background)
	{
		for (int i = 0; i < meshTriangles.size(); i++)
		{
			triangles.push_back(meshTriangles[i]);
			bounds.extend(meshTriangles[i].vertices[0].p);
			bounds.extend(meshTriangles[i].vertices[1].p);
			bounds.extend(meshTriangles[i].vertices[2].p);
		}
		for (int i = 0; i < meshMaterials.size(); i++)
		{
			materials.push_back(meshMaterials[i]);
		}
		background = _background;
		if (background->totalIntegratedPower() > 0)
		{
			lights.push_back(background);
		}
	}
	bool visible(const Vec3& p1, const Vec3& p2)
	{
		Ray ray;
		Vec3 dir = p2 - p1;
		float maxT = dir.length() - (2.0f * EPSILON);
		dir = dir.normalize();
		ray.init(p1 + (dir * EPSILON), dir);
		return bvh->traverseVisible(ray, triangles, maxT);
	}
	Colour emit(Triangle* light, ShadingData shadingData, Vec3 wi)
	{
		return materials[light->materialIndex]->emit(shadingData, wi);
	}
	ShadingData calculateShadingData(IntersectionData intersection, Ray& ray)
	{
		ShadingData shadingData = {};
		if (intersection.t < FLT_MAX)
		{
			shadingData.x = ray.at(intersection.t);
			shadingData.gNormal = triangles[intersection.ID].gNormal();
			triangles[intersection.ID].interpolateAttributes(intersection.alpha, intersection.beta, intersection.gamma, shadingData.sNormal, shadingData.tu, shadingData.tv);
			shadingData.bsdf = materials[triangles[intersection.ID].materialIndex];
			shadingData.wo = -ray.dir;
			if (shadingData.bsdf->isTwoSided())
			{
				if (Dot(shadingData.wo, shadingData.sNormal) < 0)
				{
					shadingData.sNormal = -shadingData.sNormal;
				}
				if (Dot(shadingData.wo, shadingData.gNormal) < 0)
				{
					shadingData.gNormal = -shadingData.gNormal;
				}
			}
			shadingData.frame.fromVector(shadingData.sNormal);
			shadingData.t = intersection.t;
		} else
		{
			shadingData.wo = -ray.dir;
			shadingData.t = intersection.t;
		}
		return shadingData;
	}
};