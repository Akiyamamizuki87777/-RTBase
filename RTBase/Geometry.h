#pragma once

#include "Core.h"
#include "Sampling.h"

Vec3 operator* (const float& num, const Vec3& vec)
{
	return Vec3(vec.x * num, vec.y * num, vec.z * num);
}

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		// 计算 n · d
		float denom = n.dot(r.dir);

		// 如果 denom 近似为 0，说明光线与平面平行，无交点
		if (fabs(denom) < 1e-6)
		{
			return false;
		}

		// 计算 t
		t = -(n.dot(r.o) + d) / denom;

		// t < 0 表示交点在光线的反方向，不可见
		return t >= 0;
	}

};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Change to Moller-Trumbore
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 E1 = vertices[1].p - vertices[0].p;
		Vec3 E2 = vertices[2].p - vertices[0].p;
		Vec3 p = r.dir.cross(E2);
		float det = E1.dot(p);
		if (fabs(det) < EPSILON) { return false; }
		float invDet = 1 / det;
		Vec3 T = r.o - vertices[0].p;
		u = T.dot(p) * invDet;
		if (u < 0 || u > 1) { return false; }
		Vec3 q = T.cross(E1);
		v = r.dir.dot(q) * invDet;
		if (v < 0 || v > 1 || (u + v) > 1) { return false; }
		t = E2.dot(q) * invDet;
		return t >= 0;
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		//计算重心
		float alpha = 1 - sqrt(r1);
		float beta = r2 * sqrt(r1);
		float gamma = 1 - (alpha + beta);

		pdf = 1 / area;
		return Vec3 (alpha* vertices[0].p +beta*vertices[1].p +gamma*vertices[2].p);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 invDir = Vec3(1.0f / r.dir.x, 1.0f / r.dir.y, 1.0f / r.dir.z);

		float tminX = (min.x - r.o.x) * invDir.x;
		float tmaxX = (max.x - r.o.x) * invDir.x;
		if (tminX > tmaxX) std::swap(tminX, tmaxX);

		float tminY = (min.y - r.o.y) * invDir.y;
		float tmaxY = (max.y - r.o.y) * invDir.y;
		if (tminY > tmaxY) std::swap(tminY, tmaxY);

		float tminZ = (min.z - r.o.z) * invDir.z;
		float tmaxZ = (max.z - r.o.z) * invDir.z;
		if (tminZ > tmaxZ) std::swap(tminZ, tmaxZ);

		float tEntry = std::max({ tminX, tminY, tminZ });
		float tExit = std::min({ tmaxX, tmaxY, tmaxZ });

		if (tEntry <= tExit && tExit >= 0) {
			t = tEntry;
			return true;
		}

		return false;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		float t;
		return rayAABB(r, t);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 OC = r.o - centre;
		float B = 2.0f * OC.dot(r.dir);
		float C = OC.dot(OC) - radius * radius;
		float delta = B * B - 4 * C;

		if (delta < 0) return false;

		float sqrtDelta = sqrt(delta);
		float t1 = (-B - sqrtDelta) * 0.5f;
		float t2 = (-B + sqrtDelta) * 0.5f;

		if (t1 > 0)
			t = t1;
		else if (t2 > 0)
			t = t2;
		else
			return false;

		return true;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	unsigned int offset;  // 三角形列表中的起始索引
	unsigned char num;    // 该节点包含的三角形数量
	bool isLeaf;         // 是否是叶子节点

	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
		isLeaf = false;
	}

	~BVHNode()
	{
		if (r) delete r;
		if (l) delete l;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& triangles)
	{
		// 如果是叶子节点
		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			isLeaf = true;
			offset = (unsigned int)triangles.size();  // 设置三角形的起始索引
			num = (unsigned char)inputTriangles.size();
			
			// 将三角形添加到全局三角形列表中
			triangles.insert(triangles.end(), inputTriangles.begin(), inputTriangles.end());
			
			// 计算包围盒
			bounds.reset();
			for (const auto& triangle : inputTriangles) {
				bounds.extend(triangle.vertices[0].p);
				bounds.extend(triangle.vertices[1].p);
				bounds.extend(triangle.vertices[2].p);
			}
			return;
		}

		// 计算整个节点的包围盒
		bounds.reset();
		for (const auto& triangle : inputTriangles) {
			bounds.extend(triangle.vertices[0].p);
			bounds.extend(triangle.vertices[1].p);
			bounds.extend(triangle.vertices[2].p);
		}

		// 计算所有三角形的中心点
		std::vector<Vec3> centers;
		centers.reserve(inputTriangles.size());
		for (const auto& triangle : inputTriangles) {
			centers.push_back(triangle.centre());
		}

		// 选择最佳分割轴
		Vec3 extent = bounds.max - bounds.min;
		int axis = 0;
		if (extent.y > extent.x && extent.y > extent.z) axis = 1;
		else if (extent.z > extent.x && extent.z > extent.y) axis = 2;

		// 计算分割点（使用中位数）
		float split = 0.0f;
		std::vector<float> centerValues(centers.size());
		for (size_t i = 0; i < centers.size(); i++) {
			centerValues[i] = (axis == 0) ? centers[i].x : ((axis == 1) ? centers[i].y : centers[i].z);
		}
		size_t mid = centerValues.size() / 2;
		std::nth_element(centerValues.begin(), centerValues.begin() + mid, centerValues.end());
		split = centerValues[mid];

		// 分割三角形
		std::vector<Triangle> leftTriangles, rightTriangles;
		for (size_t i = 0; i < inputTriangles.size(); i++) {
			float centerValue = (axis == 0) ? centers[i].x : ((axis == 1) ? centers[i].y : centers[i].z);
			if (centerValue <= split) {
				leftTriangles.push_back(inputTriangles[i]);
			} else {
				rightTriangles.push_back(inputTriangles[i]);
			}
		}

		// 处理特殊情况：如果一边为空，则平均分配
		if (leftTriangles.empty() || rightTriangles.empty()) {
			size_t mid = inputTriangles.size() / 2;
			leftTriangles.assign(inputTriangles.begin(), inputTriangles.begin() + mid);
			rightTriangles.assign(inputTriangles.begin() + mid, inputTriangles.end());
		}

		// 递归构建左右子树
		l = new BVHNode();
		r = new BVHNode();
		l->build(leftTriangles, triangles);
		r->build(rightTriangles, triangles);
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// 首先检查射线是否与当前节点的包围盒相交
		float boxT;
		if (!bounds.rayAABB(ray, boxT) || boxT > intersection.t) {
			return;
		}

		// 如果是叶子节点，测试与所有三角形的相交
		if (isLeaf) {
			for (unsigned int i = 0; i < num; i++) {
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v)) {
					if (t > 0 && t < intersection.t) {  // 确保t为正且是最近的交点
						intersection.t = t;
						intersection.ID = offset + i;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}

		// 递归遍历子节点，先遍历更近的子节点
		float tLeft = FLT_MAX, tRight = FLT_MAX;
		bool hitLeft = l ? l->bounds.rayAABB(ray, tLeft) : false;
		bool hitRight = r ? r->bounds.rayAABB(ray, tRight) : false;

		if (tLeft < tRight) {
			if (hitLeft) l->traverse(ray, triangles, intersection);
			if (hitRight && tRight < intersection.t) r->traverse(ray, triangles, intersection);
		} else {
			if (hitRight) r->traverse(ray, triangles, intersection);
			if (hitLeft && tLeft < intersection.t) l->traverse(ray, triangles, intersection);
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// 首先检查射线是否与当前节点的包围盒相交
		float t;
		if (!bounds.rayAABB(ray, t))
		{
			return true;
		}

		// 如果是叶子节点,测试与所有三角形的相交
		if (isLeaf)
		{
			for (unsigned int i = 0; i < num; i++)
			{
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v))
				{
					if (t < maxT)
					{
						return false;
					}
				}
			}
			return true;
		}

		// 如果是内部节点,递归遍历左右子树
		if (l && !l->traverseVisible(ray, triangles, maxT)) return false;
		if (r && !r->traverseVisible(ray, triangles, maxT)) return false;
		return true;
	}
};
