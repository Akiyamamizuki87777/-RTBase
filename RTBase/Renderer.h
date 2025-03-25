#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include<mutex>
#include<queue>
class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	bool canHitLight = true;  // 添加canHitLight变量
	static const int MAX_DEPTH = 8;  // 添加最大深度常量
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		
		// 初始化采样器数组
		samplers = new MTRandom[numProcs];
		for(unsigned int i = 0; i < numProcs; i++) {
			samplers[i] = MTRandom((unsigned int)(i + 1));  // 使用i+1作为种子以避免0
		}
		
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool isPureSpecular = false)
	{
		//Colour pathThroughput(1.0f, 1.0f, 1.0f);
		// 检查递归深度
		if (depth > MAX_DEPTH) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		// 如果没有击中任何物体，返回背景颜色
		if (shadingData.t == FLT_MAX) {
			return scene->background->evaluate(shadingData, r.dir);
		}

		// 如果击中光源
		if (shadingData.bsdf->isLight()) {
			if (depth == 0 || isPureSpecular) {
				return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return Colour(0.0f, 0.0f, 0.0f);
		}

		// 计算直接光照
		Colour directLight = computeDirect(shadingData, sampler);

		// 俄罗斯轮盘赌
		float continueProbability = min(max(pathThroughput.r, max(pathThroughput.g, pathThroughput.b)), 0.95f);
		if (sampler->next() > continueProbability) {
			return pathThroughput * directLight;
		}

		// 采样BSDF
		float pdf;
		Colour bsdfValue;
		Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfValue, pdf);

		if (pdf <= 0.0f || bsdfValue.Lum() <= 0.0f) {
			return pathThroughput * directLight;
		}

		// 更新路径吞吐量
		pathThroughput = pathThroughput * bsdfValue * fabsf(Dot(wi, shadingData.sNormal)) / (pdf * continueProbability);

		// 生成新的光线
		Ray newRay;
		newRay.init(shadingData.x + wi * EPSILON, wi);

		// 递归追踪
		return pathThroughput * directLight + pathTrace(newRay, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular());
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, shadingData.sNormal);
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	void render()
	{
		film->incrementSPP();
		const int tileSize = 16;
		int numTilesX = (film->width + tileSize - 1) / tileSize;
		int numTilesY = (film->height + tileSize - 1) / tileSize;

		std::vector<std::thread> workers;
		std::mutex queueMutex;
		std::queue<std::pair<int, int>> tileQueue;

		// 初始化瓦片队列
		for (int ty = 0; ty < numTilesY; ty++) {
			for (int tx = 0; tx < numTilesX; tx++) {
				tileQueue.push({ tx, ty });
			}
		}

		// 启动工作线程
		for (int i = 0; i < numProcs; i++) {
			workers.emplace_back([this, i, &queueMutex, &tileQueue]() {
				MTRandom localSampler(i + 1);  // 每个线程使用自己的采样器
				while (true) {
					std::pair<int, int> tile;
					{
						std::lock_guard<std::mutex> lock(queueMutex);
						if (tileQueue.empty()) return;
						tile = tileQueue.front();
						tileQueue.pop();
					}

					int startX = tile.first * tileSize;
					int startY = tile.second * tileSize;
					int endX = min(startX + tileSize, (int)film->width);
					int endY = min(startY + tileSize, (int)film->height);

					for (int y = startY; y < endY; y++) {
						for (int x = startX; x < endX; x++) {
							float px = x + 0.5f;  // 随机偏移采样
							float py = y + 0.5f;
							Ray ray = scene->camera.generateRay(px, py);
							Colour temp(1.f, 1.f, 1.f);
							// col = direct(ray,&localSampler);
							Colour col = pathTrace(ray, temp, 0, &localSampler);
							//Colour col = viewNormals(ray);
							//Colour col = albedo(ray);
							film->splat(px, py, col);
							unsigned char r, g, b;
							film->tonemap(x, y, r, g, b);
							canvas->draw(x, y, r, g, b);
						}
					}
				}
			});
		}

		// 等待所有线程完成
		for (auto& worker : workers) {
			worker.join();
		}
	}

	//	void render()
	//{
	//	film->incrementSPP();
	//	for (unsigned int y = 0; y < film->height; y++)
	//	{
	//		for (unsigned int x = 0; x < film->width; x++)
	//		{//这里就是光线追踪（遍历每一个像素点，生成光线，返回一个颜色绘制）
	//		    //这里取每个像素点的中心来生成光线
	//			float px = x + 0.5f;
	//			float py = y + 0.5f;
	//			Ray ray = scene->camera.generateRay(px, py);
	//			//Colour col = viewNormals(ray);
	//			//Colour col = albedo(ray);
	//			Colour temp(1.f, 1.f, 1.f);
	//			Colour col = pathTrace(ray, temp, 0, samplers);
	//			film->splat(px, py, col);
	//			//把法线的颜色变成255版本
	//			unsigned char r = (unsigned char)(col.r * 255);
	//			unsigned char g = (unsigned char)(col.g * 255);
	//			unsigned char b = (unsigned char)(col.b * 255);
	//			film->tonemap(x, y, r, g, b);
	//			canvas->draw(x, y, r, g, b);
	//		}
	//	}
	//}

	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};