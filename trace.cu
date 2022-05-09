/*
 * Copyright (C) 2022, Max von Buelow
 * TU Darmstadt - Interactive Graphics Systems Group
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <limits>
#include <float.h>
#include <iostream>
#include <fstream>
#include "bvh.h"
#include "reader/ply.h"
#include "image.h"
#include "mymesh.h"
#include <stdio.h>
#include <cstring>
#include <chrono>
#include "meminf.h"
#include "vec.h"
#include "occupancy.h"
#include <cuda_runtime.h>

enum Approach { WHILEWHILE, IFIF };

enum MeshOpt { KEEP, SHUFFLE, BFS };
enum Scheduling { SSM, SNORMAL };

#define __HD__ __host__ __device__
#define __D__ __device__

struct RayG {
	float org[3];
	float dir[3];
};

template <typename I>
__HD__ bool tri_intersect(I &t, I &uu, I &vv, const float *rayorg, const float *raydir, const float *v0, const float *v1, const float *v2)
{
	// moeller trumbore algorithm
// 	const I EPSILON = 0.000001;
	const I EPSILON = DBL_EPSILON;

	I e1x = v1[0] - v0[0], e1y = v1[1] - v0[1], e1z = v1[2] - v0[2];
	I e2x = v2[0] - v0[0], e2y = v2[1] - v0[1], e2z = v2[2] - v0[2];

	I hx = raydir[1] * e2z - raydir[2] * e2y;
	I hy = raydir[2] * e2x - raydir[0] * e2z;
	I hz = raydir[0] * e2y - raydir[1] * e2x;

	I a = e1x * hx + e1y * hy + e1z * hz;
// #define CULL
#ifdef CULL
	if (a < EPSILON) return false; // This ray is parallel to this triangle.
	I sx = rayorg[0] - v0[0], sy = rayorg[1] - v0[1], sz = rayorg[2] - v0[2];

	I u = sx * hx + sy * hy + sz * hz;
	if (u < 0 || u > a) return false;

	I qx = sy * e1z - sz * e1y;
	I qy = sz * e1x - sx * e1z;
	I qz = sx * e1y - sy * e1x;

	I v = raydir[0] * qx + raydir[1] * qy + raydir[2] * qz;
	if (v < 0.0 || u + v > a) return false;

	I tt = e2x * qx + e2y * qy + e2z * qz;
	I f = 1.0 / a;

	u /= f;
	v /= f;
	tt /= f;

	uu = u;
	vv = v;
	if (tt > EPSILON) {
		t = tt;
		return true; // ray intersection
	}
	return false;
#else
	if (a > -0 && a < EPSILON) return false; // This ray is parallel to this triangle.

	I f = 1.0 / a;
	I sx = rayorg[0] - v0[0], sy = rayorg[1] - v0[1], sz = rayorg[2] - v0[2];
	I u = f * (sx * hx + sy * hy + sz * hz);
	if (u < 0.0 || u > 1.0) return false;

	I qx = sy * e1z - sz * e1y;
	I qy = sz * e1x - sx * e1z;
	I qz = sx * e1y - sy * e1x;

	I v = f * (raydir[0] * qx + raydir[1] * qy + raydir[2] * qz);
	if (v < 0.0 || u + v > 1.0) return false;
	uu = u;
	vv = v;

	// At this stage we can compute t to find out where the intersection point is on the line.
	I tt = f * (e2x * qx + e2y * qy + e2z * qz);
#endif
	if (tt > EPSILON) {
		t = tt;
		return true; // ray intersection
	}
	return false; // This means that there is a line intersection but not a ray intersection.
}

struct __attribute__((packed)) camera {
	float x, y, z;
	float mat[9];
	float fov;
	uint32_t w, h;
};

struct FaceG {
	uint32_t idx[3];
};
struct Vtx {
	float v[3];
	__HD__ Vtx()
	{}
	__HD__ Vtx(const Vtx &vtx) : v{ vtx.v[0], vtx.v[1], vtx.v[2] }
	{}
	__HD__ Vtx(float a, float b, float c) : v{ a, b, c }
	{}
};
struct VtxExtra {
	float v[3];
	__HD__ VtxExtra()
	{}
	__HD__ VtxExtra(const VtxExtra &vtx) : v{ vtx.v[0], vtx.v[1], vtx.v[2] }
	{}
	__HD__ VtxExtra(float a, float b, float c) : v{ a, b, c }
	{}
};


__D__ void g_mkray(float *rayorg, float *raydir, int x, int y, int w, int h, float ox, float oy, float oz, const float *M, float fov = 3)
{
	float a = w / h; // assuming width > height 
	float Px = (2 * ((x + 0.5) / w) - 1) * tan(fov / 2 * M_PI / 180) * a;
	float Py = (1 - 2 * ((y + 0.5) / h)) * tan(fov / 2 * M_PI / 180);

	float indir[] = { Px, Py, -1 };
	for (int i = 0; i < 3; ++i) {
		float acc = 0;
		for (int j = 0; j < 3; ++j) {
			acc += M[i * 3 + j] * indir[j];
		}
		raydir[i] = acc;
	}

	rayorg[0] = ox;
	rayorg[1] = oy;
	rayorg[2] = oz;
}



template <typename T>
__D__ void swap(T &a, T &b)
{
	T t;
	t = a;
	a = b;
	b = t;
}

template <typename I>
__D__ void intersect_bounding_planes_native(I &t1, I &t2, I min, I max, int axis, const float *rayorg, const float *raydir)
{
	I dirfrac = 1.f / (axis == 0 ? raydir[0] : axis == 1 ? raydir[1] : raydir[2]);

	I ro = axis == 0 ? rayorg[0] : axis == 1 ? rayorg[1] : rayorg[2];
	t1 = (min - ro) * dirfrac;
	t2 = (max - ro) * dirfrac;
	if (t1 > t2) {
		swap(t1, t2);
	}
}

struct BoundsBVH {
	const float *bounds;
	__D__ BoundsBVH(const float *_bounds) : bounds(_bounds)
	{}

	template <typename I>
	__D__ void intersect(I &t1l, I &t2l, I &t1r, I &t2r, uint32_t idx, const float *rayorg, const float *raydir) const
	{
		t1l = FLT_MIN; t2l = FLT_MAX;
		t1r = FLT_MIN; t2r = FLT_MAX;
		I q, w, e, r;
		const float *b = bounds + idx * 12;
		for (int axis = 0; axis < 3; ++axis) {
			intersect_bounding_planes_native<I>(q, w, b[axis], b[axis + 3], axis, rayorg, raydir);
			t1l = max(t1l, q);
			t2l = min(t2l, w);
			if (t1l > t2l) break;
		}
		for (int axis = 0; axis < 3; ++axis) {
			intersect_bounding_planes_native<I>(e, r, b[axis + 6], b[axis + 9], axis, rayorg, raydir);
			t1r = max(t1r, e);
			t2r = min(t2r, r);
			if (t1r > t2r) break;
		}
	}
};

struct HitPoint {
	uint32_t idx;
	float u, v;
};
struct LeavesBVH {
	const FaceG *tris;
	const Vtx *vtx;
	int nleafesmax;
	__D__ LeavesBVH(const FaceG *_tris, const Vtx *_vtx, int nleafesmax) : tris(_tris), vtx(_vtx), nleafesmax(nleafesmax)
	{}
	__D__ uint32_t get_off(uint32_t li) const
	{
		return li * nleafesmax;
	}

	template <typename I>
	__D__ bool intersect_one(I &t, HitPoint *hitpoint, uint32_t idx, uint32_t nchilds, const float *rayorg, const float *raydir) const
	{
		FaceG f = tris[idx];
		Vtx a = vtx[f.idx[0]];
		Vtx b = vtx[f.idx[1]];
		Vtx c = vtx[f.idx[2]];

		I u, vv;
		I tt = FLT_MAX;
		tri_intersect<I>(tt, u, vv, rayorg, raydir, a.v, b.v, c.v);
		if (tt >= t) return false;

		hitpoint->idx = idx;
		hitpoint->u = u;
		hitpoint->v = vv;
		t = tt;

		return true;
	}
	template <typename I>
	__D__ void intersect(I &t, HitPoint *hitpoint, uint32_t li, uint32_t nchilds, const float *rayorg, const float *raydir) const
	{
		uint32_t off = get_off(li);
		for (int i = 0; i < nchilds; ++i) {
			intersect_one<I>(t, hitpoint, off + i, nchilds, rayorg, raydir);
		}
	}
};


template <typename I>
struct StackEntry3 {
	I t0, t1;
	uint32_t idx, leaves;
	__D__ StackEntry3()
	{}
	__D__ StackEntry3(I _t0, I _t1, uint32_t _idx, uint32_t _leaves) : t0(_t0), t1(_t1), idx(_idx), leaves(_leaves)
	{}
};


__D__ void fragment_shader(const float *vin, const float *light, float *colout, bool hit_shadow)
{
	float x = vin[0], y = vin[1], z = vin[2];
	float nx = vin[3], ny = vin[4], nz = vin[5];

	float lx = light[0] - x, ly = light[1] - y, lz = light[2] - z;
	float ll = sqrt(lx * lx + ly * ly + lz * lz);
	if (ll != 0.f) { lx /= ll; ly /= ll; lz /= ll; }

	float dot = fabs(nx * lx + ny * ly + nz * lz);
	colout[0] = min(max(1.f * dot, 0.f), 1.f) - (hit_shadow ? 0.5 : 0);
}

__device__ __forceinline__ unsigned int lane_id(void) {
    unsigned int laneid;
    asm volatile("mov.u32 %0, %laneid;" : "=r"(laneid));
    return laneid;
}
__device__ int atomicAggInc(int *ptr) {
    int mask = __match_any_sync(__activemask(), (unsigned long long)ptr);
    int leader = __ffs(mask) - 1;    // select a leader
    int res;
    if(lane_id() == leader)                  // leader does the update
        res = atomicAdd(ptr, __popc(mask));
    res = __shfl_sync(mask, res, leader);    // get leader’s old value
    return res + __popc(mask & ((1 << lane_id()) - 1)); //compute old value
}
__device__ __forceinline__ unsigned int get_smid(void) {
    unsigned int ret;
    asm("mov.u32 %0, %smid;" : "=r"(ret));
    return ret;
}
#define NSMS 68
#define BLOCKDIM_Y 2
__device__ const int B = 32 * BLOCKDIM_Y; // example batch size
__device__ int globalPoolNextRay[NSMS] = {0};
template <Approach A, Scheduling S>
__global__ void PersistentTraceKernel(uint8_t *framebuf, const uint32_t *subtrees, const float *bounds, const FaceG *faces, const Vtx *vtx, const VtxExtra *ve, uint32_t w, uint32_t h, camera cam, int nleafesmax)
{
	static const float light[] = { 50, 220, 1140 };

	BoundsBVH bo(bounds);
	LeavesBVH lv(faces, vtx, nleafesmax);
	StackEntry3<float> stack[128];

	uint32_t ni = 0, li = 0, top = 0, LCI = 0;
	float t0 = 0, t1 = FLT_MAX;
	float t = FLT_MAX;
	HitPoint hitpoint;
	float rayorg[3], raydir[3];
	int x, y;
	int smid, nsms;
	if (S == SSM) {
		smid = get_smid();
		nsms = NSMS;
	} else {
		smid = 0;
		nsms = 1;
	}

	int globalOff = smid * ((w * h) / nsms);
	int globalOffEND = (smid + 1) * ((w * h) / nsms);
	int myRayIndex = atomicAggInc(globalPoolNextRay + smid) + globalOff;
	if (myRayIndex >= globalOffEND)
		return;

	x = myRayIndex % h; 
	y = myRayIndex / h;

	g_mkray(rayorg, raydir, x, y, w, h, cam.x, cam.y, cam.z, cam.mat, cam.fov/*, 0, 200, 10000*//*, 0, 0, 4000*/);

	while (true) {
		bool hit;

		uint32_t axis, left_subtree;
		while (1) {
			uint32_t st = subtrees[ni];
			axis = st >> 30;
			left_subtree = st & 0x3fffffffu;
			if (t0 > t) goto POP;
			if (axis == 3) break;
			{
				uint32_t bi = ni - li;

				uint32_t cl = ni + 1, cr = ni + 1 + left_subtree;
				uint32_t ll = li, lr = li + (left_subtree + 1) / 2;


				// TODO check t0 and t
				float t0l = FLT_MAX, t1l = FLT_MIN, t0r = FLT_MAX, t1r = FLT_MIN;
				bo.intersect(t0l, t1l, t0r, t1r, bi, rayorg, raydir);
				t0l = max(t0l, t0);
				t1l = min(t1l, t1);
				t0r = max(t0r, t0);
				t1r = min(t1r, t1);

				if (t0l > t1l || t0l > t0r) {
					swap(t0l, t0r);
					swap(t1l, t1r);
					swap(cl, cr);
					swap(ll, lr);
				} else {
				}
				if (!(t0r > t1r) && t0r <= t) {
					StackEntry3<float> e = StackEntry3<float>(t0r, min(t1r, t), cr, lr);
					stack[top] = e;
					++top;
				} else {
				}

				if (!(t0l > t1l) && t0l <= t) {
					t0 = t0l;
	// 				t1 = t1l;
					t1 = min(t1l, t);
					ni = cl;
					li = ll;
					continue;
				}
			}
POP:
			if (top == 0) {
				hit = t != FLT_MAX;
				goto BVHTERM;
			}
			--top;
			{
				ni = stack[top].idx;
				li = stack[top].leaves;
				t0 = stack[top].t0;
				t1 = stack[top].t1;
			}
		}
		__syncwarp();
		lv.intersect(t, &hitpoint, li, left_subtree, rayorg, raydir);
		t1 = min(t1, t);
		if (top == 0) {
			hit = t != FLT_MAX;
			goto BVHTERM;
		}
		--top;
		{
			ni = stack[top].idx;
			li = stack[top].leaves;
			t0 = stack[top].t0;
			t1 = stack[top].t1;
		}

		continue;
BVHTERM:
		float res = 1;
		if (hit) {
			float u = hitpoint.u;
			float v = hitpoint.v;
			uint32_t idx = hitpoint.idx;

			FaceG f = lv.tris[idx];

			// load hit vertices completely
			Vtx v0 = lv.vtx[f.idx[0]];
			Vtx v1 = lv.vtx[f.idx[1]];
			Vtx v2 = lv.vtx[f.idx[2]];
			VtxExtra v0e = ve[f.idx[0]];
			VtxExtra v1e = ve[f.idx[1]];
			VtxExtra v2e = ve[f.idx[2]];

			// lerp
			float vertex[6];
			for (int i = 0; i < 3; ++i) {
				vertex[i] = v0.v[i] * (1.f - u - v) + v1.v[i] * u + v2.v[i] * v;
				vertex[3 + i] = v0e.v[i] * (1.f - u - v) + v1e.v[i] * u + v2e.v[i] * v;
			}
			bool hit_shadow = false;

			fragment_shader(vertex, light, &res, hit_shadow);
		}
		framebuf[y * w + x] = res * 255;
		int myRayIndex = atomicAggInc(globalPoolNextRay + smid) + globalOff;
		if (myRayIndex >= globalOffEND)
			return;
		x = myRayIndex % w; 
		y = myRayIndex / w;

		g_mkray(rayorg, raydir, x, y, w, h, cam.x, cam.y, cam.z, cam.mat, cam.fov/*, 0, 200, 10000*//*, 0, 0, 4000*/);

		t = FLT_MAX;

		ni = 0; li = 0; top = 0; LCI = 0;
		t0 = 0; t1 = FLT_MAX;
	}
}

template <Approach A, Scheduling S>
__global__ void PersistentTraceKernelIfIf(uint8_t *framebuf, const uint32_t *subtrees, const float *bounds, const FaceG *faces, const Vtx *vtx, const VtxExtra *ve, uint32_t w, uint32_t h, camera cam, int nleafesmax)
{
	static const float light[] = { 50, 220, 1140 };

	BoundsBVH bo(bounds);
	LeavesBVH lv(faces, vtx, nleafesmax);
	StackEntry3<float> stack[128];

	uint32_t ni = 0, li = 0, top = 0, LCI = 0;
	float t0 = 0, t1 = FLT_MAX;
	float t = FLT_MAX;
	HitPoint hitpoint;
	float rayorg[3], raydir[3];
	int x, y;
	int smid, nsms;
	if (S == SSM) {
		smid = get_smid();
		nsms = NSMS;
	} else {
		smid = 0;
		nsms = 1;
	}

	int globalOff = smid * ((w * h) / nsms);
	int globalOffEND = (smid + 1) * ((w * h) / nsms);
	int myRayIndex = atomicAggInc(globalPoolNextRay + smid) + globalOff;
	if (myRayIndex >= globalOffEND)
		return;

	x = myRayIndex % h; 
	y = myRayIndex / h;

	uint32_t st = subtrees[ni];
	uint32_t axis = st >> 30;
	uint32_t left_subtree = st & 0x3fffffffu;

	g_mkray(rayorg, raydir, x, y, w, h, cam.x, cam.y, cam.z, cam.mat, cam.fov/*, 0, 200, 10000*//*, 0, 0, 4000*/);
	while (true) {
		bool hit;

		if (axis != 3) {
			if (t0 > t) goto POP;
			{
				uint32_t bi = ni - li;

				uint32_t cl = ni + 1, cr = ni + 1 + left_subtree;
				uint32_t ll = li, lr = li + (left_subtree + 1) / 2;

				// TODO check t0 and t
				float t0l = FLT_MAX, t1l = FLT_MIN, t0r = FLT_MAX, t1r = FLT_MIN;
				bo.intersect(t0l, t1l, t0r, t1r, bi, rayorg, raydir);
				t0l = max(t0l, t0);
				t1l = min(t1l, t1);
				t0r = max(t0r, t0);
				t1r = min(t1r, t1);

				if (t0l > t1l || t0l > t0r) {
					swap(t0l, t0r);
					swap(t1l, t1r);
					swap(cl, cr);
					swap(ll, lr);
				} else {
				}
				if (!(t0r > t1r) && t0r <= t) {
					StackEntry3<float> e = StackEntry3<float>(t0r, min(t1r, t), cr, lr);
					stack[top] = e;
					++top;
				}

				if (!(t0l > t1l) && t0l <= t) {
					t0 = t0l;
	// 				t1 = t1l;
					t1 = min(t1l, t);
					ni = cl;
					li = ll;
					uint32_t st = subtrees[ni];
					axis = st >> 30;
					left_subtree = st & 0x3fffffffu;
				} else {
					goto POP;
				}
			}
			if (0) {
POP:
				if (top == 0) {
					hit = t != FLT_MAX;
					goto BVHTERM;
				}
				--top;
				{
					StackEntry3<float> e = stack[top];
					ni = e.idx;
					li = e.leaves;
					t0 = e.t0;
					t1 = e.t1;
				}
				{
					uint32_t st = subtrees[ni];
					axis = st >> 30;
					left_subtree = st & 0x3fffffffu;
				}
			}
		}
		__syncwarp();
		if (axis == 3) {
			uint32_t nn = left_subtree;
			uint32_t off = lv.get_off(li);
			lv.intersect_one(t, &hitpoint, off + LCI++, nn, rayorg, raydir);
			if (LCI == nn) {
				LCI = 0;
				t1 = min(t1, t);
				if (top == 0) {
					hit = t != FLT_MAX;
					goto BVHTERM;
				}
				--top;
				{
					StackEntry3<float> e = stack[top];
					ni = e.idx;
					li = e.leaves;
					t0 = e.t0;
					t1 = e.t1;
				}
				{
					uint32_t st = subtrees[ni];
					axis = st >> 30;
					left_subtree = st & 0x3fffffffu;
				}
			}
		}


		continue;
BVHTERM:
		float res = 1;
		if (hit) {
			float u = hitpoint.u;
			float v = hitpoint.v;
			uint32_t idx = hitpoint.idx;

			FaceG f = lv.tris[idx];

			// load hit vertices completely
			Vtx v0 = lv.vtx[f.idx[0]];
			Vtx v1 = lv.vtx[f.idx[1]];
			Vtx v2 = lv.vtx[f.idx[2]];
			VtxExtra v0e = ve[f.idx[0]];
			VtxExtra v1e = ve[f.idx[1]];
			VtxExtra v2e = ve[f.idx[2]];

			// lerp
			float vertex[6];
			for (int i = 0; i < 3; ++i) {
				vertex[i] = v0.v[i] * (1.f - u - v) + v1.v[i] * u + v2.v[i] * v;
				vertex[3 + i] = v0e.v[i] * (1.f - u - v) + v1e.v[i] * u + v2e.v[i] * v;
			}
			bool hit_shadow = false;

			fragment_shader(vertex, light, &res, hit_shadow);
		}
		framebuf[y * w + x] = res * 255;

		int myRayIndex = atomicAggInc(globalPoolNextRay + smid) + globalOff;
		if (myRayIndex >= globalOffEND)
			return;
		x = myRayIndex % w; 
		y = myRayIndex / w;

		g_mkray(rayorg, raydir, x, y, w, h, cam.x, cam.y, cam.z, cam.mat, cam.fov/*, 0, 200, 10000*//*, 0, 0, 4000*/);

		t = FLT_MAX;

		ni = 0; li = 0; top = 0; LCI = 0;
		t0 = 0; t1 = FLT_MAX;
		uint32_t st = subtrees[ni];
		axis = st >> 30;
		left_subtree = st & 0x3fffffffu;
	}
}


template <Approach A, Scheduling S>
void trace_gpu_sah(uint8_t *framebuf, uint32_t *subtrees, float *bounds, FaceG *faces, Vtx *vtx, VtxExtra *vtxextra, uint32_t w, uint32_t h, uint32_t maxlvl, camera cam, int nleafesmax)
{
	std::cout << "Max lvl: " << maxlvl << " " << maxlvl * sizeof(StackEntry3<float>) << std::endl;
	std::cout << "Sizes: " << sizeof(FaceG) << " " << sizeof(Vtx) << " " << sizeof(VtxExtra) << std::endl;

	cudaFuncSetCacheConfig(PersistentTraceKernel<A, S>, cudaFuncCachePreferShared);
	cudaFuncSetCacheConfig(PersistentTraceKernelIfIf<A, S>, cudaFuncCachePreferShared);

	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	dim3 blockd(32, 2);
	dim3 gridd((w + blockd.x - 1) / blockd.x, (h + blockd.y - 1) / blockd.y);
	std::cout << "Max lvl: " << maxlvl << " " << maxlvl * sizeof(StackEntry3<float>) << std::endl;

	cudaEventRecord(start);
	dim3 blockdp(32, BLOCKDIM_Y);
	int nsms = NSMS;
	auto occ = get_occupancy(75, blockdp.x * blockdp.y, 58, 0);
	int nblocks = occ.active_blocks * nsms * 1;
	std::cout << "Use blocks: " << nblocks << " Occ: " << occ.occupancy << std::endl;
	dim3 griddp(nblocks, 1);
	for (int i = 0; i < 1; ++i) {
	if (A == WHILEWHILE)
		PersistentTraceKernel<A, S><<<griddp, blockdp>>>(framebuf, subtrees, bounds, faces, vtx, vtxextra, w, h, cam, nleafesmax);
	else
		PersistentTraceKernelIfIf<A, S><<<griddp, blockdp>>>(framebuf, subtrees, bounds, faces, vtx, vtxextra, w, h, cam, nleafesmax);
	}
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	float milliseconds = 0;
	cudaEventElapsedTime(&milliseconds, start, stop);
	std::cout << "Took " << milliseconds << " ms." << std::endl;
}


void *my_malloc(std::size_t n, int pitch, MeminfDesc desc)
{
	void *p;
	cudaMalloc(&p, n * pitch);
	meminf_describe(p, desc, pitch);
	return p;
}
void my_upload(void *dst, const void *src, std::size_t s)
{
	cudaMemcpy(dst, src, s, cudaMemcpyHostToDevice);
}
void my_download(void *dst, const void *src, std::size_t s)
{
	cudaMemcpy(dst, src, s, cudaMemcpyDeviceToHost);
}


int myatoi(const std::string &s)
{
	int v = 1;
	int l = s.size();
	switch (s.back()) {
	case 'm':
	case 'M':
		v *= 1024;
	case 'k':
		v *= 1024;
		--l;
	}
	return v * std::stoi(s.substr(0, l));
}
void trace(const char *name, camera cam, Heuristic heu, MeshOpt mopt, Approach appr, Scheduling sh)
{
	std::cout << "Mesh opt method is: " << (mopt == BFS ? "bfs" : mopt == SHUFFLE ? "shuffle" : "keep") << std::endl;
	image_b test(cam.w, cam.h, 1);
	MyMesh mesh;
	std::ifstream is(name, std::ios_base::binary);
	reader::ply::read(is, mesh);
	if (mopt == SHUFFLE)
		mesh.shuffle();
	else if (mopt == BFS)
		mesh.bfs();

	mesh.compute_normals();

	std::cout << "Mesh statistics: Faces: " << mesh.faces.size() << " Vertices: " << mesh.vertices.size() << std::endl;
	static const int SIZEOF_EXPPATCH = 512;

	BVHBuilder bvhb;

	int nleafesmax = 32;

	int tag = heu * 100 + 0 * 10 + SHUFFLE /* this has no special meaning but the shuffled version currently contains the correct face order on my computer.... */;
	if (0||!bvhb.restore(name, tag)) {
		std::vector<float> aabbs(mesh.faces.size() * 6);
		std::vector<float> cens(mesh.faces.size() * 3);
		std::vector<float> vertices(mesh.faces.size() * 9);
		for (uint32_t i = 0; i < mesh.faces.size(); ++i) {
			const Face &face = mesh.faces[i];
			const Vertex &v0 = mesh.vertices[face.idx[0]];
			const Vertex &v1 = mesh.vertices[face.idx[1]];
			const Vertex &v2 = mesh.vertices[face.idx[2]];
			
			AABB aabb;
			aabb.feed(v0.pos);
			aabb.feed(v1.pos);
			aabb.feed(v2.pos);

			for (int j = 0; j < 3; ++j) {
				aabbs[i * 6 + j] = aabb.min[j];
				aabbs[i * 6 + j + 3] = aabb.max[j];
				cens[i * 3 + j] = (v0.pos[j] + v1.pos[j] + v2.pos[j]) / 3;
				vertices[i * 9 + j] = v0.pos[j];
				vertices[i * 9 + j + 3] = v1.pos[j];
				vertices[i * 9 + j + 6] = v2.pos[j];
			}
		}

		bvhb.construct(cens.data(), aabbs.data(), vertices.data(), mesh.faces.size(), nleafesmax, heu);
		bvhb.test(vertices.data(), nleafesmax);
		std::cout << "Constructed from scratch" << std::endl;

		bvhb.backup(name, tag);
	} else {
		std::cout << "Found a BVH backup!" << std::endl;
	}

	std::cout << "Got " << bvhb.num_nodes() << " nodes; bounds: " << bvhb.bounds.size() / 4 << " sum: " << bvhb.bounds.size() / 4 << std::endl;

	uint8_t *framebuf = (uint8_t*)my_malloc(test.width() * test.height(), 1, MI_FRAMEBUF);
	uint32_t *d_subtrees = (uint32_t*)my_malloc(bvhb.subtrees.size(), 4, MI_BVH);
	my_upload(d_subtrees, (const char*)bvhb.subtrees.data(), bvhb.subtrees.size() * 4);
	std::vector<Face> trispermuted(bvhb.leaf_nodes.size());
	float *d_bounds = (float*)my_malloc(bvhb.bounds.size(), 4 * 6, MI_AABBS);
	my_upload(d_bounds, (const char*)bvhb.bounds.data(), bvhb.bounds.size() * 4 * 6);

	FaceG *d_tris = (FaceG*)my_malloc(trispermuted.size(), 4 * 3, MI_FACES);
	Vtx *d_vtx = (Vtx*)my_malloc(mesh.vertices.size(), sizeof(Vtx), MI_VTX_POS);
	VtxExtra *d_vtxextra = (VtxExtra*)my_malloc(mesh.vertices.size(), sizeof(VtxExtra), MI_VTX_ATTRIB);


	std::vector<Vtx> vtx(mesh.vertices.size());
	std::vector<VtxExtra> vtxextra(mesh.vertices.size());

	std::cout << bvhb.leaf_nodes.size() << " " << mesh.faces.size() << " " << bvhb.leaf_nodes.size() * sizeof(Face) << std::endl;
	for (int i = 0; i < bvhb.leaf_nodes.size(); ++i) {
		uint32_t f = bvhb.leaf_nodes[i];
		if (f == -1u) trispermuted[i] = Face(0, 0, 0);
		else trispermuted[i] = mesh.faces[bvhb.leaf_nodes[i]];
	}
	for (int i = 0; i < mesh.vertices.size(); ++i) {
		vtx[i] = Vtx(mesh.vertices[i].pos[0], mesh.vertices[i].pos[1], mesh.vertices[i].pos[2]);
		vtxextra[i] = VtxExtra{ mesh.vertices[i].pos[3], mesh.vertices[i].pos[4], mesh.vertices[i].pos[5] };
	}

	my_upload(d_tris, (const char*)trispermuted.data(), trispermuted.size() * 4 * 3);
	my_upload(d_vtx, (const char*)vtx.data(), vtx.size() * sizeof(Vtx));
	my_upload(d_vtxextra, (const char*)vtxextra.data(), vtxextra.size() * sizeof(VtxExtra));

	std::cout << "Starting renderer" << std::endl;

	if (sh == SSM) {
		if (appr == WHILEWHILE) {
			trace_gpu_sah<WHILEWHILE, SSM>(framebuf, d_subtrees, d_bounds, d_tris, d_vtx, d_vtxextra, test.width(), test.height(), bvhb.maxlvl, cam, nleafesmax);
		} else {
			trace_gpu_sah<IFIF, SSM>(framebuf, d_subtrees, d_bounds, d_tris, d_vtx, d_vtxextra, test.width(), test.height(), bvhb.maxlvl, cam, nleafesmax);
		}
	} else {
		if (appr == WHILEWHILE) {
			trace_gpu_sah<WHILEWHILE, SNORMAL>(framebuf, d_subtrees, d_bounds, d_tris, d_vtx, d_vtxextra, test.width(), test.height(), bvhb.maxlvl, cam, nleafesmax);
		} else {
			trace_gpu_sah<IFIF, SNORMAL>(framebuf, d_subtrees, d_bounds, d_tris, d_vtx, d_vtxextra, test.width(), test.height(), bvhb.maxlvl, cam, nleafesmax);
		}
	}

	std::cout << "Download" << std::endl;
	my_download((char*)test.data(), framebuf, test.width() * test.height());

	std::cout << "Original mesh size: " << mesh.faces.size() << std::endl;
	std::cout << "Leaf triangles: " << bvhb.leaf_nodes.size() << std::endl;
	image_io::save(test, "test.png");
}

int main(int argc, const char **argv)
{
	int x = myatoi(argv[2]);
	camera cam{ std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]), {
		std::atof(argv[6]), std::atof(argv[7]), std::atof(argv[8]),
		std::atof(argv[9]), std::atof(argv[10]), std::atof(argv[11]),
		std::atof(argv[12]), std::atof(argv[13]), std::atof(argv[14])
	}, std::atof(argv[15]), x, x };

	meminf_arg(MI_ARG_CAMERA, (const char*)&cam, sizeof(camera));

	const char *name = argv[1];
	trace(name, cam, argv[16] == std::string("sah") ? SAH : MEDIAN, argv[18] == std::string("random") ? SHUFFLE : argv[18] == std::string("bfs") ? BFS : KEEP, argv[19] == std::string("whilewhile") ? WHILEWHILE : IFIF, argv[21] == std::string("sm") ? SSM : SNORMAL);
}


