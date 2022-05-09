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

#pragma once

#include "sdcm.h"
#include "memacc.h"
#include "prequant.h"
#include "writer.h"
#include "bvh_trimmer.h"



struct Alloc {
	uint64_t size;
	int order;

	int sizehash_minor;
	uint32_t nreads = 0, nwrites = 0;
	std::vector<uint32_t> memaccs;

	uint32_t sh()
	{
		return (uint32_t)size << 8 | sizehash_minor;
	}
};

uint64_t find_ptr(MeminfDesc desc)
{
	for (auto mi : meminfs) {
		if (mi.second.desc == desc) {
			return mi.first;
		}
	}
	throw std::runtime_error("Did not find ptr");
	return 0;
}

struct AABB {
	float min[3], max[3];

	AABB() : min{ std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity() }, max{ -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity() }
	{}
	AABB(float *mi, float *ma)
	{
		for (int i = 0; i < 3; ++i) {
			min[i] = mi[i];
			max[i] = ma[i];
		}
	}

	void feed(const float *pos)
	{
		feed_min(pos);
		feed_max(pos);
	}
	void feed_min(const float *pos)
	{
		for (int i = 0; i < 3; ++i) {
			min[i] = std::min(min[i], pos[i]);
		}
	}
	void feed_max(const float *pos)
	{
		for (int i = 0; i < 3; ++i) {
			max[i] = std::max(max[i], pos[i]);
		}
	}
};

inline float rescale(float v, float off, float s)
{
	return ((double)v - off - s / 2) / (s / 2);
}
inline void rescale(float *v, float *off, float s)
{
	for (int i = 0; i < 3; ++i) v[i] = rescale(v[i], off[i], s);
}

struct Scaler {
	float off[3];
	float scale;
	Scaler(const AABB &aabb)
	{
		float lenx = aabb.max[0] - (off[0] = aabb.min[0]);
		float leny = aabb.max[1] - (off[1] = aabb.min[1]);
		float lenz = aabb.max[2] - (off[2] = aabb.min[2]);
		scale = std::max(std::max(lenx, leny), lenz);
	}
	void operator()(float *v)
	{
		rescale(v, off, scale);
	}
};


void traceexport(std::ostream &os, std::map<uint64_t, Alloc> &allocs, ExtAcc *accs, uint32_t bs, int nsms)
{
	uint64_t pB = find_ptr(MI_BVH);
	uint32_t sizeB = allocs[pB].size;
	uint64_t pv = find_ptr(MI_VTX_POS);
	uint64_t pf = find_ptr(MI_FACES);
	uint64_t pa = find_ptr(MI_AABBS);
	uint32_t pitchv = meminfs[pv].pitch;
	uint32_t pitchf = meminfs[pf].pitch;
	uint32_t pitcha = meminfs[pa].pitch;
	uint32_t sizev = allocs[pv].size;
	uint32_t sizef = allocs[pf].size;
	uint32_t sizea = allocs[pa].size;
	uint32_t na_unpacked = sizea / pitcha;
	uint32_t nv = sizev / pitchv;
	uint32_t nf_unpacked = sizef / pitchf;

	{
		uint64_t pv2 = -1ull, pf2 = -1ull;
		uint64_t curf = 0;
		uint64_t imagedim = -1ull;
		uint32_t l = allocs.size();
		std::vector<uint64_t> allocvec(l);
		for (auto e : allocs) {
			allocvec[e.second.order] = e.first;
		}
		for (int i = 0; i < l; ++i) {
			uint64_t pi = allocvec[i];
// 			int fi = allocs[pi].stacc;
			if (allocs[pi].nwrites >= 16 * 16 && !allocs[pi].nreads) {
				imagedim = allocs[pi].size;
// 				std::cout << "OUTSIZE " << allocs[pi].size << std::endl;
			}

			for (int j = 0; j < l; ++j) {
				if (i == j) continue;
				uint64_t pi = allocvec[i], pj = allocvec[j];
				uint64_t fi = allocs[pi].nreads, fj = allocs[pj].nreads;
				if (fi && fj && fi % fj == 0 && fi != fj && fi + fj > curf) {
					curf = fi + fj;
					pf2 = fi < fj ? pi : pj;
					pv2 = fi < fj ? pj : pi;
// 					uint32_t facei = fi < fj ? allocs[pi].sh() : allocs[pj].sh();
// 					uint32_t vtxi = fi < fj ? allocs[pj].sh() : allocs[pi].sh();
// 					std::cout << "Face hash: " << facei << "; Vertex hash: " << vtxi << " (" << fi + fj << ")" << std::endl;
				}
			}
		}
		if (pv != pv2 || pf != pf2) {
			std::cout << pv << " ?= " << pv2 << std::endl;
			std::cout << pf << " ?= " << pf2 << std::endl;
// 			std::exit(1);
		}
	}

	std::cout << pv << " " << pf << " " << pa << std::endl;
	std::cout << sizev << " " << sizef << " " << sizea << std::endl;

	std::vector<uint8_t> vertices(sizev);
	std::vector<uint32_t> faces(sizef / 4);
	std::vector<AABB> aabb_data(sizea / sizeof(AABB));
	std::vector<uint32_t> BVH(sizeB / 4);
	cudaMemcpy(vertices.data(), (void*)pv, sizev, cudaMemcpyDeviceToHost);
	cudaMemcpy(faces.data(), (void*)pf, sizef, cudaMemcpyDeviceToHost);
	cudaMemcpy(aabb_data.data(), (void*)pa, sizea, cudaMemcpyDeviceToHost);
	cudaMemcpy(BVH.data(), (void*)pB, sizeB, cudaMemcpyDeviceToHost);

	BVHTrimmer bvhtrim;
	bvhtrim.annotate_levels(BVH.data());

	std::vector<uint32_t> facespacked;
	facespacked.reserve(faces.size());
	std::vector<uint32_t> perm, invperm(faces.size());
	perm.reserve(faces.size());
// 	uint32_t nv = 0;
	std::vector<uint32_t> invperm_aabb(na_unpacked);
	std::set<uint32_t> used_aabbs;
	for (uint32_t i = 0; i < bs; ++i) {
		int id = accs[i].getdesc();
		if (id != MI_AABBS) continue;
		used_aabbs.insert(accs[i].off);
	}

	AABB aabb;
	for (uint32_t f = 0; f < faces.size() / 3; ++f) {
		uint32_t a = faces[f * 3], b = faces[f * 3 + 1], c = faces[f * 3 + 2];
		if (a == 0 && b == 0 && c == 0) continue;
		aabb.feed((float*)(vertices.data() + a * pitchv));
		aabb.feed((float*)(vertices.data() + b * pitchv));
		aabb.feed((float*)(vertices.data() + c * pitchv));
// 		nv = std::max(nv, a + 1);
// 		nv = std::max(nv, b + 1);
// 		nv = std::max(nv, c + 1);
		invperm[f] = perm.size();
		perm.push_back(f);
		facespacked.push_back(a);
		facespacked.push_back(b);
		facespacked.push_back(c);
	}
	Scaler scaler(aabb);
	for (uint32_t v = 0; v < nv; ++v) {
		float *vv = (float*)(vertices.data() + v * pitchv);
// 		scaler(vv);
	}
	std::vector<char> camdata = meminf_args[MI_ARG_CAMERA];
	float *vc = (float*)camdata.data();
// 	scaler(vc);

	std::vector<AABB> aabb_data_used(used_aabbs.size());
	uint32_t permi = 0;
	for (uint32_t x : used_aabbs) {
		aabb_data_used[permi] = aabb_data[x];
		invperm_aabb[x] = permi++;
	}

	for (int i = 0; i < used_aabbs.size(); ++i) {
// 		scaler(aabb_data_used[i].min);
// 		scaler(aabb_data_used[i].max);
	}

	uint32_t nf = facespacked.size() / 3;

	uint32_t na = used_aabbs.size();
// 	std::cout << na_unpacked << " " << na << std::endl;
// 	std::cout << nf_unpacked << " " << nf << std::endl;
	uint32_t argpitch = camdata.size();
	std::cout << "Faces: " << nf << "; Vertices: " << nv << std::endl;
	MAWriter writer(os);
	writer.header(argpitch, pitchv, pitchf, pitcha, nv, nf, na, camdata.data());
	std::cout << ">>>>>>>>>> " << nv << " " << nf << " " << na << " " << pitchv << " " << pitchf << " " << pitcha << std::endl;
// 	std::cout << "xxx" << meminf_args[MI_ARG_CAMERA].size << std::endl;
	writer.mesh((const char*)vertices.data(), (const char*)facespacked.data(), (const char*)aabb_data_used.data());

	if (nv * pitchv != vertices.size()) { std::cerr << "vert" << std::endl; std::exit(1); }
	if (nf * pitchf != facespacked.size() * 4) { std::cerr << "face" << std::endl; std::exit(1); }
// 	if (na * pitcha != aabb_data_used.size()) { std::cerr << "aabb" << std::endl; std::exit(1); }

	static const int sizeload[] = { 0, 0, 6 * 4, 0, 12, 12, 4 * 4 };
	// MI_FRAMEBUF, MI_BVH, MI_AABBS, MI_VTX_ATTRIB, MI_FACES, MI_VTX_POS, MI_SPHERES
	uint64_t bufsizes[] = { 0, 0, na_unpacked * sizeload[MI_AABBS], 0, nf_unpacked * sizeload[MI_FACES], nv * sizeload[MI_VTX_POS], 0 };
	uint64_t pseudoseg[7] = { 0 };
	for (int i = 1; i < 7; ++i) {
		pseudoseg[i] = pseudoseg[i - 1] + (bufsizes[i] + 127) / 128 * 128;
	}

	process_memoryaccesses(accs, bs, nsms, bufsizes, pseudoseg, sizeload, faces.data());
	bvhtrim(accs, bs);
	std::cout << "Recalc idx" << std::endl;
	for (uint32_t i = 0; i < bs; ++i) {
		int id = accs[i].getdesc();
		if (bufsizes[id] == 0) continue;
		if (id == MI_FACES) accs[i].off = invperm[accs[i].off];
		else if (id == MI_AABBS) accs[i].off = invperm_aabb[accs[i].off];
	}

	uint32_t maxframes = 16;
	QChunks chunks;
	chunks.reserve(bs);
	std::cout << "Prequant" << std::endl;
	prequantize(chunks, accs, bs, maxframes);
	std::cout << "Write" << std::endl;
	writer.chunks(chunks.accs.data(), chunks.accs.size(), chunks.offs.data(), chunks.size());
}
