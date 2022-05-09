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

#include "memacc.h"
#include "meminf.h"
#include <stack>

struct BVHTrimmer {
	struct AABBMeta { uint32_t parent, level; };
	std::vector<AABBMeta> aabb_meta;
	uint32_t maxlvl = 0;
	void annotate_levels(uint32_t *bvh)
	{
		std::cout << "Parse BVH" << std::endl;
		struct SEBVH { uint32_t ni, li, parent, level; };
		std::stack<SEBVH> stack;

		uint32_t ni = 0, li = 0, parent = -1u, level = 0;
		stack.push(SEBVH{ ni, li, parent, level });
		do {
			SEBVH e = stack.top(); stack.pop();
			ni = e.ni; li = e.li; parent = e.parent; level = e.level;
			uint32_t st = bvh[ni];
			uint32_t axis = st >> 30, left_subtree = st & 0x3fffffffu;

			if (axis != 3) {
				maxlvl = std::max(maxlvl, level);
				uint32_t bi = ni - li;

				if (aabb_meta.size() <= bi * 2 + 1)
					aabb_meta.resize(bi * 2 + 2);
				aabb_meta[bi * 2] = AABBMeta{ parent, level };
				aabb_meta[bi * 2 + 1] = AABBMeta{ parent, level };

				uint32_t cl = ni + 1, cr = ni + 1 + left_subtree;
				uint32_t ll = li, lr = li + (left_subtree + 1) / 2;

				stack.push(SEBVH{ cr, lr, bi * 2 + 1, level + 1 });
				stack.push(SEBVH{ cl, ll, bi * 2, level + 1 });
			}
		} while (!stack.empty());
		std::cout << "Parse BVH: done" << std::endl;
	}
	void operator()(ExtAcc *accs, uint32_t n)
	{
		int TRIMAABBLEVELS = maxlvl - 13;
		int trimcount = 0;
		for (int l = 0; l < TRIMAABBLEVELS; ++l) {
			trimcount = 0;
			for (uint32_t i = 0; i < n; ++i) {
				int id = accs[i].getdesc();
				if (id != MI_AABBS) continue;
				AABBMeta am = aabb_meta[accs[i].off];
				if (am.level == maxlvl) {
					accs[i].off = am.parent;
					++trimcount;
				}
			}
			--maxlvl;
		}
		std::cout << "Trim aabb: done => removed aabbs: " << trimcount << std::endl;
	}
};
