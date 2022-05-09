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
#include <iostream>

struct QChunks {
	std::vector<QAcc> accs;
	std::vector<uint64_t> offs;
	QChunks() : offs(1, 0)
	{}
	void reserve(uint64_t n)
	{
		accs.reserve(n);
	}
	void operator()(const QAcc *newaccs, uint64_t n)
	{
		accs.insert(accs.end(), newaccs, newaccs + n);
		offs.push_back(accs.size());
	}
	uint32_t size()
	{
		return offs.size() - 1;
	}
};
void prequantize(QChunks &out, const ExtAcc *accs, uint32_t n, uint32_t maxframes)
{
	double framesize = (double)n / maxframes;
	for (uint32_t f = 0; f < maxframes; ++f) {
		uint32_t beg = f * framesize, end = (f + 1) * framesize;
		end = std::min(end, n);
		std::unordered_map<uint32_t, std::pair<uint64_t, QAcc>> accum[_MI_MAX];
		uint64_t seq = 0;
		for (uint32_t i = beg; i < end; ++i) {
			QAcc q = accs[i];
			uint32_t ptr = q.off;
			auto ins = accum[q.getdesc()].insert(std::make_pair(ptr, std::make_pair(0, q)));
			if (ins.second) {
				ins.first->second.first = seq++;
			} else {
				ins.first->second.second.merge(q);
			}
		}
		std::vector<QAcc> chunk(seq);
		for (int i = 0; i < _MI_MAX; ++i) {
			for (auto e : accum[i]) {
				chunk[e.second.first] = e.second.second;
			}
		}
		out(chunk.data(), chunk.size());
	}
}
