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

#include <limits>
#include <algorithm>

struct AABB {
	float min[3], max[3];

// 	AABB(const AABB &other)
// 	{
// 		for (int i = 0; i < 3; ++i) {
// 			min[i] = other.min[i];
// 			max[i] = other.max[i];
// 		}
// 	}
	AABB() : min{ std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity() }, max{ -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity() }
	{}
	AABB(float *mi, float *ma)
	{
		for (int i = 0; i < 3; ++i) {
			min[i] = mi[i];
			max[i] = ma[i];
		}
	}

	bool test(const float *pos) {
		for (int i = 0; i < 3; ++i) {
			if (pos[i] < min[i] || pos[i] > max[i]) return false;
		}
		return true;
	}
	bool test_as_sphere(const float *pos)
	{
		float a = 0;
		for (int i = 0; i < 3; ++i) {
			float d = pos[i] - min[i];
			a += d * d;
		}
		return a <= max[0] * max[0]/* + std::numeric_limits<float>::epsilon()*/;
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
