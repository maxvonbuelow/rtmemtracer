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

typedef __attribute__((packed)) struct {
    uint32_t cta_id_x;
    uint32_t cta_id_y;
    uint32_t cta_id_z;
    uint32_t warp_id;
    uint32_t opcode_id;
    uint64_t addrs[32];
	uint32_t preds;
	uint32_t sm;
} mem_access_t;

struct __attribute__((packed)) memop {
	enum MemorySpace {
		LOCAL,             // local memory operation
		GENERIC,           // generic memory operation
		GLOBAL,            // global memory operation
		SHARED,            // shared memory operation
// 		CONSTANT,          // constant memory operation
		GLOBAL_TO_SHARED,  // read from global memory then write to shared memory
		SURFACE,   // surface memory operation
		TEXTURE,   // texture memory operation
	};
	enum Flags { NONCOHERENT = 1 };
	char ld, st;
	MemorySpace ms;
	uint8_t flags;
	int s; // 1 2 4 8 16
};
