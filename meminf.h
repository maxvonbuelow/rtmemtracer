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

#include <cstdint>

extern "C" {

enum MeminfDesc {
	MI_FRAMEBUF, MI_BVH, MI_AABBS, MI_VTX_ATTRIB, MI_FACES, MI_VTX_POS, MI_SPHERES, _MI_MAX
};
enum MeminfArg {
	MI_ARG_CAMERA, _MI_ARG_MAX
};

void meminf_describe(void *ptr, enum MeminfDesc desc, int pitch = 1);
void meminf_arg(enum MeminfArg type, const char *data, int size);

}
