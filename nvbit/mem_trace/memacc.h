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

#include "meminf_storage.h"

struct __attribute__((packed)) QAcc {
	uint32_t off;
	float rdg, rdl;
	uint32_t denominator_g, denominator_l;
	uint8_t flags;
	inline int getdesc() const
	{
		return (flags >> 1) & 0x1f;
	}
	inline void setdesc(int d)
	{
		flags = ((int)d & 0x1f) << 1;
	}

	inline QAcc(MeminfDesc d, uint32_t off, float rdg, float rdl, uint32_t de_l, uint32_t de_g) : off(off), flags(((int)d & 0x1f) << 1 | 0), rdg(rdg), rdl(rdl), denominator_g(de_g), denominator_l(de_l)
	{}
	inline QAcc()
	{}
	inline void merge(const QAcc &other)
	{
		rdg += other.rdg;
		rdl += other.rdl;

		denominator_g += other.denominator_g;
		denominator_l += other.denominator_l;
	}

	inline void set_global(float sumhit, uint32_t denom)
	{
		rdg = sumhit;
		denominator_g = denom;
	}
	inline void set_local(float sumhit, uint32_t denom)
	{
		rdl = sumhit;
		denominator_l = denom;
	}
};

struct ExtAcc : QAcc {
	uint8_t sm;
	uint32_t cta;

	inline ExtAcc(MeminfDesc d, uint32_t off, float rdg, float rdl, uint32_t denomg, uint32_t denoml, uint8_t sm, uint32_t cta) : QAcc(d, off, rdg, rdl, denomg, denoml), sm(sm), cta(cta)
	{}
}; 
