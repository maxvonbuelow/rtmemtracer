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

#include "sdcm.h"
#include "rdtree.h"

#include <cmath>

double binomial(int n, int k)
{
	if (k == 0 || k == n)
		return 1;

	double res = 1;

	if (k > n - k)
		k = n - k;

	for (int i = 0; i < k; ++i) {
		res *= n - i;
		res /= i + 1;
	}

	return res;
}

double sdcm(uint64_t dist, uint32_t blocks, uint32_t assoc)
{
	double sum = 0;
	if (dist == ~(uint64_t)0) return 0.;
	for (int i = 0; i < assoc; ++i) {
		double c0 = (double)binomial(dist, i);
		double c1 = std::pow((double)assoc / blocks, (double)i);
		double c2 = std::pow((double)(blocks - assoc) / blocks, (double)(dist - i));
		sum += c0 * c1 * c2;
	}
	return sum;
}

struct RdAccu {
	uint64_t rds[128];
	int n = 0;
	void account(ReuseDistance &rdtree, uint64_t addr, int linesize, int elmsize)
	{
		uint64_t line = addr / linesize;
		int off = addr % linesize;
		while (elmsize > 0) {
			rds[n++] = rdtree(line++);
			elmsize -= linesize - off;
			off = 0;
			if (elmsize < 0) elmsize = 0;
		}
	}
	float sum_phit(uint32_t blocks, uint32_t assoc)
	{
		double res = 0.;
		for (int i = 0; i < n; ++i) {
			uint64_t dist = rds[i];
			res += sdcm(dist, blocks, assoc);
		}
		return res;
	}
	uint32_t denominator()
	{
		return n;
	}
};


void calc_rd_for_sm(int sm, const ExtAcc *accs, uint32_t n, RdAccu *accus, int linesize, const uint64_t *bufsizes, const uint64_t *pseudoseg, const int *sizeload, const uint32_t *faces)
{
	ReuseDistance reuse;
	for (uint32_t i = 0; i < n; ++i) {
		if (accs[i].sm != sm && sm != -1) continue;
		int id = accs[i].getdesc();
		if (bufsizes[id] == 0) continue;
		uint32_t orgoff = accs[i].off;
		uint64_t addr = orgoff * sizeload[id] + pseudoseg[id];
		accus[i].account(reuse, addr, linesize, sizeload[id]);

		if (id == MI_FACES) {
			for (int v = 0; v < 3; ++v) {
				uint32_t vidx = faces[orgoff * 3 + v];
				uint64_t addr_v = vidx * sizeload[MI_VTX_POS] + pseudoseg[MI_VTX_POS];

				accus[i].account(reuse, addr_v, linesize, sizeload[MI_VTX_POS]);
			}
		}
	}
}

void process_memoryaccesses(ExtAcc *accs, uint32_t n, int nsms, const uint64_t *bufsizes, const uint64_t *pseudoseg, const int *sizeload, const uint32_t *faces)
{
	ReuseDistance reuse_l2;
	std::vector<ReuseDistance> reuse_l1(nsms);

	std::vector<RdAccu> accus_l1(n);
	std::vector<RdAccu> accus_l2(n);

	std::cout << "Calc RD" << std::endl;
#pragma omp parallel for
	for (int i = -1; i < nsms; ++i) {
		if (i == -1)
			calc_rd_for_sm(-1, accs, n, accus_l2.data(), L2LINE, bufsizes, pseudoseg, sizeload, faces);
		else
			calc_rd_for_sm(i, accs, n, accus_l1.data(), L1LINE, bufsizes, pseudoseg, sizeload, faces);
	}

	std::cout << "Calc SDCM" << std::endl;
#pragma omp parallel for
	for (uint32_t i = 0; i < n; ++i) {
		accs[i].set_global(accus_l2[i].sum_phit(6 * 1024 * 1024 / L2LINE, 16), accus_l2[i].denominator());
		accs[i].set_local(accus_l1[i].sum_phit(32 * 1024 / L1LINE, 64), accus_l1[i].denominator());
	}
}
