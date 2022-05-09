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

#include <ostream>

#include <zlib.h>

static const int BUFSIZE = 2048;
struct MAWriter {
	std::ostream &os;
	uint32_t numvert, numface, numaabb;
	uint32_t sizevert, sizeface, sizeaabb;
	bool gzip_active = false;
	z_stream zs;
	Bytef outbuf[BUFSIZE];
	MAWriter(std::ostream &os) : os(os)
	{}
	void start_gzip()
	{
		gzip_active = true;
		zs.zalloc = Z_NULL;
		zs.zfree = Z_NULL;
		zs.opaque = Z_NULL;
		deflateInit2(&zs, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 15 | 16, 8, Z_DEFAULT_STRATEGY);
	}
	void write(const char *data, uint32_t size)
	{
		if (!gzip_active) {
			os.write(data, size);
			return;
		}
		zs.avail_in = (uInt)size;
		zs.next_in = (Bytef*)data;
		do {
			zs.avail_out = BUFSIZE;
			zs.next_out = outbuf;
			deflate(&zs, Z_NO_FLUSH);
			uint32_t have = BUFSIZE - zs.avail_out;
			os.write((const char*)outbuf, have);
		} while (zs.avail_out == 0);
	}
	~MAWriter()
	{
		do {
			zs.avail_out = BUFSIZE;
			zs.next_out = outbuf;
			deflate(&zs, Z_FINISH);
			uint32_t have = BUFSIZE - zs.avail_out;
			os.write((const char*)outbuf, have);
		} while (zs.avail_out == 0);
		deflateEnd(&zs);
	}
	void header(uint32_t sizeof_cam, uint32_t sizeof_vertex, uint32_t sizeof_face, uint32_t sizeof_aabb, uint32_t nv, uint32_t nf, uint32_t na, const char *camdata)
	{
		start_gzip();
		numvert = nv;
		numface = nf;
		numaabb = na;
		sizevert = sizeof_vertex;
		sizeface = sizeof_face;
		sizeaabb = sizeof_aabb;
		write((const char*)&sizeof_cam, 4);
		write((const char*)&nv, 4);
		write((const char*)&sizeof_vertex, 4);
		write((const char*)&nf, 4);
		write((const char*)&sizeof_face, 4);
		write((const char*)&na, 4);
		write((const char*)&sizeof_aabb, 4);
		write(camdata, sizeof_cam);
	}
	void new_header(uint32_t sizeof_cam, uint32_t sizeof_vertex, uint32_t sizeof_face, uint32_t sizeof_aabb, uint32_t nv, uint32_t nf, uint32_t na, const char *camdata)
	{
		const char *magic = "GBLA";
		uint32_t sizeof_header = sizeof_cam + 8 * 4;
		uint32_t version = 0;
		write(magic, 4);
		write((const char*)&sizeof_header, 4);
		write((const char*)&version, 4);
		numvert = nv;
		numface = nf;
		numaabb = na;
		sizevert = sizeof_vertex;
		sizeface = sizeof_face;
		sizeaabb = sizeof_aabb;
		write((const char*)&sizeof_cam, 4);
		write((const char*)&nv, 4);
		write((const char*)&sizeof_vertex, 4);
		write((const char*)&nf, 4);
		write((const char*)&sizeof_face, 4);
		write((const char*)&na, 4);
		write((const char*)&sizeof_aabb, 4);
		write(camdata, sizeof_cam);
		start_gzip();
	}

	void mesh(const char *vertices, const char *faces, const char *aabbs)
	{
		write(vertices, numvert * sizevert);
		write(faces, numface * sizeface);
		write(aabbs, numaabb * sizeaabb);
	}
	void chunks(const QAcc *accs, uint64_t na, const uint64_t *offs, uint32_t nf)
	{
		enum AccuOp { SUM = 0, PROD = 1 };
		uint32_t accuop = SUM;
		uint32_t normalize = true;
		write((const char*)&nf, 4);
		write((const char*)&na, 4);
		write((const char*)&accuop, 4);
		write((const char*)&normalize, 4);
		for (uint32_t i = 0; i < nf; ++i) {
			uint32_t n = offs[i + 1] - offs[i];
			write((const char*)&n, 4);
			write((const char*)(accs + offs[i]), n * sizeof(QAcc));
		}
	}
}; 
