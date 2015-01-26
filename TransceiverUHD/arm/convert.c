/*
 * NEON type conversions
 * Copyright (C) 2012, 2013 Thomas Tsou <tom@tsou.cc>
 * Copyright (C) 2015 Ettus Research 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <malloc.h>
#include <string.h>
#include "convert.h"

void neon_convert_ps_si16_4n(short *, float *, float *, int);
void neon_convert_si16_ps_4n(float *, short *, float *, int);

/* 4*N 16-bit signed integer conversion with remainder */
static void neon_convert_si16_ps(float *restrict out,
				 short *restrict in,
				 float *restrict scale,
				 int len)
{
	int start = len / 4 * 4;

	neon_convert_si16_ps_4n(out, in, scale, len >> 2);

	for (int i = 0; i < len % 4; i++)
		out[start + i] = (float) in[start + i] * scale[0];
}

/* 4*N 16-bit signed integer conversion with remainder */
static void neon_convert_ps_si16(short *restrict out,
				 float *restrict in,
				 float *restrict scale,
				 int len)
{
	int start = len / 4 * 4;

	neon_convert_ps_si16_4n(out, in, scale, len >> 2);

	for (int i = 0; i < len % 4; i++)
		out[start + i] = (short) (in[start + i] * (*scale));
}

void convert_float_short(short *out, float *in, float scale, int len)
{
        float q[4] = { scale, scale, scale, scale };

	if (len % 4)
		neon_convert_ps_si16(out, in, q, len);
	else
		neon_convert_ps_si16_4n(out, in, q, len >> 2);
}

void convert_short_float(float *out, short *in, float scale, int len)
{
        float q[4] = { scale, scale, scale, scale };

	if (len % 4)
		neon_convert_si16_ps(out, in, q, len);
	else
		neon_convert_si16_ps_4n(out, in, q, len >> 2);
}
