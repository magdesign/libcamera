/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Based on the code from http://jgt.akpeters.com/papers/McGuire08/
 *
 * Efficient, High-Quality Bayer Demosaic Filtering on GPUs
 *
 * Morgan McGuire
 *
 * This paper appears in issue Volume 13, Number 4.
 * ---------------------------------------------------------
 * Copyright (c) 2008, Morgan McGuire. All rights reserved.
 *
 *
 * Modified by Linaro Ltd for 10/12-bit packed vs 8-bit raw Bayer format,
 * and for simpler demosaic algorithm.
 * Copyright (C) 2020, Linaro
 *
 * The regular Bayer path uses the 5x5 linear filters of Malvar, He and
 * Cutler, "High-quality linear interpolation for demosaicing of
 * Bayer-patterned color images", ICASSP 2004, as the unpacked shader does.
 * Quad Bayer keeps the bilinear interpolation of 2x2 binned cells.
 *
 * bayer_1x_packed.frag - Fragment shader code for raw Bayer 10-bit and 12-bit
 * packed formats
 */

#ifdef GL_ES
precision highp float;
#endif

/*
 * BPP - bytes per pixel. The most significant bits of pixel x are stored in
 * byte floor(BPP * x), the least significant bits of a group of pixels
 * follow in an extra byte, which is skipped. E.g. for RAW10P:
 * -------------+-------------------+-------------------+--
 *  pixel No    |  0   1    2   3   |  4   5    6   7   | ...
 * -------------+-------------------+-------------------+--
 *  byte offset | 0   1   2   3   4 | 5   6   7   8   9 | ...
 * -------------+-------------------+-------------------+--
 *  BPP * x     | 0.0 1.25 2.5 3.75 | 5.0 6.25 7.5 8.75 | ...
 * -------------+-------------------+-------------------+--
 */
#if defined(RAW10P)
#define BPP		1.25
#elif defined(RAW12P)
#define BPP		1.5
#else
#error Invalid raw format
#endif


varying vec2 textureOut;

/* the texture size in pixels */
uniform vec2 tex_size;
uniform vec2 tex_step;
uniform vec2 tex_bayer_first_red;

uniform sampler2D tex_y;
uniform vec3 awb;
uniform mat3 ccm;
uniform vec3 blacklevel;
uniform float gamma;
uniform float contrastExp;
/* 0.0 = bilinear, 1.0 = Malvar-He-Cutler demosaicing */
uniform float demosaic_strength;
/* 0.0 = off, 1.0 = full chroma noise reduction */
uniform float chroma_denoise;
/* White balanced colour of each 2x2 cell, square root encoded */
uniform sampler2D chroma_tex;
uniform vec2 chroma_size;

#if defined(APPLY_LSC)
uniform sampler2D lsc_tex;
#endif

#if defined (QUAD_BAYER)
vec2 quad_cell(vec2 pixel)
{
	return clamp(floor(pixel / 2.0) * 2.0, vec2(0.0), tex_size - vec2(2.0));
}

float quad_fetch(vec2 pixel)
{
	vec2 cell = quad_cell(pixel);
	vec2 byte_pos = vec2(floor(BPP * cell.x + 0.02), cell.y) * tex_step;
	vec2 byte_pos_x1 = vec2(floor(BPP * (cell.x + 1.0) + 0.02), cell.y) * tex_step;
	vec2 byte_pos_y1 = byte_pos + vec2(0.0, tex_step.y);
	vec2 byte_pos_xy1 = vec2(byte_pos_x1.x, byte_pos_y1.y);
	return (texture2D(tex_y, byte_pos).r +
		texture2D(tex_y, byte_pos_x1).r +
		texture2D(tex_y, byte_pos_y1).r +
		texture2D(tex_y, byte_pos_xy1).r) * 0.25;
}
#else
/* Byte holding the most significant bits of pixel x in a packed row. */
float packed_byte(float x)
{
	/* Add a few mantissa LSBs to avoid float representation issues. */
	return floor(BPP * x + 0.02);
}

/*
 * Mirror coordinates at the image borders. The distance to the mirrored
 * pixel is even, so it has the same colour as the missing one.
 */
float mirror(float v, float last)
{
	v = abs(v);
	return v > last ? 2.0 * last - v : v;
}

float fetch_pixel(vec2 pixel)
{
	vec2 p = vec2(mirror(pixel.x, tex_size.x - 1.0),
		      mirror(pixel.y, tex_size.y - 1.0));
	return texture2D(tex_y, vec2(packed_byte(p.x), p.y) * tex_step).r;
}

#endif

#if !defined (QUAD_BAYER)
vec3 cell_colour(vec2 pos)
{
	vec3 c = texture2D(chroma_tex, pos / chroma_size).rgb;
	return c * c;
}

/*
 * Chroma noise reduction: replace the colour differences to green by their
 * mean over the 8x8 Bayer cells around the centre cell n. Each of the nine
 * bilinear taps, three cells apart, averages 2x2 cells, and its weight falls to zero when its
 * luminance differs from the centre by more than the noise tolerance, which
 * keeps colours from bleeding across edges. Green, which carries most of the
 * detail, is kept.
 */
vec3 denoise_chroma(vec3 rgb, vec2 n)
{
	const vec3 luma = vec3(0.25, 0.5, 0.25);
	vec3 c0 = cell_colour(n + vec2(0.5));
	float l0 = dot(c0, luma);
	float tol = 0.02 + 0.2 * sqrt(l0);
	vec2 sum = c0.rb - c0.g;
	float weights = 1.0;

	for (int j = -1; j <= 1; j++) {
		for (int i = -1; i <= 1; i++) {
			vec3 c = cell_colour(n + 3.0 * vec2(float(i), float(j)));
			float d = (dot(c, luma) - l0) / tol;
			float w = max(1.0 - d * d, 0.0);
			sum += w * (c.rb - c.g);
			weights += w;
		}
	}

	vec2 chroma = sum / weights;
	vec3 smoothed = clamp(vec3(rgb.g + chroma.x, rgb.g, rgb.g + chroma.y),
			      vec3(0.0), vec3(1.0));

	return mix(rgb, smoothed, min(chroma_denoise, 1.0));
}
#endif

float apply_contrast(float value)
{
	// Apply simple S-curve
	if (value < 0.5)
		return 0.5 * pow(value / 0.5, contrastExp);
	else
		return 1.0 - 0.5 * pow((1.0 - value) / 0.5, contrastExp);
}

void main(void)
{
	vec3 rgb;

#if defined (QUAD_BAYER)
	/*
	 * Each 2x2 cell of the quad Bayer pattern is binned into one pixel
	 * of a regular Bayer pattern with twice the pitch. Interpolate the
	 * missing colours bilinearly from the neighbouring cells, named
	 * below with C the cell being sampled:
	 *
	 *   +----+----+----+----+
	 *   | \ x|    |    |    |
	 *   |y \ | -2 |  0 | +2 |
	 *   +----+----+----+----+
	 *   | +2 | D2 | A1 | D3 |
	 *   +----+----+----+----+
	 *   |  0 | B0 |  C | B1 |
	 *   +----+----+----+----+
	 *   | -2 | D0 | A0 | D1 |
	 *   +----+----+----+----+
	 */
	vec2 center_pixel = quad_cell(floor(textureOut * tex_size));
	vec2 xcoords = center_pixel.x + vec2(-2.0, 2.0);
	vec2 ycoords = center_pixel.y + vec2(-2.0, 2.0);

	vec2 alternate = mod(center_pixel.xy / 2.0 + tex_bayer_first_red, 2.0);
	bool even_col = alternate.x < 1.0;
	bool even_row = alternate.y < 1.0;

	#define fetch(x, y) quad_fetch(vec2(x, y))
	float C = quad_fetch(center_pixel);

	/*
	 *   patterns.x = (A0 + A1) / 2.0
	 *   patterns.y = (B0 + B1) / 2.0
	 *   patterns.z = (A0 + A1 + B0 + B1) / 4.0
	 *   patterns.w = (D0 + D1 + D2 + D3) / 4.0
	 */
	vec4 patterns = vec4(
		fetch(center_pixel.x, ycoords[0]),	/* A0 */
		fetch(xcoords[0], center_pixel.y),	/* B0 */
		fetch(xcoords[0], ycoords[0]),		/* D0 */
		fetch(xcoords[1], ycoords[0]));		/* D1 */
	vec4 temp = vec4(
		fetch(center_pixel.x, ycoords[1]),	/* A1 */
		fetch(xcoords[1], center_pixel.y),	/* B1 */
		fetch(xcoords[1], ycoords[1]),		/* D3 */
		fetch(xcoords[0], ycoords[1]));		/* D2 */
	patterns = (patterns + temp) * 0.5;
	patterns.w = (patterns.z + patterns.w) * 0.5;
	patterns.z = (patterns.x + patterns.y) * 0.5;

	rgb = even_col ?
		(even_row ?
			vec3(C, patterns.zw) :
			vec3(patterns.x, C, patterns.y)) :
		(even_row ?
			vec3(patterns.y, C, patterns.x) :
			vec3(patterns.wz, C));
#else
	/*
	 * Malvar-He-Cutler demosaicing: the missing colours are interpolated
	 * bilinearly and corrected with the Laplacian of the colour sampled
	 * at the centre. This follows luminance edges much better than plain
	 * bilinear interpolation and avoids most of its colour fringes. The
	 * 5x5 neighbourhood of the centre C is
	 *
	 *            A0
	 *        D0  B0  D1
	 *    E0  F0  C   F1  E1
	 *        D2  B1  D3
	 *            A1
	 *
	 * and with each letter standing for the sum of its pixels, the four
	 * filters, in eighths, are
	 *
	 *   cross   (G at R or B):     4 C + 2 (B + F) - (A + E)
	 *   checker (B at R, R at B):  6 C + 2 D - 3/2 (A + E)
	 *   theta   (R or B at G, same colour left and right):
	 *                              5 C + 4 F - E - D + 1/2 A
	 *   phi     (R or B at G, same colour above and below):
	 *                              5 C + 4 B - A - D + 1/2 E
	 */
	vec2 center_pixel = floor(textureOut * tex_size);

	vec2 alternate = mod(center_pixel.xy + tex_bayer_first_red, 2.0);
	bool even_col = alternate.x < 1.0;
	bool even_row = alternate.y < 1.0;

	#define fetch(x, y) fetch_pixel(center_pixel + vec2(x, y))
	float C = fetch(0.0, 0.0);
	float A = fetch(0.0, -2.0) + fetch(0.0, 2.0);
	float B = fetch(0.0, -1.0) + fetch(0.0, 1.0);
	float D = fetch(-1.0, -1.0) + fetch(1.0, -1.0) +
		  fetch(-1.0, 1.0) + fetch(1.0, 1.0);
	float E = fetch(-2.0, 0.0) + fetch(2.0, 0.0);
	float F = fetch(-1.0, 0.0) + fetch(1.0, 0.0);

	/*
	 * Each filter is the bilinear interpolation plus a Laplacian
	 * correction. demosaic_strength scales the correction, which carries
	 * the recovered detail but also amplifies the sensor noise.
	 */
	float k = demosaic_strength / 8.0;
	float f_cross = (B + F) / 4.0 + k * (4.0 * C - (A + E));
	float f_checker = D / 4.0 + k * (6.0 * C - 1.5 * (A + E));
	float f_theta = F / 2.0 + k * (5.0 * C - E - D + 0.5 * A);
	float f_phi = B / 2.0 + k * (5.0 * C - A - D + 0.5 * E);

	rgb = even_col ?
		(even_row ?
			vec3(C, f_cross, f_checker) :	/* R */
			vec3(f_phi, C, f_theta)) :		/* G in a B row */
		(even_row ?
			vec3(f_theta, C, f_phi) :		/* G in an R row */
			vec3(f_checker, f_cross, C));	/* B */
#endif

	/*
	 * \todo: Black level normalising, AWB and digital gain could be
	 * reworked into a single multiplication.
	 */
	rgb = (rgb - blacklevel) / (1.0 - blacklevel);

#if defined(APPLY_LSC)
        /* Multiple by 4.0 for UQ<2, 6> -> float conversion:
         * texture2D(...) = original_float * 2^6 (quantization) * 1/2^8 (texture byte to float)
         * => original_float = texture2D(...) / 2^6 * 2^8 = texture2D(...) * 2^2
         */
        rgb = rgb * (texture2D(lsc_tex, textureOut).rgb * 4.0);
#endif

	/* Apply AWB gains, and saturate each channel at sensor range */
	rgb = clamp(rgb * awb, vec3(0.0), vec3(1.0));

#if !defined (QUAD_BAYER)
	if (chroma_denoise > 0.0)
		rgb = denoise_chroma(rgb, (center_pixel - alternate -
					   mod(tex_bayer_first_red, 2.0)) / 2.0);
#endif

	/* Colour Correction Matrix */
	rgb = ccm * rgb;

	/*
	 * Contrast
	 * contrastExp 0..2 contrast to 0..infinity; avoid actual infinity at tan(pi/2)
	 */
	rgb = clamp(rgb, 0.0, 1.0);
	rgb.r = apply_contrast(rgb.r);
	rgb.g = apply_contrast(rgb.g);
	rgb.b = apply_contrast(rgb.b);

	/* Apply gamma after colour correction */
	rgb = pow(rgb, vec3(gamma));

#if defined (SWAP_BLUE)
	gl_FragColor = vec4(rgb.bgr, 1.0);
#else
	gl_FragColor = vec4(rgb, 1.0);
#endif
}
