/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * chroma_cells.frag - Colour of each 2x2 cell of a packed raw Bayer image
 *
 * Renders one texel per Bayer cell: the red, mean green and blue pixels after
 * black level, lens shading and white balance correction, square root
 * encoded so that 8 bits keep the precision of dark colours. The debayering
 * shader samples it with bilinear filtering to smooth the chroma noise.
 */

#ifdef GL_ES
precision highp float;
#endif

#if defined(RAW10P)
#define BPP		1.25
#elif defined(RAW12P)
#define BPP		1.5
#else
#error Invalid raw format
#endif

varying vec2 textureOut;

uniform vec2 tex_size;
uniform vec2 tex_step;
uniform vec2 tex_bayer_first_red;
uniform sampler2D tex_y;
uniform vec3 awb;
uniform vec3 blacklevel;

#if defined(APPLY_LSC)
uniform sampler2D lsc_tex;
#endif

float fetch(vec2 p)
{
	return texture2D(tex_y, vec2(floor(BPP * p.x + 0.02), p.y) * tex_step).r;
}

void main(void)
{
	/* The red pixel of the cell, at even distance from the first red */
	vec2 o = 2.0 * floor(textureOut * tex_size / 2.0) +
		 mod(tex_bayer_first_red, 2.0);
	o = min(o, tex_size - vec2(2.0));

	vec3 rgb = vec3(fetch(o),
			0.5 * (fetch(o + vec2(1.0, 0.0)) + fetch(o + vec2(0.0, 1.0))),
			fetch(o + vec2(1.0, 1.0)));

	rgb = (rgb - blacklevel) / (1.0 - blacklevel);
#if defined(APPLY_LSC)
	rgb = rgb * (texture2D(lsc_tex, textureOut).rgb * 4.0);
#endif
	rgb = clamp(rgb * awb, vec3(0.0), vec3(1.0));

	gl_FragColor = vec4(sqrt(rgb), 1.0);
}
