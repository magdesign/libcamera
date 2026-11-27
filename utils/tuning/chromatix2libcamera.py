#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""
Convert Qualcomm CamX Chromatix tuning data (com.qti.tuned.<module>.bin from
the Xiaomi Poco F1 vendor image) into a libcamera softisp tuning file.

Extracted per sensor module:
  - cc13  colour correction matrices per colour temperature, taken from the
          AEC (lux index) region that covers normal indoor light
  - lsc34 lens shading gain tables (17x13, R/Gr/Gb/B), same AEC region,
          resampled to libcamera's 17x17 grid, Gr/Gb averaged, quantized
          to the softisp UQ2.6 format (tabular LSC data is not converted
          by libcamera)
  - AWB   reference points (R/G, B/G of a neutral surface per illuminant),
          converted to libcamera colourGains (1/(R/G), 1/(B/G))

File layout (reverse engineered):
  symbol table entries of 64 bytes: { u32 0xffffffff, u32 offset, u32 size,
  u32 id, char name[48] }; offsets are relative to a data base that is found
  by locating a known structure (the cc13 colour matrix block).
  Hierarchical "trigger" nodes: { f32 start, f32 end, [f32 start2, f32 end2],
  u32 count, u32 ref } select the child data by lux index, gain, CCT; the
  child is the symbol with id ref + 1. Symbol names describe the child type
  rather than the symbol's own content, so nodes are matched by structure.

Usage: chromatix2libcamera.py tuned.bin --sensor imx363 [--awb-ct CT,...] > out.yaml
"""

import argparse
import struct
import sys

LUX_NORMAL = 250.0      # Qualcomm lux index for typical indoor light


def symbols(d):
    out = {}
    i = 0
    while True:
        j = d.find(b'\xff\xff\xff\xff', i)
        if j < 0 or j + 64 > len(d):
            break
        off, size, sid = struct.unpack_from('<III', d, j + 4)
        raw = d[j + 16:j + 64]
        name = raw.split(b'\0')[0]
        if (len(name) > 3 and all(32 <= c < 127 for c in name)
                and raw[len(name):].strip(b'\0') == b''):
            out[sid] = (off, size, name.decode())
            i = j + 64
            continue
        i = j + 4
    return out


def floats(d, o, n):
    return struct.unpack_from('<%df' % n, d, o)


def is_ccm(v):
    for r in range(3):
        row = v[r * 3:r * 3 + 3]
        if not (0.9 < sum(row) < 1.1 and 0.9 < row[r] < 4.0):
            return False
    return True


def find_base(d, syms):
    """Data base: align a cc13_cct_data symbol with a block of CCM regions
    (56 bytes each: f32 cct_start, cct_end, ccm[9], offsets[3])."""
    for o in range(0, len(d) - 64, 4):
        v = floats(d, o + 8, 9)
        if not is_ccm(v):
            continue
        for sid, (off, size, name) in syms.items():
            if name == 'mod_cc13_cct_data' and size % 56 == 0 and size >= 112:
                base = o - off
                if base < 0:
                    continue
                ok = all(is_ccm(floats(d, base + off + k * 56 + 8, 9))
                         for k in range(size // 56))
                if ok:
                    return base
    raise SystemExit('data base not found')


def aec_nodes(d, base, syms, prefix, child_ok):
    """Find AEC trigger nodes of a module by structure (symbol names label
    the child type, not the content): entries of 24 bytes { f32 lux0, lux1,
    gain0, gain1, u32 count, u32 child } whose children pass child_ok()."""
    for sid, (off, size, name) in sorted(syms.items()):
        if not name.startswith('mod_%s_' % prefix) or size % 24 or size < 48:
            continue
        p = base + off
        if p < 0 or p + size > len(d):
            continue
        ents = []
        for k in range(size // 24):
            l0, l1, g0, g1 = floats(d, p + k * 24, 4)
            cnt, child = struct.unpack_from('<II', d, p + k * 24 + 16)
            child += 1      # stored as the id preceding the child symbol
            if not (0 <= l0 < l1 <= 10000 and 0 < g0 <= g1 and 0 < cnt < 32
                    and child in syms):
                ents = []
                break
            ents.append((l0, l1, cnt, child))
        if ents and all(child_ok(e[3]) for e in ents):
            yield ents


def pick_lux(ents, lux=LUX_NORMAL):
    def dist(e):
        return 0 if e[0] <= lux <= e[1] else min(abs(lux - e[0]), abs(lux - e[1]))
    return min(ents, key=dist)


def ccm_block(d, base, syms, sid):
    off, size, _ = syms[sid]
    if size % 56:
        return None
    regs = []
    for k in range(size // 56):
        p = base + off + k * 56
        if p < 0 or p + 56 > len(d):
            return None
        m = floats(d, p + 8, 9)
        if not is_ccm(m):
            return None
        regs.append((floats(d, p, 2), m))
    return regs


def cc13(d, base, syms):
    ok = lambda sid: ccm_block(d, base, syms, sid) is not None
    for ents in aec_nodes(d, base, syms, 'cc13', ok):
        e = pick_lux(ents)
        return (e[0], e[1]), ccm_block(d, base, syms, e[3])
    raise SystemExit('cc13 not found')


LSC_W, LSC_H = 17, 13
LSC_BLOCK = 8 + LSC_W * LSC_H * 4 * 4


def lsc_blocks(d, base, syms, sid):
    """Child of an LSC AEC entry: either data blocks directly, or a CCT
    node { f32 cct0, cct1, u32 count, u32 child } pointing to data blocks."""
    off, size, _ = syms[sid]
    p = base + off
    if p < 0 or p + size > len(d):
        return None
    if size % LSC_BLOCK == 0:
        return [p + k * LSC_BLOCK for k in range(size // LSC_BLOCK)]
    if size % 16 == 0:
        out = []
        for k in range(size // 16):
            n, cid = struct.unpack_from('<II', d, p + k * 16 + 8)
            cid += 1
            if cid not in syms:
                return None
            co, cs, _ = syms[cid]
            if cs % LSC_BLOCK:
                return None
            out += [base + co + j * LSC_BLOCK for j in range(cs // LSC_BLOCK)]
        return out
    return None


def lsc_table(d, p):
    v = floats(d, p + 8, LSC_W * LSC_H * 4)
    ch = [v[c * LSC_W * LSC_H:(c + 1) * LSC_W * LSC_H] for c in range(4)]
    if not all(0.9 < min(t) and max(t) < 16 for t in ch):
        return None
    return floats(d, p, 2), ch


def lsc34(d, base, syms):
    def ok(sid):
        b = lsc_blocks(d, base, syms, sid)
        return bool(b) and all(lsc_table(d, p) for p in b)
    for ents in aec_nodes(d, base, syms, 'lsc34', ok):
        e = pick_lux(ents)
        return (e[0], e[1]), [lsc_table(d, p) for p in lsc_blocks(d, base, syms, e[3])]
    raise SystemExit('lsc34 not found')


def awb_record(d, base, name):
    """AWB tuning records: name[52], u32 offset, u32 size, u32 id. The data
    starts with a 24 byte header."""
    i = d.find(name.encode() + b'\0')
    if i < 0:
        return None
    off, size = struct.unpack_from('<II', d, i + 0x34)
    return base + off + 24, size - 24


def awb_refpts(d, base):
    rec = awb_record(d, base, 'refPtV1')
    if not rec:
        return None
    p, size = rec
    pts = []
    for k in range(size // 8):
        rg, bg = floats(d, p + k * 8, 2)
        if not (0.05 < rg < 4 and 0.05 < bg < 4):
            break
        pts.append((rg, bg))
    return pts


def awb_init(d, base):
    """Default white balance: gains R, G, B and its colour temperature."""
    rec = awb_record(d, base, 'initDataV1')
    if not rec:
        return None
    r, g, b, ct = floats(d, rec[0], 4)
    return r / g, b / g, ct


def resample(t, w, h, n=17):
    out = []
    for y in range(n):
        fy = y * (h - 1) / (n - 1)
        y0 = min(int(fy), h - 2)
        ay = fy - y0
        for x in range(n):
            fx = x * (w - 1) / (n - 1)
            x0 = min(int(fx), w - 2)
            ax = fx - x0
            a = t[y0 * w + x0] * (1 - ax) + t[y0 * w + x0 + 1] * ax
            b = t[(y0 + 1) * w + x0] * (1 - ax) + t[(y0 + 1) * w + x0 + 1] * ax
            out.append(a * (1 - ay) + b * ay)
    return out


def ct_of(c0, c1):
    if c0 <= 1:
        return int(c1 if c1 < 3000 else c1)  # open low end: use upper bound
    if c1 >= 9999:
        return int(c0 + 700)
    return int(round((c0 + c1) / 2 / 50) * 50)


def fmt_ints(v, per_line, indent):
    lines = [', '.join('%3d' % x for x in v[k:k + per_line])
             for k in range(0, len(v), per_line)]
    return '[ ' + (',\n' + ' ' * indent + '  ').join(lines) + ' ]'


def fmt_list(v, per_line, indent):
    lines = []
    for k in range(0, len(v), per_line):
        lines.append(', '.join('%.4f' % x for x in v[k:k + per_line]))
    pad = ' ' * indent
    return '[ ' + (',\n' + pad + '  ').join(lines) + ' ]'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bin')
    ap.add_argument('--sensor', required=True)
    ap.add_argument('--awb-ct', help='comma separated CT per AWB reference '
                    'point (0 = skip point)')
    ap.add_argument('--no-lsc', action='store_true')
    ap.add_argument('--af-scan', help='MIN,MAX: lens positions to scan; '
                    'adds the Af algorithm (only for cameras with a focus lens)')
    ap.add_argument('--af-map', help='D0,P0,D1,P1,...: dioptres to lens '
                    'position map for the Af algorithm')
    ap.add_argument('--af-steps', default='160,40',
                    help='COARSE,FINE: Af scan steps in lens positions')
    ap.add_argument('--max-size', help='WxH: limit softisp output sizes, so '
                    'that libcamera picks a sensor mode no larger than this')
    a = ap.parse_args()

    d = open(a.bin, 'rb').read()
    syms = symbols(d)
    base = find_base(d, syms)

    lux, ccms = cc13(d, base, syms)
    out = ['# SPDX-License-Identifier: CC0-1.0',
           '%YAML 1.1',
           '---',
           '# %s: generated by chromatix2libcamera.py from %s' % (a.sensor, a.bin.split('/')[-1]),
           '# CCM and LSC from the Qualcomm Chromatix AEC region lux index %g-%g.' % lux,
           'version: 1']
    if a.max_size:
        w, h = a.max_size.split('x')
        out += ['options:',
                '  max_output_width: %s' % w,
                '  max_output_height: %s' % h]
    out += ['algorithms:',
           '  - BlackLevel:',
           '  - Awb:']

    pts = awb_refpts(d, base)
    if pts and a.awb_ct:
        cts = [int(x) for x in a.awb_ct.split(',')]
        gains = sorted((ct, 1 / rg, 1 / bg) for ct, (rg, bg) in zip(cts, pts) if ct)
        out.append('      colourGains:')
        for ct, r, b in gains:
            out.append('        - ct: %d' % ct)
            out.append('          gains: [ %.4f, %.4f ]' % (r, b))

    out.append('  - Ccm:')
    out.append('      ccms:')
    seen = set()
    for (c0, c1), m in ccms:
        ct = ct_of(c0, c1)
        if ct in seen:
            continue
        seen.add(ct)
        out.append('        - ct: %d' % ct)
        out.append('          ccm: ' + fmt_list(m, 3, 15))
        out.append('          offsets: [ 0, 0, 0 ]')

    if not a.no_lsc:
        llux, sets = lsc34(d, base, syms)
        out.append('  - Lsc:')
        out.append('      type: table')
        out.append('      sets:')
        seen = set()
        for (c0, c1), (r, gr, gb, b) in sets:
            ct = ct_of(c0, c1) if c0 > 1 or c1 < 9999 else 5000
            if ct in seen:
                continue
            seen.add(ct)
            g = [(x + y) / 2 for x, y in zip(gr, gb)]
            out.append('        - ct: %d' % ct)
            for key, t in (('r', r), ('g', g), ('b', b)):
                # Tabular LSC data is stored pre-quantized in the format of
                # the platform: softisp uses UQ2.6 (gain * 64, max 3.98).
                q = [min(255, max(0, round(x * 64))) for x in resample(t, LSC_W, LSC_H)]
                out.append('          %s: ' % key + fmt_ints(q, 17, 13))
    out += ['  - Adjust:', '  - Agc:']
    if a.af_scan:
        lo, hi = a.af_scan.split(',')
        coarse, fine = a.af_steps.split(',')
        out += ['  - Af:',
                '      scanRange: [ %s, %s ]' % (lo, hi),
                '      coarseStep: %s' % coarse,
                '      fineStep: %s' % fine]
        if a.af_map:
            out.append('      map: [ %s ]' % ', '.join(a.af_map.split(',')))
    out.append('...')
    print('\n'.join(out))

    print('# refPt (R/G, B/G):', ' '.join('%.3f/%.3f' % p for p in (pts or [])),
          file=sys.stderr)
    init = awb_init(d, base)
    if init:
        print('# initData: R gain %.3f B gain %.3f (R/G %.3f, B/G %.3f) at %d K'
              % (init[0], init[1], 1 / init[0], 1 / init[1], init[2]), file=sys.stderr)


if __name__ == '__main__':
    main()
