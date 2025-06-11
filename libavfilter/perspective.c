/*
 * Copyright (c) 2002 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (c) 2013 Paul B Mahol
 * Copyright (c) 2025 Quentin Renard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "filters.h"
#include "perspective.h"

static inline double get_coeff(double d)
{
    double coeff, A = -0.60;

    d = fabs(d);

    if (d < 1.0)
        coeff = (1.0 - (A + 3.0) * d * d + (A + 2.0) * d * d * d);
    else if (d < 2.0)
        coeff = (-4.0 * A + 8.0 * A * d - 5.0 * A * d * d + A * d * d * d);
    else
        coeff = 0.0;

    return coeff;
}

typedef struct ThreadData {
    PerspectiveResampleContext *s;
    uint8_t *dst;
    int dst_linesize;
    uint8_t *src;
    int src_linesize;
    int w, h;
    int hsub, vsub;
} ThreadData;

static int perspective_resample_cubic(AVFilterContext *ctx, void *arg,
                          int job, int nb_jobs)
{
    ThreadData *td = arg;
    uint8_t *dst = td->dst;
    int dst_linesize = td->dst_linesize;
    uint8_t *src = td->src;
    int src_linesize = td->src_linesize;
    int w = td->w;
    int h = td->h;
    int hsub = td->hsub;
    int vsub = td->vsub;
    int start = (h * job) / nb_jobs;
    int end   = (h * (job+1)) / nb_jobs;
    const int linesize = td->s->linesize[0];
    int x, y;

    for (y = start; y < end; y++) {
        int sy = y << vsub;
        for (x = 0; x < w; x++) {
            int u, v, subU, subV, sum, sx;

            sx   = x << hsub;
            u    = td->s->pv[sx + sy * linesize][0] >> hsub;
            v    = td->s->pv[sx + sy * linesize][1] >> vsub;
            subU = u & (PERSPECTIVE_RESAMPLE_SUB_PIXELS - 1);
            subV = v & (PERSPECTIVE_RESAMPLE_SUB_PIXELS - 1);
            u  >>= PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS;
            v  >>= PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS;

            if (u > 0 && v > 0 && u < w - 2 && v < h - 2){
                const int index = u + v*src_linesize;
                const int a = td->s->coeff[subU][0];
                const int b = td->s->coeff[subU][1];
                const int c = td->s->coeff[subU][2];
                const int d = td->s->coeff[subU][3];

                sum = td->s->coeff[subV][0] * (a * src[index - 1 -     src_linesize] + b * src[index - 0 -     src_linesize]  +
                                      c *      src[index + 1 -     src_linesize] + d * src[index + 2 -     src_linesize]) +
                      td->s->coeff[subV][1] * (a * src[index - 1                   ] + b * src[index - 0                   ]  +
                                      c *      src[index + 1                   ] + d * src[index + 2                   ]) +
                      td->s->coeff[subV][2] * (a * src[index - 1 +     src_linesize] + b * src[index - 0 +     src_linesize]  +
                                      c *      src[index + 1 +     src_linesize] + d * src[index + 2 +     src_linesize]) +
                      td->s->coeff[subV][3] * (a * src[index - 1 + 2 * src_linesize] + b * src[index - 0 + 2 * src_linesize]  +
                                      c *      src[index + 1 + 2 * src_linesize] + d * src[index + 2 + 2 * src_linesize]);
            } else {
                int dx, dy;

                sum = 0;

                for (dy = 0; dy < 4; dy++) {
                    int iy = v + dy - 1;

                    if (iy < 0)
                        iy = 0;
                    else if (iy >= h)
                        iy = h-1;
                    for (dx = 0; dx < 4; dx++) {
                        int ix = u + dx - 1;

                        if (ix < 0)
                            ix = 0;
                        else if (ix >= w)
                            ix = w - 1;

                        sum += td->s->coeff[subU][dx] * td->s->coeff[subV][dy] * src[ ix + iy * src_linesize];
                    }
                }
            }

            sum = (sum + (1<<(PERSPECTIVE_RESAMPLE_COEFF_BITS * 2 - 1))) >> (PERSPECTIVE_RESAMPLE_COEFF_BITS * 2);
            sum = av_clip_uint8(sum);
            dst[x + y * dst_linesize] = sum;
        }
    }
    return 0;
}

static int perspective_resample_linear(AVFilterContext *ctx, void *arg,
                           int job, int nb_jobs)
{
    ThreadData *td = arg;
    uint8_t *dst = td->dst;
    int dst_linesize = td->dst_linesize;
    uint8_t *src = td->src;
    int src_linesize = td->src_linesize;
    int w = td->w;
    int h = td->h;
    int hsub = td->hsub;
    int vsub = td->vsub;
    int start = (h * job) / nb_jobs;
    int end   = (h * (job+1)) / nb_jobs;
    const int linesize = td->s->linesize[0];
    int x, y;

    for (y = start; y < end; y++){
        int sy = y << vsub;
        for (x = 0; x < w; x++){
            int u, v, subU, subV, sum, sx, index, subUI, subVI;

            sx   = x << hsub;
            u    = td->s->pv[sx + sy * linesize][0] >> hsub;
            v    = td->s->pv[sx + sy * linesize][1] >> vsub;
            subU = u & (PERSPECTIVE_RESAMPLE_SUB_PIXELS - 1);
            subV = v & (PERSPECTIVE_RESAMPLE_SUB_PIXELS - 1);
            u  >>= PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS;
            v  >>= PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS;

            index = u + v * src_linesize;
            subUI = PERSPECTIVE_RESAMPLE_SUB_PIXELS - subU;
            subVI = PERSPECTIVE_RESAMPLE_SUB_PIXELS - subV;

            if ((unsigned)u < (unsigned)(w - 1)){
                if((unsigned)v < (unsigned)(h - 1)){
                    sum = subVI * (subUI * src[index] + subU * src[index + 1]) +
                          subV  * (subUI * src[index + src_linesize] + subU * src[index + src_linesize + 1]);
                    sum = (sum + (1 << (PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS * 2 - 1)))>> (PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS * 2);
                } else {
                    if (v < 0)
                        v = 0;
                    else
                        v = h - 1;
                    index = u + v * src_linesize;
                    sum   = subUI * src[index] + subU * src[index + 1];
                    sum   = (sum + (1 << (PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS - 1))) >> PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS;
                }
            } else {
                if (u < 0)
                    u = 0;
                else
                    u = w - 1;
                if ((unsigned)v < (unsigned)(h - 1)){
                    index = u + v * src_linesize;
                    sum   = subVI * src[index] + subV * src[index + src_linesize];
                    sum   = (sum + (1 << (PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS - 1))) >> PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS;
                } else {
                    if (v < 0)
                        v = 0;
                    else
                        v = h - 1;
                    index = u + v * src_linesize;
                    sum   = src[index];
                }
            }

            sum = av_clip_uint8(sum);
            dst[x + y * dst_linesize] = sum;
        }
    }
    return 0;
}

PerspectiveResampleContext* perspective_resample_context_alloc(int interpolation, int sense)
{
    int i, j;

    PerspectiveResampleContext *s = av_malloc(sizeof(*s));

    if (!s)
        return NULL;

    s->pix_fmt = AV_PIX_FMT_NONE;
    s->sense = sense;

    switch (interpolation) {
    case PERSPECTIVE_RESAMPLE_INTERPOLATION_LINEAR: s->resample = perspective_resample_linear; break;
    case PERSPECTIVE_RESAMPLE_INTERPOLATION_CUBIC:  s->resample = perspective_resample_cubic;  break;
    }

    for (i = 0; i < PERSPECTIVE_RESAMPLE_SUB_PIXELS; i++){
        double d = i / (double)PERSPECTIVE_RESAMPLE_SUB_PIXELS;
        double temp[4];
        double sum = 0;

        for (j = 0; j < 4; j++)
            temp[j] = get_coeff(j - d - 1);

        for (j = 0; j < 4; j++)
            sum += temp[j];

        for (j = 0; j < 4; j++)
            s->coeff[i][j] = lrint((1 << PERSPECTIVE_RESAMPLE_COEFF_BITS) * temp[j] / sum);
    }

    return s;
}

void perspective_resample_context_free(PerspectiveResampleContext **s)
{
    if (!s || !*s)
        return;

    av_freep(&(*s)->pv);
    av_freep(s);
}

static int luts_parameters_changed(PerspectiveResampleContext *s, int w, int h, double ref[4][2])
{
    if (s->w != w || s->h != h)
        return 1;

    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 2; j++) {
            if (ref[i][j] != s->ref[i][j])
                return 1;
        }
    }

    return 0;
}

static int calc_luts(PerspectiveResampleContext *s, int w, int h, double ref[4][2])
{
    double x0, x1, x2, x3, x4, x5, x6, x7, x8, q;
    double t0, t1, t2, t3;
    int x, y;

    switch (s->sense) {
    case PERSPECTIVE_RESAMPLE_SENSE_SOURCE:
        x6 = ((ref[0][0] - ref[1][0] - ref[2][0] + ref[3][0]) *
              (ref[2][1] - ref[3][1]) -
             ( ref[0][1] - ref[1][1] - ref[2][1] + ref[3][1]) *
              (ref[2][0] - ref[3][0])) * h;
        x7 = ((ref[0][1] - ref[1][1] - ref[2][1] + ref[3][1]) *
              (ref[1][0] - ref[3][0]) -
             ( ref[0][0] - ref[1][0] - ref[2][0] + ref[3][0]) *
              (ref[1][1] - ref[3][1])) * w;
        q =  ( ref[1][0] - ref[3][0]) * (ref[2][1] - ref[3][1]) -
             ( ref[2][0] - ref[3][0]) * (ref[1][1] - ref[3][1]);

        x0 = q * (ref[1][0] - ref[0][0]) * h + x6 * ref[1][0];
        x1 = q * (ref[2][0] - ref[0][0]) * w + x7 * ref[2][0];
        x2 = q *  ref[0][0] * w * h;
        x3 = q * (ref[1][1] - ref[0][1]) * h + x6 * ref[1][1];
        x4 = q * (ref[2][1] - ref[0][1]) * w + x7 * ref[2][1];
        x5 = q *  ref[0][1] * w * h;
        x8 = q * w * h;
        break;
    case PERSPECTIVE_RESAMPLE_SENSE_DESTINATION:
        t0 = ref[0][0] * (ref[3][1] - ref[1][1]) +
             ref[1][0] * (ref[0][1] - ref[3][1]) +
             ref[3][0] * (ref[1][1] - ref[0][1]);
        t1 = ref[1][0] * (ref[2][1] - ref[3][1]) +
             ref[2][0] * (ref[3][1] - ref[1][1]) +
             ref[3][0] * (ref[1][1] - ref[2][1]);
        t2 = ref[0][0] * (ref[3][1] - ref[2][1]) +
             ref[2][0] * (ref[0][1] - ref[3][1]) +
             ref[3][0] * (ref[2][1] - ref[0][1]);
        t3 = ref[0][0] * (ref[1][1] - ref[2][1]) +
             ref[1][0] * (ref[2][1] - ref[0][1]) +
             ref[2][0] * (ref[0][1] - ref[1][1]);

        x0 = t0 * t1 * w * (ref[2][1] - ref[0][1]);
        x1 = t0 * t1 * w * (ref[0][0] - ref[2][0]);
        x2 = t0 * t1 * w * (ref[0][1] * ref[2][0] - ref[0][0] * ref[2][1]);
        x3 = t1 * t2 * h * (ref[1][1] - ref[0][1]);
        x4 = t1 * t2 * h * (ref[0][0] - ref[1][0]);
        x5 = t1 * t2 * h * (ref[0][1] * ref[1][0] - ref[0][0] * ref[1][1]);
        x6 = t1 * t2 * (ref[1][1] - ref[0][1]) +
             t0 * t3 * (ref[2][1] - ref[3][1]);
        x7 = t1 * t2 * (ref[0][0] - ref[1][0]) +
             t0 * t3 * (ref[3][0] - ref[2][0]);
        x8 = t1 * t2 * (ref[0][1] * ref[1][0] - ref[0][0] * ref[1][1]) +
             t0 * t3 * (ref[2][0] * ref[3][1] - ref[2][1] * ref[3][0]);
        break;
    default:
        av_assert0(0);
    }

    for (y = 0; y < h; y++){
        for (x = 0; x < w; x++){
            int u, v;

            u =      lrint(PERSPECTIVE_RESAMPLE_SUB_PIXELS * (x0 * x + x1 * y + x2) /
                                        (x6 * x + x7 * y + x8));
            v =      lrint(PERSPECTIVE_RESAMPLE_SUB_PIXELS * (x3 * x + x4 * y + x5) /
                                        (x6 * x + x7 * y + x8));

            s->pv[x + y * w][0] = u;
            s->pv[x + y * w][1] = v;
        }
    }

    return 0;
}

int perspective_resample_config_props(PerspectiveResampleContext *s, int w, int h, int pix_fmt, double ref[4][2])
{
    int i, j, ret;

    if (s->pix_fmt != pix_fmt) {
        s->desc = av_pix_fmt_desc_get(pix_fmt);
        s->hsub = s->desc->log2_chroma_w;
        s->vsub = s->desc->log2_chroma_h;
        s->nb_planes = av_pix_fmt_count_planes(pix_fmt);
    }

    if (s->pix_fmt != pix_fmt || s->w != w) {
        if ((ret = av_image_fill_linesizes(s->linesize, pix_fmt, w)) < 0)
            return ret;
    }

    if (s->pix_fmt != pix_fmt || s->h != h) {
        s->height[1] = s->height[2] = AV_CEIL_RSHIFT(h, s->desc->log2_chroma_h);
        s->height[0] = s->height[3] = h;
    }

    if (s->w != w || s->h !=h) {
        s->pv = av_realloc_f(s->pv, w * h, 2 * sizeof(*s->pv));
        if (!s->pv)
            return AVERROR(ENOMEM);
    }

    if (luts_parameters_changed(s, w, h, ref)) {
        if ((ret = calc_luts(s, w, h, ref)) < 0)
            return ret;
    }
    
    s->w = w;
    s->h = h;
    s->pix_fmt = pix_fmt;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 2; j++) {
            s->ref[i][j] = ref[i][j];
        }
    }
    return 0;
}

void perspective_resample(PerspectiveResampleContext *s, AVFilterContext *ctx, AVFrame *src, AVFrame *dst)
{
    int plane;

    for (plane = 0; plane < s->nb_planes; plane++) {
        int hsub = plane == 1 || plane == 2 ? s->hsub : 0;
        int vsub = plane == 1 || plane == 2 ? s->vsub : 0;
        ThreadData td = {.s = s,
                         .dst = dst->data[plane],
                         .dst_linesize = dst->linesize[plane],
                         .src = src->data[plane],
                         .src_linesize = src->linesize[plane],
                         .w = s->linesize[plane],
                         .h = s->height[plane],
                         .hsub = hsub,
                         .vsub = vsub };
        ff_filter_execute(ctx, s->resample, &td, NULL,
                          FFMIN(td.h, ff_filter_get_nb_threads(ctx)));
    }
}
