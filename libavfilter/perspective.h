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

#ifndef AVFILTER_PERSPECTIVE_H
#define AVFILTER_PERSPECTIVE_H

#include "libavutil/pixdesc.h"
#include "avfilter.h"

#define PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS  8
#define PERSPECTIVE_RESAMPLE_SUB_PIXELS      (1 << PERSPECTIVE_RESAMPLE_SUB_PIXEL_BITS)
#define PERSPECTIVE_RESAMPLE_COEFF_BITS      11

typedef struct PerspectiveResampleContext {
    int sense;
    int w, h, pix_fmt;
    const AVPixFmtDescriptor *desc;
    double ref[4][2];
    int32_t (*pv)[2];
    int32_t coeff[PERSPECTIVE_RESAMPLE_SUB_PIXELS][4];
    int linesize[4];
    int height[4];
    int hsub, vsub;
    int nb_planes;

    int (*resample)(AVFilterContext *ctx,
                       void *arg, int job, int nb_jobs);
} PerspectiveResampleContext;

typedef enum {
    PERSPECTIVE_RESAMPLE_INTERPOLATION_LINEAR = 0,
    PERSPECTIVE_RESAMPLE_INTERPOLATION_CUBIC  = 1,
} PerspectiveResampleInterpolation;

typedef enum {
    PERSPECTIVE_RESAMPLE_SENSE_SOURCE      = 0, ///< coordinates give locations in source of corners of destination.
    PERSPECTIVE_RESAMPLE_SENSE_DESTINATION = 1, ///< coordinates give locations in destination of corners of source.
} PerspectiveResampleSense;

PerspectiveResampleContext* perspective_resample_context_alloc(int interpolation, int sense);

void perspective_resample_context_free(PerspectiveResampleContext **s);

int perspective_resample_config_props(PerspectiveResampleContext *s, int w, int h, int pix_fmt, double ref[4][2]);

void perspective_resample(PerspectiveResampleContext *s, AVFilterContext *ctx, AVFrame *src, AVFrame *dst);

#endif /* AVFILTER_PERSPECTIVE_H */