/*
 * Copyright (c) 2025 Quentin Renard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/eval.h"
#include "libavutil/opt.h"
#include "video.h"

static const char *const var_names[] = {
    "in_w",   "iw",
    "in_h",   "ih",
    "z",
    "zw",
    "zh",
    "n",
    "t",
    NULL
};

enum var_name {
    VAR_IN_W,   VAR_IW,
    VAR_IN_H,   VAR_IH,
    VAR_Z,
    VAR_ZW,
    VAR_ZH,
    VAR_N,
    VAR_T,
    VARS_NB
};

typedef struct YAZFContext {
    const AVClass *class;
    char *x_expr_str, *y_expr_str, *w_expr_str, *h_expr_str, *zoom_expr_str;
    AVExpr *x_expr, *y_expr, *w_expr, *h_expr, *zoom_expr;
    double var_values[VARS_NB];
} YAZFContext;

typedef struct ThreadData {
    AVFrame *in, *out;
    float crop_h, crop_x, crop_y, crop_w;
    int w, h;
} ThreadData;

static av_cold int init(AVFilterContext *ctx)
{
    YAZFContext *s = ctx->priv;
    int ret;

    ret = av_expr_parse(&s->x_expr, s->x_expr_str, var_names, NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0)
        return ret;

    ret = av_expr_parse(&s->y_expr, s->y_expr_str, var_names, NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0)
        return ret;

    ret = av_expr_parse(&s->w_expr, s->w_expr_str, var_names, NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0)
        return ret;

    ret = av_expr_parse(&s->h_expr, s->h_expr_str, var_names, NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0)
        return ret;

    ret = av_expr_parse(&s->zoom_expr, s->zoom_expr_str, var_names, NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0)
        return ret;

    return 0;
}

static int config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    YAZFContext *s = ctx->priv;

    s->var_values[VAR_IN_W] = s->var_values[VAR_IW] = inlink->w;
    s->var_values[VAR_IN_H] = s->var_values[VAR_IH] = inlink->h;

    outlink->w = FFMAX(av_expr_eval(s->w_expr, s->var_values, NULL), 1);
    outlink->h = FFMAX(av_expr_eval(s->h_expr, s->var_values, NULL), 1);
    return 0;
}

static inline uint8_t zoompan_pixel(const uint8_t *src, const int src_stride, 
                                    const int src_w, const int src_h, 
                                    const float x, const float y)
{
    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    float fx = x - x0;
    float fy = y - y0;

    x0 = FFMAX(0, FFMIN(x0, src_w - 1));
    x1 = FFMAX(0, FFMIN(x1, src_w - 1));
    y0 = FFMAX(0, FFMIN(y0, src_h - 1));
    y1 = FFMAX(0, FFMIN(y1, src_h - 1));

    float p00 = src[y0 * src_stride + x0];
    float p10 = src[y0 * src_stride + x1];
    float p01 = src[y1 * src_stride + x0];
    float p11 = src[y1 * src_stride + x1];

    return (1 - fx) * (1 - fy) * p00 +
           fx * (1 - fy) * p10 +
           (1 - fx) * fy * p01 +
           fx * fy * p11;
}

static void zoompan_plane(const uint8_t *src, const int src_stride,
                            const int src_w, const int src_h, const float crop_x,
                            const float crop_y, const float crop_w,
                            const float crop_h, uint8_t *dst,
                            const int dst_stride, const int dst_w, const int dst_h,
                            const int dst_y_start, const int dst_y_end)
{
    float u, v, x, y, val;
    for (int dst_y = dst_y_start; dst_y < dst_y_end; dst_y++) {
        for (int dst_x = 0; dst_x < dst_w; dst_x++) {
            u = (dst_w > 1) ? (float)dst_x / (float)(dst_w - 1) : 0.0f;
            v = (dst_h > 1) ? (float)dst_y / (float)(dst_h - 1) : 0.0f;

            x = crop_x + u * crop_w;
            y = crop_y + v * crop_h;

            val = zoompan_pixel(src, src_stride, src_w, src_h, x, y);

            dst[dst_y * dst_stride + dst_x] = FFMIN(FFMAX(val, 0), 255);
        }
    }
}

static int zoompan_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFilterLink *inlink = ctx->inputs[0];
    ThreadData *td = arg;

    const int slice_start = (td->out->height * jobnr) / nb_jobs;
    const int slice_end = (td->out->height * (jobnr+1)) / nb_jobs;

    int nb_planes = 4;
    struct {
        int resolution_w;
        int resolution_h;
    } planes[4] = {
        {.resolution_h = 1, .resolution_w = 1},
        {.resolution_h = 1, .resolution_w = 1},
        {.resolution_h = 1, .resolution_w = 1},
        {.resolution_h = 1, .resolution_w = 1},
    };
    switch (inlink->format) {
        case AV_PIX_FMT_YUV410P:
            nb_planes = 3;
            planes[1].resolution_h = 4;
            planes[1].resolution_w = 4;
            planes[2].resolution_h = 4;
            planes[2].resolution_w = 4;
            break;
        case AV_PIX_FMT_YUV411P:
        case AV_PIX_FMT_YUVJ411P:
            nb_planes = 3;
            planes[1].resolution_w = 4;
            planes[2].resolution_w = 4;
            break;
        case AV_PIX_FMT_YUV420P:
        case AV_PIX_FMT_YUVJ420P:
            nb_planes = 3;
            planes[1].resolution_h = 2;
            planes[1].resolution_w = 2;
            planes[2].resolution_h = 2;
            planes[2].resolution_w = 2;
            break;
        case AV_PIX_FMT_YUVA420P:
            nb_planes = 4;
            planes[1].resolution_h = 2;
            planes[1].resolution_w = 2;
            planes[2].resolution_h = 2;
            planes[2].resolution_w = 2;
            break;
        case AV_PIX_FMT_YUV422P:
        case AV_PIX_FMT_YUVJ422P:
            nb_planes = 3;
            planes[1].resolution_w = 2;
            planes[2].resolution_w = 2;
            break;
        case AV_PIX_FMT_YUVA422P:
            nb_planes = 4;
            planes[1].resolution_w = 2;
            planes[2].resolution_w = 2;
            break;
        case AV_PIX_FMT_YUV440P:
        case AV_PIX_FMT_YUVJ440P:
            nb_planes = 3;
            planes[1].resolution_h = 2;
            planes[2].resolution_h = 2;
            break;
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_YUVJ444P:
            nb_planes = 3;
            break;
        case AV_PIX_FMT_YUVA444P:
            nb_planes = 4;
            break;
    }

    for (int i = 0; i < nb_planes; i++) {
        zoompan_plane(td->in->data[i], td->in->linesize[i], td->in->width/planes[i].resolution_w,
                    td->in->height/planes[i].resolution_h, td->crop_x/planes[i].resolution_w,
                    td->crop_y/planes[i].resolution_h, td->crop_w/planes[i].resolution_w,
                    td->crop_h/planes[i].resolution_h, td->out->data[i], td->out->linesize[i],
                    td->out->width/planes[i].resolution_w, td->out->height/planes[i].resolution_h,
                    slice_start/planes[i].resolution_h, slice_end/planes[i].resolution_h);
    }
    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    YAZFContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    int ret;
    AVFrame *out = NULL;
    ThreadData td;
    float zoom, a;

    inlink->w = in->width;
    inlink->h = in->height;
    s->var_values[VAR_N] = ff_filter_link(inlink)->frame_count_out;
    s->var_values[VAR_T] = TS2T(in->pts, inlink->time_base);

    if ((ret = config_props(outlink)) < 0)
        goto err;

    td.w = outlink->w;
    td.h = outlink->h;
    a = (float)outlink->w / (float)outlink->h;
    
    s->var_values[VAR_Z] = zoom = av_clipd(av_expr_eval(s->zoom_expr, s->var_values, NULL), 1, 10);
    
    td.crop_w = (float)inlink->w / zoom;
    td.crop_h = td.crop_w / a;
    if (td.crop_h > inlink->h) {
        td.crop_h = inlink->h;
        td.crop_w = td.crop_h * a;
    }
    s->var_values[VAR_ZW] = td.crop_w;
    s->var_values[VAR_ZH] = td.crop_h;
    
    td.crop_x = av_clipd(av_expr_eval(s->x_expr, s->var_values, NULL), 0, FFMAX(inlink->w - td.crop_w, 0));
    td.crop_y = av_clipd(av_expr_eval(s->y_expr, s->var_values, NULL), 0, FFMAX(inlink->h - td.crop_h, 0));

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto err;
    }

    if ((ret = av_frame_copy_props(out, in)) < 0)
        goto err;

    td.out = out, td.in = in;
    if ((ret = ff_filter_execute(ctx, zoompan_slice, &td, NULL,
                                FFMIN(td.h, ff_filter_get_nb_threads(ctx)))) < 0)
        goto err;

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
    
err:
    av_frame_free(&in);
    av_frame_free(&out);
    return ret;
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUVJ411P, AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ440P,
    AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUVA422P,
    AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_NONE
};

static av_cold void uninit(AVFilterContext *ctx)
{
    YAZFContext *s = ctx->priv;

    av_expr_free(s->x_expr);
    av_expr_free(s->y_expr);
    av_expr_free(s->zoom_expr);
    av_expr_free(s->w_expr);
    av_expr_free(s->h_expr);
}

#define OFFSET(x) offsetof(YAZFContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption yazf_options[] = {
    { "z", "set the zoom expression", OFFSET(zoom_expr_str), AV_OPT_TYPE_STRING, {.str = "1" }, .flags = FLAGS },
    { "x", "set the zoom x expression", OFFSET(x_expr_str), AV_OPT_TYPE_STRING, {.str = "0" }, .flags = FLAGS },
    { "y", "set the zoom y expression", OFFSET(y_expr_str), AV_OPT_TYPE_STRING, {.str = "0" }, .flags = FLAGS },
    { "w", "set the output w expression", OFFSET(w_expr_str), AV_OPT_TYPE_STRING, {.str = "1" }, .flags = FLAGS },
    { "h", "set the output h expression", OFFSET(h_expr_str), AV_OPT_TYPE_STRING, {.str = "1" }, .flags = FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(yazf);

static const AVFilterPad avfilter_vf_yazf_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad avfilter_vf_yazf_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_props,
    },
};

const FFFilter ff_vf_yazf = {
    .p.name          = "yazf",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply Zoom & Pan effect with floating point precision."),
    .p.priv_class    = &yazf_class,
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS,
    .init            = init,
    .priv_size       = sizeof(YAZFContext),
    .uninit          = uninit,
    FILTER_INPUTS(avfilter_vf_yazf_inputs),
    FILTER_OUTPUTS(avfilter_vf_yazf_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};
