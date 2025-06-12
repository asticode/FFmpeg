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
#include "perspective.h"
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
    PerspectiveResampleContext *r;
} YAZFContext;

static av_cold int init(AVFilterContext *ctx)
{
    YAZFContext *s = ctx->priv;
    int ret;

    s->r = perspective_resample_context_alloc(PERSPECTIVE_RESAMPLE_INTERPOLATION_LINEAR, PERSPECTIVE_RESAMPLE_SENSE_SOURCE);
    if (!s->r)
        return AVERROR(ENOMEM);

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

static int config_outlink(AVFilterLink *outlink)
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

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    YAZFContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    int ret;
    AVFrame *out = NULL;
    float zoom, a, crop_x, crop_y, crop_w, crop_h;
    double ref[4][2];

    inlink->w = in->width;
    inlink->h = in->height;
    s->var_values[VAR_N] = ff_filter_link(inlink)->frame_count_out;
    s->var_values[VAR_T] = TS2T(in->pts, inlink->time_base);

    if ((ret = config_outlink(outlink)) < 0)
        goto err;

    a = (float)outlink->w / (float)outlink->h;
    
    s->var_values[VAR_Z] = zoom = av_clipd(av_expr_eval(s->zoom_expr, s->var_values, NULL), 1, 10);
    
    crop_w = (float)inlink->w / zoom;
    crop_h = crop_w / a;
    if (crop_h > inlink->h) {
        crop_h = inlink->h;
        crop_w = crop_h * a;
    }
    s->var_values[VAR_ZW] = crop_w;
    s->var_values[VAR_ZH] = crop_h;
    
    crop_x = av_clipd(av_expr_eval(s->x_expr, s->var_values, NULL), 0, FFMAX(inlink->w - crop_w, 0));
    crop_y = av_clipd(av_expr_eval(s->y_expr, s->var_values, NULL), 0, FFMAX(inlink->h - crop_h, 0));

    ref[0][0] = crop_x;
    ref[0][1] = crop_y;
    ref[1][0] = crop_x + crop_w;
    ref[1][1] = crop_y;
    ref[2][0] = crop_x;
    ref[2][1] = crop_y + crop_h;
    ref[3][0] = crop_x + crop_w;
    ref[3][1] = crop_y + crop_h;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto err;
    }

    if ((ret = av_frame_copy_props(out, in)) < 0)
        goto err;

    if ((ret = perspective_resample_config_props(s->r, out->width, out->height, out->format, ref)) < 0) {
        goto err;
    }

    perspective_resample(s->r, ctx, in, out);

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

    perspective_resample_context_free(&s->r);
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
        .config_props = config_outlink,
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
