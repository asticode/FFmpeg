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
#include "libavutil/eval.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "perspective.h"
#include "video.h"

typedef struct PerspectiveContext {
    const AVClass *class;
    char *expr_str[4][2];
    int eval_mode;
    int interpolation;
    int sense;
    PerspectiveResampleContext *r;
} PerspectiveContext;

#define OFFSET(x) offsetof(PerspectiveContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

enum EvalMode {
    EVAL_MODE_INIT,
    EVAL_MODE_FRAME,
    EVAL_MODE_NB
};

static const AVOption perspective_options[] = {
    { "x0", "set top left x coordinate",     OFFSET(expr_str[0][0]), AV_OPT_TYPE_STRING, {.str="0"}, 0, 0, FLAGS },
    { "y0", "set top left y coordinate",     OFFSET(expr_str[0][1]), AV_OPT_TYPE_STRING, {.str="0"}, 0, 0, FLAGS },
    { "x1", "set top right x coordinate",    OFFSET(expr_str[1][0]), AV_OPT_TYPE_STRING, {.str="W"}, 0, 0, FLAGS },
    { "y1", "set top right y coordinate",    OFFSET(expr_str[1][1]), AV_OPT_TYPE_STRING, {.str="0"}, 0, 0, FLAGS },
    { "x2", "set bottom left x coordinate",  OFFSET(expr_str[2][0]), AV_OPT_TYPE_STRING, {.str="0"}, 0, 0, FLAGS },
    { "y2", "set bottom left y coordinate",  OFFSET(expr_str[2][1]), AV_OPT_TYPE_STRING, {.str="H"}, 0, 0, FLAGS },
    { "x3", "set bottom right x coordinate", OFFSET(expr_str[3][0]), AV_OPT_TYPE_STRING, {.str="W"}, 0, 0, FLAGS },
    { "y3", "set bottom right y coordinate", OFFSET(expr_str[3][1]), AV_OPT_TYPE_STRING, {.str="H"}, 0, 0, FLAGS },
    { "interpolation", "set interpolation", OFFSET(interpolation), AV_OPT_TYPE_INT, {.i64=PERSPECTIVE_RESAMPLE_INTERPOLATION_LINEAR}, 0, 1, FLAGS, .unit = "interpolation" },
    {      "linear", "", 0, AV_OPT_TYPE_CONST, {.i64=PERSPECTIVE_RESAMPLE_INTERPOLATION_LINEAR}, 0, 0, FLAGS, .unit = "interpolation" },
    {       "cubic", "", 0, AV_OPT_TYPE_CONST, {.i64=PERSPECTIVE_RESAMPLE_INTERPOLATION_CUBIC},  0, 0, FLAGS, .unit = "interpolation" },
    { "sense",   "specify the sense of the coordinates", OFFSET(sense), AV_OPT_TYPE_INT, {.i64=PERSPECTIVE_RESAMPLE_SENSE_SOURCE}, 0, 1, FLAGS, .unit = "sense"},
    {       "source", "specify locations in source to send to corners in destination",
                0, AV_OPT_TYPE_CONST, {.i64=PERSPECTIVE_RESAMPLE_SENSE_SOURCE}, 0, 0, FLAGS, .unit = "sense"},
    {       "destination", "specify locations in destination to send corners of source",
                0, AV_OPT_TYPE_CONST, {.i64=PERSPECTIVE_RESAMPLE_SENSE_DESTINATION}, 0, 0, FLAGS, .unit = "sense"},
    { "eval", "specify when to evaluate expressions", OFFSET(eval_mode), AV_OPT_TYPE_INT, {.i64 = EVAL_MODE_INIT}, 0, EVAL_MODE_NB-1, FLAGS, .unit = "eval" },
         { "init",  "eval expressions once during initialization", 0, AV_OPT_TYPE_CONST, {.i64=EVAL_MODE_INIT},  .flags = FLAGS, .unit = "eval" },
         { "frame", "eval expressions per-frame",                  0, AV_OPT_TYPE_CONST, {.i64=EVAL_MODE_FRAME}, .flags = FLAGS, .unit = "eval" },

    { NULL }
};

AVFILTER_DEFINE_CLASS(perspective);

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA420P,
    AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ422P,AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ411P,
    AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
    AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRAP, AV_PIX_FMT_GRAY8, AV_PIX_FMT_NONE
};

static const char *const var_names[] = {   "W",   "H",   "in",   "on",        NULL };
enum                                   { VAR_W, VAR_H, VAR_IN, VAR_ON, VAR_VARS_NB };

static int config_props(AVFilterContext *ctx, int w, int h, int pix_fmt)
{
    PerspectiveContext *s = ctx->priv;
    AVFilterLink *inlink    = ctx->inputs[0];
    FilterLink *inl       = ff_filter_link(inlink);
    AVFilterLink *outlink = ctx->outputs[0];
    FilterLink *outl      = ff_filter_link(outlink);
    double ref[4][2];

    double values[VAR_VARS_NB] = { [VAR_W] = w, [VAR_H] = h,
                                   [VAR_IN] = inl->frame_count_out + 1,
                                   [VAR_ON] = outl->frame_count_in + 1 };
    int i, j, ret;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 2; j++) {
            if (!s->expr_str[i][j])
                return AVERROR(EINVAL);
            ret = av_expr_parse_and_eval(&ref[i][j], s->expr_str[i][j],
                                         var_names, &values[0],
                                         NULL, NULL, NULL, NULL,
                                         0, 0, ctx);
            if (ret < 0)
                return ret;
        }
    }

    return perspective_resample_config_props(s->r, w, h, pix_fmt, ref);
}

static int config_input(AVFilterLink *inlink)
{
    return config_props(inlink->dst, inlink->w, inlink->h, inlink->format);
}

static av_cold int init(AVFilterContext *ctx)
{
    PerspectiveContext *s = ctx->priv;

    s->r = perspective_resample_context_alloc(s->interpolation, s->sense);
    if (!s->r)
        return AVERROR(ENOMEM);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    PerspectiveContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        av_frame_free(&frame);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, frame);

    if (s->eval_mode == EVAL_MODE_FRAME) {
        if ((ret = config_props(ctx, frame->width, frame->height, frame->format)) < 0) {
            av_frame_free(&out);
            return ret;
        }
    }

    perspective_resample(s->r, ctx, frame, out);

    av_frame_free(&frame);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    PerspectiveContext *s = ctx->priv;

    perspective_resample_context_free(&s->r);
}

static const AVFilterPad perspective_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_perspective = {
    .p.name        = "perspective",
    .p.description = NULL_IF_CONFIG_SMALL("Correct the perspective of video."),
    .p.priv_class  = &perspective_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(PerspectiveContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(perspective_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};
