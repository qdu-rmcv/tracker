"""
Build iGPU-friendly OpenVINO end-to-end YOLO11-pose ONNX.

Compared with build_openvino_end2end.py, this script avoids full per-class NMS
over all anchors. It is faster on Intel integrated GPUs because it first reduces
the candidates:

  raw YOLO head [1, F, N]
    -> Transpose [1, N, F]
    -> Slice: cxcywh / scores / kpts
    -> best_score = ReduceMax(scores, axis=class)
    -> best_class = ArgMax(scores, axis=class)
    -> pre-TopK anchors by best_score, e.g. 300 out of 8400
    -> Gather boxes / classes / keypoints for only those anchors
    -> class-agnostic ONNX NonMaxSuppression on [1, pre_topk, 4]
    -> pad + global TopK to fixed keep_topk
    -> ONNX outputs aligned with TensorRT e2e:
         num_dets     [1, 1]                 int32
         det_boxes    [1, keep_topk, 4]      float32, xyxy
         det_scores   [1, keep_topk]         float32
         det_classes  [1, keep_topk]         int32
         det_kpts     [1, keep_topk, K*D]    float32

Trade-off:
  This is not exactly the same as TensorRT EfficientNMS_TRT with class_agnostic=False.
  It assumes each anchor uses its best class only, which is usually fine for YOLO-style
  detectors and is much more friendly to Intel iGPU / OpenVINO GPU execution.
"""

from __future__ import annotations

import argparse

import numpy as np
import onnx
import onnx_graphsurgeon as gs
from onnx import TensorProto


def _const(name: str, arr, dtype=np.int64):
    return gs.Constant(name, np.array(arr, dtype=dtype))


def _scalar_const(name: str, value, dtype):
    return gs.Constant(name, np.array(value, dtype=dtype))


def _slice(
    graph: gs.Graph,
    name: str,
    inp,
    start: int,
    end: int,
    axis: int,
    dtype=None,
    shape=None,
):
    out = gs.Variable(name, dtype=dtype or inp.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Slice",
            name=f"{name}_slice",
            inputs=[
                inp,
                _const(f"{name}_starts", [start]),
                _const(f"{name}_ends", [end]),
                _const(f"{name}_axes", [axis]),
            ],
            outputs=[out],
        )
    )
    return out


def _unsqueeze(graph: gs.Graph, name: str, inp, axes, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype or inp.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Unsqueeze",
            name=f"{name}_unsqueeze",
            inputs=[inp, _const(f"{name}_axes", axes)],
            outputs=[out],
        )
    )
    return out


def _squeeze(graph: gs.Graph, name: str, inp, axes, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype or inp.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Squeeze",
            name=f"{name}_squeeze",
            inputs=[inp, _const(f"{name}_axes", axes)],
            outputs=[out],
        )
    )
    return out


def _binary(graph: gs.Graph, op: str, name: str, a, b, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype, shape=shape)
    graph.nodes.append(gs.Node(op=op, name=name, inputs=[a, b], outputs=[out]))
    return out


def _concat(graph: gs.Graph, name: str, inputs, axis: int, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype or inputs[0].dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Concat", name=name, inputs=inputs, outputs=[out], attrs={"axis": axis}
        )
    )
    return out


def _transpose(graph: gs.Graph, name: str, inp, perm, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype or inp.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Transpose", name=name, inputs=[inp], outputs=[out], attrs={"perm": perm}
        )
    )
    return out


def _cast(graph: gs.Graph, name: str, inp, to_type: int, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Cast", name=name, inputs=[inp], outputs=[out], attrs={"to": to_type}
        )
    )
    return out


def _where(graph: gs.Graph, name: str, condition, x, y, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype or x.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(op="Where", name=name, inputs=[condition, x, y], outputs=[out])
    )
    return out


def _gathernd(graph: gs.Graph, name: str, data, indices, dtype=None, shape=None):
    out = gs.Variable(name, dtype=dtype or data.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(op="GatherND", name=name, inputs=[data, indices], outputs=[out])
    )
    return out


def _gather(
    graph: gs.Graph, name: str, data, indices, axis: int, dtype=None, shape=None
):
    out = gs.Variable(name, dtype=dtype or data.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Gather",
            name=name,
            inputs=[data, indices],
            outputs=[out],
            attrs={"axis": axis},
        )
    )
    return out


def _gather_elements(
    graph: gs.Graph, name: str, data, indices, axis: int, dtype=None, shape=None
):
    out = gs.Variable(name, dtype=dtype or data.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="GatherElements",
            name=name,
            inputs=[data, indices],
            outputs=[out],
            attrs={"axis": axis},
        )
    )
    return out


def _expand(graph: gs.Graph, name: str, data, shape, dtype=None):
    out = gs.Variable(name, dtype=dtype or data.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="Expand",
            name=name,
            inputs=[data, _const(f"{name}_shape", shape)],
            outputs=[out],
        )
    )
    return out


def _reduce_sum(
    graph: gs.Graph, name: str, inp, axes, keepdims: int, dtype=None, shape=None
):
    out = gs.Variable(name, dtype=dtype or inp.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="ReduceSum",
            name=name,
            inputs=[inp, _const(f"{name}_axes", axes)],
            outputs=[out],
            attrs={"keepdims": keepdims},
        )
    )
    return out


def _reduce_max(
    graph: gs.Graph, name: str, inp, axes, keepdims: int, dtype=None, shape=None
):
    out = gs.Variable(name, dtype=dtype or inp.dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="ReduceMax",
            name=name,
            inputs=[inp, _const(f"{name}_axes", axes)],
            outputs=[out],
            attrs={"keepdims": keepdims},
        )
    )
    return out


def _argmax(
    graph: gs.Graph,
    name: str,
    inp,
    axis: int,
    keepdims: int,
    dtype=np.int64,
    shape=None,
):
    out = gs.Variable(name, dtype=dtype, shape=shape)
    graph.nodes.append(
        gs.Node(
            op="ArgMax",
            name=name,
            inputs=[inp],
            outputs=[out],
            attrs={"axis": axis, "keepdims": keepdims, "select_last_index": 0},
        )
    )
    return out


def _topk_2d_axis1(graph: gs.Graph, name: str, data, k: int):
    values = gs.Variable(f"{name}_values", dtype=np.float32, shape=[1, k])
    indices = gs.Variable(f"{name}_indices", dtype=np.int64, shape=[1, k])
    graph.nodes.append(
        gs.Node(
            op="TopK",
            name=name,
            inputs=[data, _const(f"{name}_k", [k])],
            outputs=[values, indices],
            attrs={"axis": 1, "largest": 1, "sorted": 1},
        )
    )
    return values, indices


def _topk_1d(graph: gs.Graph, name: str, data, k: int):
    values = gs.Variable(f"{name}_values", dtype=np.float32, shape=[k])
    indices = gs.Variable(f"{name}_indices", dtype=np.int64, shape=[k])
    graph.nodes.append(
        gs.Node(
            op="TopK",
            name=name,
            inputs=[data, _const(f"{name}_k", [k])],
            outputs=[values, indices],
            attrs={"axis": 0, "largest": 1, "sorted": 1},
        )
    )
    return values, indices


def _cxcywh_to_xyxy(graph: gs.Graph, cxcywh):
    cx = _slice(graph, "box_cx", cxcywh, 0, 1, axis=2, dtype=np.float32)
    cy = _slice(graph, "box_cy", cxcywh, 1, 2, axis=2, dtype=np.float32)
    w = _slice(graph, "box_w", cxcywh, 2, 3, axis=2, dtype=np.float32)
    h = _slice(graph, "box_h", cxcywh, 3, 4, axis=2, dtype=np.float32)

    half = gs.Constant("half_const", np.array([0.5], dtype=np.float32))
    half_w = _binary(graph, "Mul", "half_w", w, half, dtype=np.float32)
    half_h = _binary(graph, "Mul", "half_h", h, half, dtype=np.float32)

    x1 = _binary(graph, "Sub", "x1", cx, half_w, dtype=np.float32)
    y1 = _binary(graph, "Sub", "y1", cy, half_h, dtype=np.float32)
    x2 = _binary(graph, "Add", "x2", cx, half_w, dtype=np.float32)
    y2 = _binary(graph, "Add", "y2", cy, half_h, dtype=np.float32)

    return _concat(
        graph,
        "boxes_xyxy",
        [x1, y1, x2, y2],
        axis=2,
        dtype=np.float32,
        shape=[1, None, 4],
    )


def _xyxy_to_yxyx(graph: gs.Graph, boxes_xyxy, prefix="nms"):
    x1 = _slice(graph, f"{prefix}_x1", boxes_xyxy, 0, 1, axis=2, dtype=np.float32)
    y1 = _slice(graph, f"{prefix}_y1", boxes_xyxy, 1, 2, axis=2, dtype=np.float32)
    x2 = _slice(graph, f"{prefix}_x2", boxes_xyxy, 2, 3, axis=2, dtype=np.float32)
    y2 = _slice(graph, f"{prefix}_y2", boxes_xyxy, 3, 4, axis=2, dtype=np.float32)
    return _concat(
        graph,
        f"{prefix}_boxes_yxyx",
        [y1, x1, y2, x2],
        axis=2,
        dtype=np.float32,
        shape=[1, None, 4],
    )


def _force_min_opset(model: onnx.ModelProto, min_opset: int = 13):
    found_default_domain = False
    for opset in model.opset_import:
        if opset.domain == "":
            found_default_domain = True
            if opset.version < min_opset:
                opset.version = min_opset

    if not found_default_domain:
        model.opset_import.append(onnx.helper.make_opsetid("", min_opset))

    return model


def build_openvino_fast_end2end_onnx(
    src_onnx: str,
    dst_onnx: str,
    *,
    num_classes: int = 38,
    num_kpts: int = 4,
    kp_dim: int = 2,
    pre_topk: int = 300,
    keep_topk: int = 50,
    score_threshold: float = 0.25,
    iou_threshold: float = 0.45,
    min_opset: int = 13,
):
    if pre_topk < keep_topk:
        raise ValueError("--pre-topk must be >= --keep-topk")

    graph = gs.import_onnx(onnx.load(src_onnx))
    graph.toposort()

    assert len(graph.outputs) == 1, "Expected single output from raw YOLO ONNX"

    raw = graph.outputs[0]
    expected_f = 4 + num_classes + num_kpts * kp_dim
    kp_channels = num_kpts * kp_dim

    # 1. [1, F, N] -> [1, N, F]
    pred = _transpose(
        graph,
        "pred_trans",
        raw,
        perm=[0, 2, 1],
        dtype=np.float32,
        shape=[1, None, expected_f],
    )

    # 2. Split.
    cxcywh = _slice(graph, "cxcywh", pred, 0, 4, axis=2, dtype=np.float32)
    scores = _slice(
        graph,
        "scores",
        pred,
        4,
        4 + num_classes,
        axis=2,
        dtype=np.float32,
        shape=[1, None, num_classes],
    )
    kpts = _slice(
        graph,
        "kpts",
        pred,
        4 + num_classes,
        expected_f,
        axis=2,
        dtype=np.float32,
        shape=[1, None, kp_channels],
    )

    boxes_xyxy = _cxcywh_to_xyxy(graph, cxcywh)

    # 3. One best class per anchor, then pre-TopK anchors.
    best_scores = _reduce_max(
        graph,
        "anchor_best_scores",
        scores,
        axes=[2],
        keepdims=0,
        dtype=np.float32,
        shape=[1, None],
    )
    best_classes_i64 = _argmax(
        graph,
        "anchor_best_classes_i64",
        scores,
        axis=2,
        keepdims=0,
        dtype=np.int64,
        shape=[1, None],
    )

    pre_scores, pre_indices = _topk_2d_axis1(
        graph,
        "pre_nms_anchor_topk",
        best_scores,
        pre_topk,
    )

    pre_indices_3d = _unsqueeze(
        graph,
        "pre_nms_indices_3d",
        pre_indices,
        axes=[2],
        dtype=np.int64,
        shape=[1, pre_topk, 1],
    )

    pre_box_indices = _expand(
        graph,
        "pre_nms_box_indices",
        pre_indices_3d,
        [1, pre_topk, 4],
        dtype=np.int64,
    )
    pre_kpt_indices = _expand(
        graph,
        "pre_nms_kpt_indices",
        pre_indices_3d,
        [1, pre_topk, kp_channels],
        dtype=np.int64,
    )

    pre_boxes = _gather_elements(
        graph,
        "pre_nms_boxes",
        boxes_xyxy,
        pre_box_indices,
        axis=1,
        dtype=np.float32,
        shape=[1, pre_topk, 4],
    )
    pre_kpts = _gather_elements(
        graph,
        "pre_nms_kpts",
        kpts,
        pre_kpt_indices,
        axis=1,
        dtype=np.float32,
        shape=[1, pre_topk, kp_channels],
    )
    pre_classes_i64 = _gather_elements(
        graph,
        "pre_nms_classes_i64",
        best_classes_i64,
        pre_indices,
        axis=1,
        dtype=np.int64,
        shape=[1, pre_topk],
    )
    pre_classes_i32 = _cast(
        graph,
        "pre_nms_classes",
        pre_classes_i64,
        to_type=TensorProto.INT32,
        dtype=np.int32,
        shape=[1, pre_topk],
    )

    # 4. Class-agnostic NMS on only pre_topk boxes.
    nms_boxes = _xyxy_to_yxyx(graph, pre_boxes, prefix="fast_nms")
    nms_scores = _unsqueeze(
        graph,
        "fast_nms_scores",
        pre_scores,
        axes=[1],
        dtype=np.float32,
        shape=[1, 1, pre_topk],
    )

    selected_indices = gs.Variable("selected_indices", dtype=np.int64, shape=[None, 3])
    graph.nodes.append(
        gs.Node(
            op="NonMaxSuppression",
            name="Fast_ClassAgnostic_NMS",
            inputs=[
                nms_boxes,
                nms_scores,
                _scalar_const("max_output_boxes_per_class", keep_topk, np.int64),
                _scalar_const("iou_threshold", iou_threshold, np.float32),
                _scalar_const("score_threshold", score_threshold, np.float32),
            ],
            outputs=[selected_indices],
            attrs={"center_point_box": 0},
        )
    )

    # selected_indices: [M, 3] = [batch, class(0), pre_topk_box_index].
    selected_batch_raw = _slice(
        graph,
        "selected_batch_raw",
        selected_indices,
        0,
        1,
        axis=1,
        dtype=np.int64,
        shape=[None, 1],
    )
    valid_mask_2d = _binary(
        graph,
        "GreaterOrEqual",
        "valid_nms_row_mask_2d",
        selected_batch_raw,
        _scalar_const("zero_i64_for_valid_mask", 0, np.int64),
        dtype=np.bool_,
        shape=[None, 1],
    )
    valid_mask_1d = _squeeze(
        graph,
        "valid_nms_row_mask",
        valid_mask_2d,
        axes=[1],
        dtype=np.bool_,
        shape=[None],
    )

    selected_indices_safe = _binary(
        graph,
        "Max",
        "selected_indices_safe",
        selected_indices,
        _scalar_const("zero_i64_for_clamp", 0, np.int64),
        dtype=np.int64,
        shape=[None, 3],
    )

    batch_idx = _slice(
        graph,
        "selected_batch_idx",
        selected_indices_safe,
        0,
        1,
        axis=1,
        dtype=np.int64,
        shape=[None, 1],
    )
    box_idx = _slice(
        graph,
        "selected_box_idx",
        selected_indices_safe,
        2,
        3,
        axis=1,
        dtype=np.int64,
        shape=[None, 1],
    )
    batch_box_idx = _concat(
        graph,
        "selected_batch_box_idx",
        [batch_idx, box_idx],
        axis=1,
        dtype=np.int64,
        shape=[None, 2],
    )

    det_boxes_2d = _gathernd(
        graph,
        "det_boxes_2d_raw",
        pre_boxes,
        batch_box_idx,
        dtype=np.float32,
        shape=[None, 4],
    )
    det_scores_1d_raw = _gathernd(
        graph,
        "det_scores_1d_raw",
        pre_scores,
        batch_box_idx,
        dtype=np.float32,
        shape=[None],
    )
    det_classes_i32_raw = _gathernd(
        graph,
        "det_classes_i32_raw",
        pre_classes_i32,
        batch_box_idx,
        dtype=np.int32,
        shape=[None],
    )
    det_kpts_2d = _gathernd(
        graph,
        "det_kpts_2d_raw",
        pre_kpts,
        batch_box_idx,
        dtype=np.float32,
        shape=[None, kp_channels],
    )

    det_scores_1d = _where(
        graph,
        "det_scores_1d_validated",
        valid_mask_1d,
        det_scores_1d_raw,
        _scalar_const("invalid_score_const", -1.0, np.float32),
        dtype=np.float32,
        shape=[None],
    )
    det_classes_i32 = _where(
        graph,
        "det_classes_i32_validated",
        valid_mask_1d,
        det_classes_i32_raw,
        _scalar_const("invalid_class_const", -1, np.int32),
        dtype=np.int32,
        shape=[None],
    )

    # 5. Pad to fixed keep_topk and sort by score.
    pad_scores = gs.Constant("pad_scores", np.full([keep_topk], -1.0, dtype=np.float32))
    pad_classes = gs.Constant("pad_classes", np.full([keep_topk], -1, dtype=np.int32))
    pad_boxes = gs.Constant("pad_boxes", np.zeros([keep_topk, 4], dtype=np.float32))
    pad_kpts = gs.Constant(
        "pad_kpts", np.zeros([keep_topk, kp_channels], dtype=np.float32)
    )

    scores_padded = _concat(
        graph,
        "det_scores_padded",
        [det_scores_1d, pad_scores],
        axis=0,
        dtype=np.float32,
        shape=[None],
    )
    classes_padded = _concat(
        graph,
        "det_classes_padded",
        [det_classes_i32, pad_classes],
        axis=0,
        dtype=np.int32,
        shape=[None],
    )
    boxes_padded = _concat(
        graph,
        "det_boxes_padded",
        [det_boxes_2d, pad_boxes],
        axis=0,
        dtype=np.float32,
        shape=[None, 4],
    )
    kpts_padded = _concat(
        graph,
        "det_kpts_padded",
        [det_kpts_2d, pad_kpts],
        axis=0,
        dtype=np.float32,
        shape=[None, kp_channels],
    )

    topk_scores, topk_indices = _topk_1d(
        graph,
        "global_score_topk",
        scores_padded,
        keep_topk,
    )

    topk_boxes_2d = _gather(
        graph,
        "det_boxes_topk_2d",
        boxes_padded,
        topk_indices,
        axis=0,
        dtype=np.float32,
        shape=[keep_topk, 4],
    )
    topk_classes_1d = _gather(
        graph,
        "det_classes_topk_1d",
        classes_padded,
        topk_indices,
        axis=0,
        dtype=np.int32,
        shape=[keep_topk],
    )
    topk_kpts_2d = _gather(
        graph,
        "det_kpts_topk_2d",
        kpts_padded,
        topk_indices,
        axis=0,
        dtype=np.float32,
        shape=[keep_topk, kp_channels],
    )

    topk_valid_mask = _binary(
        graph,
        "GreaterOrEqual",
        "topk_valid_mask",
        topk_scores,
        _scalar_const("zero_f32_for_topk_valid", 0.0, np.float32),
        dtype=np.bool_,
        shape=[keep_topk],
    )
    topk_valid_i32 = _cast(
        graph,
        "topk_valid_i32",
        topk_valid_mask,
        to_type=TensorProto.INT32,
        dtype=np.int32,
        shape=[keep_topk],
    )
    num_dets_scalar = _reduce_sum(
        graph,
        "num_dets_scalar",
        topk_valid_i32,
        axes=[0],
        keepdims=0,
        dtype=np.int32,
        shape=[],
    )
    num_dets = _unsqueeze(
        graph,
        "num_dets",
        num_dets_scalar,
        axes=[0, 1],
        dtype=np.int32,
        shape=[1, 1],
    )

    det_boxes = _unsqueeze(
        graph,
        "det_boxes",
        topk_boxes_2d,
        axes=[0],
        dtype=np.float32,
        shape=[1, keep_topk, 4],
    )
    det_scores = _unsqueeze(
        graph,
        "det_scores",
        topk_scores,
        axes=[0],
        dtype=np.float32,
        shape=[1, keep_topk],
    )
    det_classes = _unsqueeze(
        graph,
        "det_classes",
        topk_classes_1d,
        axes=[0],
        dtype=np.int32,
        shape=[1, keep_topk],
    )
    det_kpts = _unsqueeze(
        graph,
        "det_kpts",
        topk_kpts_2d,
        axes=[0],
        dtype=np.float32,
        shape=[1, keep_topk, kp_channels],
    )

    graph.outputs = [num_dets, det_boxes, det_scores, det_classes, det_kpts]

    graph.cleanup().toposort()

    model = gs.export_onnx(graph)
    model = _force_min_opset(model, min_opset=min_opset)

    onnx.checker.check_model(model)
    onnx.save(model, dst_onnx)

    print(f"[onnx] saved iGPU-friendly OpenVINO e2e ONNX: {dst_onnx}")
    print("[onnx] outputs:")
    print("  num_dets    [1, 1] int32")
    print(f"  det_boxes   [1, {keep_topk}, 4] float32")
    print(f"  det_scores  [1, {keep_topk}] float32")
    print(f"  det_classes [1, {keep_topk}] int32")
    print(f"  det_kpts    [1, {keep_topk}, {kp_channels}] float32")
    print("")
    print(f"[note] pre_topk={pre_topk}, keep_topk={keep_topk}")
    print("[note] fast path uses best-class-per-anchor + class-agnostic NMS.")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--src", required=True, help="Original raw YOLO11-pose ONNX")
    parser.add_argument(
        "--dst-onnx",
        default="yolo11_openvino_fast_end2end.onnx",
        help="Output iGPU-friendly OpenVINO ONNX",
    )

    parser.add_argument("--num-classes", type=int, default=38)
    parser.add_argument("--num-kpts", type=int, default=4)
    parser.add_argument("--kp-dim", type=int, default=2, choices=[2, 3])

    parser.add_argument(
        "--pre-topk",
        type=int,
        default=300,
        help="Number of best anchors kept before NMS. Lower is faster; higher is safer.",
    )
    parser.add_argument("--keep-topk", type=int, default=50)
    parser.add_argument("--score-thr", type=float, default=0.25)
    parser.add_argument("--iou-thr", type=float, default=0.45)

    parser.add_argument(
        "--min-opset",
        type=int,
        default=13,
        help="Minimum ONNX opset. Keep >=13 for axes-as-input ops.",
    )

    args = parser.parse_args()

    build_openvino_fast_end2end_onnx(
        src_onnx=args.src,
        dst_onnx=args.dst_onnx,
        num_classes=args.num_classes,
        num_kpts=args.num_kpts,
        kp_dim=args.kp_dim,
        pre_topk=args.pre_topk,
        keep_topk=args.keep_topk,
        score_threshold=args.score_thr,
        iou_threshold=args.iou_thr,
        min_opset=args.min_opset,
    )


if __name__ == "__main__":
    main()
