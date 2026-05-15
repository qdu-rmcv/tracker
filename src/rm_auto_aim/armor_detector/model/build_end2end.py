"""
Build TensorRT-friendly end-to-end YOLO11-pose ONNX.

This script only depends on:
  - onnx
  - onnx_graphsurgeon
  - numpy

It does NOT depend on TensorRT.

Pipeline:
  YOLO head [1, F, 8400]
    -> Transpose [1, 8400, F]
    -> Slice: cxcywh / scores / kpts
    -> cxcywh -> xyxy
    -> EfficientNMS_TRT
    -> L1 match det_boxes <-> all xyxy anchors
    -> GatherElements kpts
    -> ONNX outputs:
         num_dets     [1, 1]                 int32
         det_boxes    [1, keep_topk, 4]      float32, xyxy
         det_scores   [1, keep_topk]         float32
         det_classes  [1, keep_topk]         int32
         det_kpts     [1, keep_topk, K*D]    float32
"""

from __future__ import annotations

import argparse

import numpy as np
import onnx
import onnx_graphsurgeon as gs


# =============================================================================
# Graph helpers
# =============================================================================


def _const(name, arr, dtype=np.int64):
    return gs.Constant(name, np.array(arr, dtype=dtype))


def _slice(graph, name, inp, start, end, axis=2, dtype=np.float32):
    starts = _const(f"{name}_starts", [start])
    ends = _const(f"{name}_ends", [end])
    axes = _const(f"{name}_axes", [axis])

    out = gs.Variable(name, dtype=dtype)

    graph.nodes.append(
        gs.Node(
            op="Slice",
            inputs=[inp, starts, ends, axes],
            outputs=[out],
        )
    )

    return out


def _unsqueeze(graph, name, inp, axes, dtype=None):
    out = gs.Variable(name, dtype=dtype or inp.dtype)
    axes_const = _const(f"{name}_axes", axes)

    graph.nodes.append(
        gs.Node(
            op="Unsqueeze",
            inputs=[inp, axes_const],
            outputs=[out],
        )
    )

    return out


def _binary(graph, op, name, a, b, dtype=np.float32):
    out = gs.Variable(name, dtype=dtype)

    graph.nodes.append(
        gs.Node(
            op=op,
            inputs=[a, b],
            outputs=[out],
        )
    )

    return out


def _cxcywh_to_xyxy(graph, cxcywh):
    cx = _slice(graph, "box_cx", cxcywh, 0, 1)
    cy = _slice(graph, "box_cy", cxcywh, 1, 2)
    w = _slice(graph, "box_w", cxcywh, 2, 3)
    h = _slice(graph, "box_h", cxcywh, 3, 4)

    half = gs.Constant("half_const", np.array([0.5], dtype=np.float32))

    half_w = _binary(graph, "Mul", "half_w", w, half)
    half_h = _binary(graph, "Mul", "half_h", h, half)

    x1 = _binary(graph, "Sub", "x1", cx, half_w)
    y1 = _binary(graph, "Sub", "y1", cy, half_h)
    x2 = _binary(graph, "Add", "x2", cx, half_w)
    y2 = _binary(graph, "Add", "y2", cy, half_h)

    xyxy = gs.Variable("boxes_xyxy", dtype=np.float32)

    graph.nodes.append(
        gs.Node(
            op="Concat",
            inputs=[x1, y1, x2, y2],
            outputs=[xyxy],
            attrs={"axis": 2},
        )
    )

    return xyxy


def _recover_indices_from_boxes(graph, det_boxes, boxes_xyxy):
    """
    det_boxes:  [1, keep_topk, 4]
    boxes_xyxy: [1, N, 4]

    Return:
      topk_indices [1, keep_topk, 1] int64

    Logic:
      For each selected det_box, compare with all original boxes_xyxy.
      Use L1 distance to recover the best-matched original anchor index.
    """
    # [1, keep_topk, 1, 4]
    det_exp = _unsqueeze(
        graph,
        "det_boxes_exp",
        det_boxes,
        axes=[2],
        dtype=np.float32,
    )

    # [1, 1, N, 4]
    all_exp = _unsqueeze(
        graph,
        "all_boxes_exp",
        boxes_xyxy,
        axes=[1],
        dtype=np.float32,
    )

    diff = _binary(
        graph,
        "Sub",
        "box_match_diff",
        det_exp,
        all_exp,
    )

    abs_diff = gs.Variable("box_match_abs", dtype=np.float32)

    graph.nodes.append(
        gs.Node(
            op="Abs",
            inputs=[diff],
            outputs=[abs_diff],
        )
    )

    dist = gs.Variable("box_match_l1", dtype=np.float32)

    graph.nodes.append(
        gs.Node(
            op="ReduceSum",
            inputs=[
                abs_diff,
                _const("box_match_reduce_axes", [3]),
            ],
            outputs=[dist],
            attrs={"keepdims": 0},
        )
    )
    # dist: [1, keep_topk, N]

    topk_values = gs.Variable("box_match_min_dist", dtype=np.float32)
    topk_indices = gs.Variable("box_match_indices", dtype=np.int64)

    graph.nodes.append(
        gs.Node(
            op="TopK",
            inputs=[
                dist,
                _const("box_match_topk_k", [1]),
            ],
            outputs=[topk_values, topk_indices],
            attrs={
                "axis": 2,
                "largest": 0,
                "sorted": 0,
            },
        )
    )
    # topk_indices: [1, keep_topk, 1]

    return topk_indices


def _expand_gather_indices(graph, idx, keep_topk, last_dim):
    """
    idx: [1, keep_topk, 1]
      -> [1, keep_topk, last_dim]
    """
    shape_const = gs.Constant(
        "kpt_gather_idx_shape",
        np.array([1, keep_topk, last_dim], dtype=np.int64),
    )

    out = gs.Variable(
        "kpt_gather_idx_expand",
        dtype=np.int64,
        shape=[1, keep_topk, last_dim],
    )

    graph.nodes.append(
        gs.Node(
            op="Expand",
            inputs=[idx, shape_const],
            outputs=[out],
        )
    )

    return out


def _force_min_opset(model: onnx.ModelProto, min_opset: int = 13):
    """
    Unsqueeze / ReduceSum with axes as input require newer ONNX opsets.
    Keep original opset if it is already >= min_opset.
    """
    found_default_domain = False

    for opset in model.opset_import:
        if opset.domain == "":
            found_default_domain = True
            if opset.version < min_opset:
                opset.version = min_opset

    if not found_default_domain:
        model.opset_import.append(onnx.helper.make_opsetid("", min_opset))

    return model


# =============================================================================
# ONNX surgery
# =============================================================================


def build_end2end_onnx(
    src_onnx: str,
    dst_onnx: str,
    *,
    num_classes: int = 38,
    num_kpts: int = 4,
    kp_dim: int = 2,
    keep_topk: int = 50,
    score_threshold: float = 0.25,
    iou_threshold: float = 0.45,
    min_opset: int = 13,
):
    graph = gs.import_onnx(onnx.load(src_onnx))
    graph.toposort()

    assert len(graph.outputs) == 1, "Expected single output from raw YOLO ONNX"

    raw = graph.outputs[0]
    expected_f = 4 + num_classes + num_kpts * kp_dim
    kp_channels = num_kpts * kp_dim

    # 1. Transpose [1, F, N] -> [1, N, F]
    trans = gs.Variable("pred_trans", dtype=np.float32)

    graph.nodes.append(
        gs.Node(
            op="Transpose",
            inputs=[raw],
            outputs=[trans],
            attrs={"perm": [0, 2, 1]},
        )
    )

    # 2. Slice
    cxcywh = _slice(graph, "cxcywh", trans, 0, 4)
    scores = _slice(graph, "scores", trans, 4, 4 + num_classes)
    kpts = _slice(graph, "kpts", trans, 4 + num_classes, expected_f)

    # 3. cxcywh -> xyxy
    boxes_xyxy = _cxcywh_to_xyxy(graph, cxcywh)

    # 4. EfficientNMS_TRT
    nms_num = gs.Variable(
        "num_dets",
        dtype=np.int32,
        shape=[1, 1],
    )

    nms_boxes = gs.Variable(
        "det_boxes",
        dtype=np.float32,
        shape=[1, keep_topk, 4],
    )

    nms_scores = gs.Variable(
        "det_scores",
        dtype=np.float32,
        shape=[1, keep_topk],
    )

    nms_classes = gs.Variable(
        "det_classes",
        dtype=np.int32,
        shape=[1, keep_topk],
    )

    graph.nodes.append(
        gs.Node(
            op="EfficientNMS_TRT",
            name="EfficientNMS",
            inputs=[boxes_xyxy, scores],
            outputs=[
                nms_num,
                nms_boxes,
                nms_scores,
                nms_classes,
            ],
            attrs={
                "plugin_version": "1",
                "background_class": -1,
                "max_output_boxes": keep_topk,
                "score_threshold": float(score_threshold),
                "iou_threshold": float(iou_threshold),
                "score_activation": False,
                "box_coding": 0,  # xyxy
                "class_agnostic": False,
            },
        )
    )

    # 5. Recover selected anchor indices on GPU
    match_idx = _recover_indices_from_boxes(
        graph=graph,
        det_boxes=nms_boxes,
        boxes_xyxy=boxes_xyxy,
    )

    # 6. Gather keypoints
    gather_idx = _expand_gather_indices(
        graph=graph,
        idx=match_idx,
        keep_topk=keep_topk,
        last_dim=kp_channels,
    )

    det_kpts = gs.Variable(
        "det_kpts",
        dtype=np.float32,
        shape=[1, keep_topk, kp_channels],
    )

    graph.nodes.append(
        gs.Node(
            op="GatherElements",
            inputs=[kpts, gather_idx],
            outputs=[det_kpts],
            attrs={"axis": 1},
        )
    )

    # 7. Set outputs
    graph.outputs = [
        nms_num,
        nms_boxes,
        nms_scores,
        nms_classes,
        det_kpts,
    ]

    graph.cleanup().toposort()

    model = gs.export_onnx(graph)
    model = _force_min_opset(model, min_opset=min_opset)

    onnx.save(model, dst_onnx)

    print(f"[onnx] saved end2end ONNX: {dst_onnx}")
    print(f"[onnx] outputs:")
    print(f"  num_dets    [1, 1] int32")
    print(f"  det_boxes   [1, {keep_topk}, 4] float32")
    print(f"  det_scores  [1, {keep_topk}] float32")
    print(f"  det_classes [1, {keep_topk}] int32")
    print(f"  det_kpts    [1, {keep_topk}, {kp_channels}] float32")


# =============================================================================
# CLI
# =============================================================================


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--src",
        required=True,
        help="Original YOLO11-pose ONNX",
    )

    parser.add_argument(
        "--dst-onnx",
        default="yolo11_end2end.onnx",
        help="Output end2end ONNX",
    )

    parser.add_argument("--num-classes", type=int, default=38)
    parser.add_argument("--num-kpts", type=int, default=4)
    parser.add_argument("--kp-dim", type=int, default=2, choices=[2, 3])

    parser.add_argument("--keep-topk", type=int, default=50)
    parser.add_argument("--score-thr", type=float, default=0.25)
    parser.add_argument("--iou-thr", type=float, default=0.45)

    parser.add_argument(
        "--min-opset",
        type=int,
        default=13,
        help="Minimum ONNX opset. Keep >=13 for Unsqueeze/ReduceSum axes inputs.",
    )

    args = parser.parse_args()

    build_end2end_onnx(
        src_onnx=args.src,
        dst_onnx=args.dst_onnx,
        num_classes=args.num_classes,
        num_kpts=args.num_kpts,
        kp_dim=args.kp_dim,
        keep_topk=args.keep_topk,
        score_threshold=args.score_thr,
        iou_threshold=args.iou_thr,
        min_opset=args.min_opset,
    )


if __name__ == "__main__":
    main()
