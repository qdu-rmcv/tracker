#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ONNX 修复 + TensorRT 就绪化 一体流水线

流程:
    raw.onnx
        --> patched.onnx   (修复已知 Ends1 bug)
        --> clean.onnx     (graphsurgeon cleanup / fold)
        --> trt_ready.onnx (修复 MaxPool / QDQ / Slice)

--stage:
    all : 默认
    fix : 只修复静态检查问题，生成 clean.onnx (此时 --input 指向原始 onnx)
    trt : 只进行 TensorRT 兼容化 (此时 --input 应指向已经 fix 过的 onnx)
"""

import argparse
from pathlib import Path

import numpy as np
import onnx
from onnx import numpy_helper, shape_inference
import onnxruntime as ort

try:
    import onnx_graphsurgeon as gs
except ImportError as e:
    raise ImportError(
        "缺少 onnx-graphsurgeon, 请先安装: pip install -U onnx-graphsurgeon"
    ) from e


# ============================================================
# constants
# ============================================================
TARGET_ENDS1 = "__module.model.23/prim::ListUnpack/VariadicSplit/Ends1"
DEFAULT_INPUT_SHAPE = [1, 3, 640, 640]
WATCH_SHAPES = {
    "__module.model.23/aten::unsqueeze/Unsqueeze_output0",
    "__module.model.23/prim::ListUnpack/VariadicSplit_output4",
    "__module.model.23/aten::add/Add_output2",
}


def log(msg: str):
    print(msg, flush=True)


# ============================================================
# shared helpers
# ============================================================
def shape_of(v):
    tt = v.type.tensor_type
    if not tt.HasField("shape"):
        return None
    dims = []
    for d in tt.shape.dim:
        if d.HasField("dim_value"):
            dims.append(d.dim_value)
        elif d.HasField("dim_param"):
            dims.append(d.dim_param)
        else:
            dims.append("?")
    return dims


def print_key_shapes(model_path: str, title: str):
    log(title)
    inferred = shape_inference.infer_shapes(onnx.load(model_path))
    for v in (
        list(inferred.graph.value_info)
        + list(inferred.graph.input)
        + list(inferred.graph.output)
    ):
        if v.name in WATCH_SHAPES:
            log(f"  {v.name} {shape_of(v)}")


def _build_default_input(input_meta):
    shape = []
    for idx, dim in enumerate(input_meta.shape):
        if isinstance(dim, int) and dim > 0:
            shape.append(dim)
        else:
            shape.append(
                DEFAULT_INPUT_SHAPE[idx] if idx < len(DEFAULT_INPUT_SHAPE) else 1
            )
    return np.random.rand(*shape).astype(np.float32)


def validate(model_path: str, do_ort: bool = True, do_smoke: bool = True):
    log(f"Validation: {model_path}")

    model = onnx.load(model_path)
    onnx.checker.check_model(model, full_check=False)
    log("[OK] basic check")

    inferred = shape_inference.infer_shapes(model)
    log("[OK] infer_shapes")
    for v in list(inferred.graph.input) + list(inferred.graph.output):
        log(f"  {v.name} {shape_of(v)}")

    if not do_ort:
        return

    sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    log("[OK] ORT load")
    log(f"  inputs : {[(i.name, i.shape, i.type) for i in sess.get_inputs()]}")
    log(f"  outputs: {[(o.name, o.shape, o.type) for o in sess.get_outputs()]}")

    if not do_smoke:
        return

    input_meta = sess.get_inputs()[0]
    x = _build_default_input(input_meta)
    outputs = sess.run(None, {input_meta.name: x})
    log("[OK] smoke run")
    if outputs:
        first = outputs[0]
        if isinstance(first, np.ndarray):
            log(f"  output shape: {first.shape}")
            log(f"  has nan: {bool(np.isnan(first).any())}")
            log(f"  has inf: {bool(np.isinf(first).any())}")


def compare_models(model_a: str, model_b: str):
    log(f"Compare outputs: {model_a} vs {model_b}")
    sess_a = ort.InferenceSession(model_a, providers=["CPUExecutionProvider"])
    sess_b = ort.InferenceSession(model_b, providers=["CPUExecutionProvider"])
    ia = sess_a.get_inputs()[0]
    ib = sess_b.get_inputs()[0]
    x = _build_default_input(ia)
    y_a = sess_a.run(None, {ia.name: x})[0]
    y_b = sess_b.run(None, {ib.name: x})[0]
    log(f"  max  abs diff: {float(np.max(np.abs(y_a - y_b)))}")
    log(f"  mean abs diff: {float(np.mean(np.abs(y_a - y_b)))}")


# ============================================================
# stage: fix_and_validate
# ============================================================
def patch_known_bug(model: onnx.ModelProto) -> bool:
    patched = False
    for node in model.graph.node:
        if node.name == TARGET_ENDS1 and node.op_type == "Constant":
            for attr in node.attribute:
                if attr.name == "value":
                    old_arr = numpy_helper.to_array(attr.t)
                    if old_arr.tolist() == [1]:
                        attr.t.CopyFrom(
                            numpy_helper.from_array(np.array([4], dtype=np.int64))
                        )
                        patched = True
                    break
    return patched


def clean_with_graphsurgeon(
    model: onnx.ModelProto,
    do_fold_constants: bool,
    partitioning: str,
) -> onnx.ModelProto:
    graph = gs.import_onnx(model)
    graph.cleanup(remove_unused_graph_inputs=False).toposort()
    if do_fold_constants:
        log(f"GraphSurgeon: fold_constants(partitioning={partitioning!r})")
        graph.fold_constants(partitioning=partitioning)
        graph.cleanup(remove_unused_graph_inputs=False).toposort()
    else:
        log("GraphSurgeon: cleanup + toposort only (fold_constants disabled)")
    return gs.export_onnx(graph)


def run_fix_stage(
    src: Path,
    patched_path: Path,
    clean_path: Path,
    do_fold: bool,
    fold_partitioning: str,
    do_ort_check: bool,
    do_compare: bool,
) -> Path:
    """patch + graphsurgeon cleanup, 返回 clean onnx 路径."""
    log(f"\n========== [fix] stage start ==========")
    log(f"[fix] load raw: {src}")
    model = onnx.load(str(src))

    if patch_known_bug(model):
        log("[fix] Patched Ends1: [1] -> [4]")
    else:
        log("[fix] No patch applied (already fixed or target node missing).")

    onnx.save(model, str(patched_path))
    log(f"[fix] saved patched: {patched_path}")
    print_key_shapes(str(patched_path), "[fix] key shapes after patch:")
    validate(str(patched_path), do_ort=do_ort_check)

    clean_model = clean_with_graphsurgeon(
        onnx.load(str(patched_path)),
        do_fold_constants=do_fold,
        partitioning=fold_partitioning,
    )
    clean_model = shape_inference.infer_shapes(clean_model)
    onnx.save(clean_model, str(clean_path))
    log(f"[fix] saved clean: {clean_path}")
    print_key_shapes(str(clean_path), "[fix] key shapes after cleanup:")
    validate(str(clean_path), do_ort=do_ort_check)

    if do_compare and do_ort_check:
        compare_models(str(patched_path), str(clean_path))

    log(f"========== [fix] stage done ===========\n")
    return clean_path


# ============================================================
# stage: fix_trt
# ============================================================
def is_const_tensor(t):
    return isinstance(t, gs.Constant)


def is_missing_tensor(t):
    if t is None:
        return True
    name = getattr(t, "name", None)
    return name is None or name == ""


def replace_tensor_everywhere(graph: gs.Graph, old_tensor, new_tensor):
    consumers = list(old_tensor.outputs)
    for node in consumers:
        node.inputs = [new_tensor if x is old_tensor else x for x in node.inputs]
    graph.outputs = [new_tensor if x is old_tensor else x for x in graph.outputs]


def get_const_array(tensor):
    if tensor is None or is_missing_tensor(tensor):
        return None
    if isinstance(tensor, gs.Constant):
        return np.asarray(tensor.values)
    producers = getattr(tensor, "inputs", None)
    if producers and len(producers) == 1:
        node = producers[0]
        if node.op == "Constant":
            value = node.attrs.get("value", None)
            if value is None:
                return None
            if isinstance(value, gs.Constant):
                return np.asarray(value.values)
            if hasattr(value, "values"):
                return np.asarray(value.values)
            return np.asarray(value)
    return None


def make_const(name: str, values, dtype=np.int64):
    return gs.Constant(name=name, values=np.asarray(values, dtype=dtype))


def drop_maxpool_indices_outputs(graph: gs.Graph):
    patched, skipped = 0, 0
    for node in graph.nodes:
        if node.op != "MaxPool":
            continue
        if len(node.outputs) <= 1:
            continue
        idx_out = node.outputs[1]
        if len(idx_out.outputs) > 0 or idx_out in graph.outputs:
            log(f"[MaxPool] skip (second output still used): {node.name}")
            skipped += 1
            continue
        log(f"[MaxPool] drop extra outputs: {node.name}")
        node.outputs = [node.outputs[0]]
        patched += 1
    log(f"[MaxPool] patched={patched}, skipped={skipped}")


def _reshape_qparam(param, x_shape, axis=None):
    arr = np.asarray(param)
    if arr.ndim == 0:
        return arr
    if arr.ndim == 1:
        rank = len(x_shape)
        if axis is None:
            hits = [i for i, d in enumerate(x_shape) if d == arr.shape[0]]
            axis = hits[0] if len(hits) == 1 else 0
        if axis < 0:
            axis += rank
        if not (0 <= axis < rank):
            raise ValueError(f"invalid axis={axis} for x_shape={x_shape}")
        if x_shape[axis] != arr.shape[0]:
            raise ValueError(
                f"1-D qparam length {arr.shape[0]} does not match "
                f"x_shape[{axis}]={x_shape[axis]} for x_shape={x_shape}"
            )
        shape = [1] * rank
        shape[axis] = arr.shape[0]
        return arr.reshape(shape)
    if tuple(arr.shape) == tuple(x_shape):
        return arr
    raise ValueError(
        f"unsupported qparam shape {arr.shape} for x_shape={x_shape}, axis={axis}"
    )


def dequantize_constant(q, s, z=None, axis=None):
    q = np.asarray(q)
    s = np.asarray(s, dtype=np.float32)
    if z is None:
        z = np.zeros_like(s, dtype=q.dtype)
    else:
        z = np.asarray(z, dtype=q.dtype)
    s = _reshape_qparam(s, q.shape, axis)
    z = _reshape_qparam(z, q.shape, axis)
    return (q.astype(np.float32) - z.astype(np.float32)) * s.astype(np.float32)


def strip_qdq(graph: gs.Graph, verbose: bool = False):
    const_dq = qdq_pairs = bypass_q = bypass_dq = skipped = 0

    for dq_node in list(graph.nodes):
        if dq_node.op != "DequantizeLinear":
            continue
        if len(dq_node.inputs) < 2 or len(dq_node.outputs) != 1:
            if verbose:
                log(f"[DQ] skip malformed node: {dq_node.name}")
            skipped += 1
            continue

        x = dq_node.inputs[0]
        scale = dq_node.inputs[1]
        zero_point = dq_node.inputs[2] if len(dq_node.inputs) >= 3 else None
        axis = dq_node.attrs.get("axis", None)
        dq_out = dq_node.outputs[0]

        if (
            is_const_tensor(x)
            and is_const_tensor(scale)
            and (zero_point is None or is_const_tensor(zero_point))
        ):
            q = x.values
            s = scale.values
            z = None if zero_point is None else zero_point.values
            if verbose:
                log(
                    f"[DQ-const] {dq_node.name}: "
                    f"q={np.shape(q)}, s={np.shape(s)}, "
                    f"z={None if z is None else np.shape(z)}, axis={axis}"
                )
            fp = dequantize_constant(q, s, z, axis=axis).astype(np.float32)
            new_const = gs.Constant(
                name=(dq_out.name or dq_node.name or "dq_const") + "_fp32",
                values=fp,
            )
            replace_tensor_everywhere(graph, dq_out, new_const)
            dq_node.inputs = []
            dq_node.outputs = []
            const_dq += 1
            continue

        if hasattr(x, "inputs") and len(x.inputs) == 1:
            q_node = x.inputs[0]
            if q_node.op == "QuantizeLinear" and len(q_node.inputs) >= 1:
                src_tensor = q_node.inputs[0]
                if verbose:
                    log(
                        f"[QDQ-bypass] dq={dq_node.name}, q={q_node.name}, "
                        f"src={src_tensor.name}, axis={axis}"
                    )
                replace_tensor_everywhere(graph, dq_out, src_tensor)
                dq_node.inputs = []
                dq_node.outputs = []
                bypass_dq += 1
                if len(x.outputs) == 0 and x not in graph.outputs:
                    q_node.inputs = []
                    q_node.outputs = []
                    bypass_q += 1
                qdq_pairs += 1
                continue

        if verbose:
            log(f"[DQ] skip unsupported pattern: {dq_node.name}")
        skipped += 1

    log(
        f"[QDQ] const_dq={const_dq}, qdq_pairs={qdq_pairs}, "
        f"bypass_q={bypass_q}, bypass_dq={bypass_dq}, skipped={skipped}"
    )


def normalize_slice_nodes(graph: gs.Graph, verbose: bool = False):
    patched_axes = trimmed_trailing_empty = skipped = 0

    for node in graph.nodes:
        if node.op != "Slice":
            continue
        if verbose:
            log(f"[Slice] inspect: {node.name}, n_inputs={len(node.inputs)}")

        while len(node.inputs) > 3 and is_missing_tensor(node.inputs[-1]):
            if verbose:
                log(f"[Slice] trim trailing empty input: {node.name}")
            node.inputs = node.inputs[:-1]
            trimmed_trailing_empty += 1

        if (
            len(node.inputs) >= 5
            and is_missing_tensor(node.inputs[3])
            and not is_missing_tensor(node.inputs[4])
        ):
            starts_arr = get_const_array(node.inputs[1])
            ends_arr = get_const_array(node.inputs[2])
            steps_arr = get_const_array(node.inputs[4])
            n = None
            for arr in (starts_arr, ends_arr, steps_arr):
                if arr is not None:
                    arr = np.asarray(arr)
                    n = 1 if arr.ndim == 0 else int(arr.size)
                    break
            if n is None:
                log(f"[Slice] skip (cannot infer missing axes): {node.name}")
                skipped += 1
                continue
            axes = np.arange(n, dtype=np.int64)
            axes_const = make_const(f"{node.name}_axes_fix", axes, dtype=np.int64)
            node.inputs[3] = axes_const
            patched_axes += 1
            if verbose:
                log(
                    f"[Slice] patched missing axes: {node.name}, "
                    f"starts={None if starts_arr is None else starts_arr.tolist()}, "
                    f"ends={None if ends_arr is None else ends_arr.tolist()}, "
                    f"steps={None if steps_arr is None else steps_arr.tolist()}, "
                    f"axes={axes.tolist()}"
                )
        elif len(node.inputs) == 4 and is_missing_tensor(node.inputs[3]):
            node.inputs = node.inputs[:3]
            trimmed_trailing_empty += 1
            if verbose:
                log(f"[Slice] removed empty axes placeholder: {node.name}")

    log(
        f"[Slice] patched_axes={patched_axes}, "
        f"trimmed_trailing_empty={trimmed_trailing_empty}, skipped={skipped}"
    )


def run_trt_stage(
    src: Path,
    dst: Path,
    do_ort_check: bool,
    verbose: bool,
):
    log(f"\n========== [trt] stage start ==========")
    log(f"[trt] load: {src}")
    graph = gs.import_onnx(onnx.load(str(src)))

    drop_maxpool_indices_outputs(graph)
    strip_qdq(graph, verbose=verbose)
    normalize_slice_nodes(graph, verbose=verbose)

    graph.cleanup().toposort()
    onnx.save(gs.export_onnx(graph), str(dst))
    log(f"[trt] saved: {dst}")

    validate(str(dst), do_ort=do_ort_check, do_smoke=False)
    log(f"========== [trt] stage done ===========\n")


# ============================================================
# CLI
# ============================================================
def parse_args():
    p = argparse.ArgumentParser(
        description="ONNX 修复 + TensorRT 就绪化 一体流水线",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # --- 输入输出 ---
    p.add_argument(
        "-i",
        "--input",
        default="yolo11_raw.onnx",
        help="输入 ONNX (stage=all/fix 用原始模型; stage=trt 用已 fix 过的模型)",
    )
    p.add_argument(
        "-o",
        "--output",
        default="yolo11_trt_ready.onnx",
        help="最终输出 ONNX (stage=all/trt 生效)",
    )

    # --- 阶段选择 ---
    p.add_argument(
        "--stage",
        choices=["all", "fix", "trt"],
        default="all",
        help="执行阶段: all=fix+trt, fix=仅 fix_and_validate, trt=仅 fix_trt",
    )

    # --- 中间产物 ---
    p.add_argument(
        "--patched",
        default="yolo11_raw_patched.onnx",
        help="fix 阶段中间产物: patched onnx 路径",
    )
    p.add_argument(
        "--clean",
        default="yolo11_raw_clean.onnx",
        help="fix 阶段最终产物: clean onnx 路径 (stage=fix 时即最终输出)",
    )
    p.add_argument(
        "--keep-mid",
        action="store_true",
        help="stage=all 时保留 patched / clean 中间文件 (默认删除)",
    )

    # --- fix 阶段细节 ---
    p.add_argument(
        "--no-fold", action="store_true", help="关闭 GraphSurgeon 的 fold_constants"
    )
    p.add_argument(
        "--fold-part",
        default="recursive",
        choices=["none", "basic", "recursive"],
        help="fold_constants 的 partitioning 策略",
    )
    p.add_argument(
        "--no-compare", action="store_true", help="跳过 patched vs clean 的数值比对"
    )

    # --- 通用 ---
    p.add_argument(
        "--no-ort-check",
        action="store_true",
        help="跳过所有 ONNX Runtime 加载 / smoke 检查",
    )
    p.add_argument(
        "--verbose", action="store_true", help="详细日志 (主要用于 trt 阶段)"
    )
    return p.parse_args()


def main():
    args = parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    patched_path = Path(args.patched)
    clean_path = Path(args.clean)

    if not input_path.exists():
        raise FileNotFoundError(f"input ONNX not found: {input_path}")

    fold_partitioning = args.fold_part if args.fold_part != "none" else None
    do_fold = not args.no_fold
    do_ort_check = not args.no_ort_check
    do_compare = not args.no_compare

    # ---------- stage = fix ----------
    if args.stage == "fix":
        run_fix_stage(
            src=input_path,
            patched_path=patched_path,
            clean_path=clean_path,
            do_fold=do_fold,
            fold_partitioning=fold_partitioning or "recursive",
            do_ort_check=do_ort_check,
            do_compare=do_compare,
        )
        log(f"[done] fix-only finished. final: {clean_path}")
        return

    # ---------- stage = trt ----------
    if args.stage == "trt":
        run_trt_stage(
            src=input_path,
            dst=output_path,
            do_ort_check=do_ort_check,
            verbose=args.verbose,
        )
        log(f"[done] trt-only finished. final: {output_path}")
        return

    # ---------- stage = all ----------
    clean_out = run_fix_stage(
        src=input_path,
        patched_path=patched_path,
        clean_path=clean_path,
        do_fold=do_fold,
        fold_partitioning=fold_partitioning or "recursive",
        do_ort_check=do_ort_check,
        do_compare=do_compare,
    )

    run_trt_stage(
        src=clean_out,
        dst=output_path,
        do_ort_check=do_ort_check,
        verbose=args.verbose,
    )

    if not args.keep_mid:
        for p in (patched_path, clean_path):
            try:
                if p.exists():
                    p.unlink()
                    log(f"[cleanup] removed intermediate: {p}")
            except OSError as e:
                log(f"[cleanup] failed to remove {p}: {e}")

    log(f"[done] pipeline finished. final: {output_path}")


if __name__ == "__main__":
    main()
