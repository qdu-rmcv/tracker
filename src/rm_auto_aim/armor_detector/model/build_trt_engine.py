"""
Build TensorRT engine from ONNX.

This script only depends on:
  - tensorrt
  - standard Python libraries

It does NOT depend on:
  - onnx
  - onnx_graphsurgeon
  - numpy
"""

from __future__ import annotations

import argparse
import os

import tensorrt as trt


def list_nms_plugins():
    logger = trt.Logger(trt.Logger.INFO)
    trt.init_libnvinfer_plugins(logger, "")

    registry = trt.get_plugin_registry()

    print(f"TensorRT version: {trt.__version__}")
    print(f"TensorRT module : {trt.__file__}")
    print("NMS-related plugins:")

    found = False

    for creator in registry.plugin_creator_list:
        name = getattr(creator, "name", "")
        version = getattr(creator, "plugin_version", "")
        namespace = getattr(creator, "plugin_namespace", "")

        if "NMS" in name or "nms" in name:
            found = True
            print(f"  {name}, version={version}, namespace='{namespace}'")

    if not found:
        print("  <none>")


def build_engine(
    onnx_path: str,
    engine_path: str,
    *,
    fp16: bool = True,
    int8: bool = False,
    strongly_typed: bool = False,
    optimization_level: int = 5,
    workspace_gib: int = 8,
    timing_cache_path: str = "trt_timing.cache",
    avg_timing_iterations: int = 8,
    builder_threads: int = 0,
    prefer_precision_constraints: bool = True,
    sparsity: bool = False,
    tf32: bool = True,
    verbose: bool = False,
):
    logger = trt.Logger(trt.Logger.VERBOSE if verbose else trt.Logger.INFO)

    # Required for EfficientNMS_TRT plugin visibility.
    trt.init_libnvinfer_plugins(logger, "")

    builder = trt.Builder(logger)

    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)

    if strongly_typed:
        if not hasattr(trt.NetworkDefinitionCreationFlag, "STRONGLY_TYPED"):
            raise RuntimeError("This TensorRT version does not expose STRONGLY_TYPED.")

        network_flags |= 1 << int(trt.NetworkDefinitionCreationFlag.STRONGLY_TYPED)

    network = builder.create_network(network_flags)
    parser = trt.OnnxParser(network, logger)

    with open(onnx_path, "rb") as f:
        onnx_bytes = f.read()

    if not parser.parse(onnx_bytes):
        for i in range(parser.num_errors):
            print(parser.get_error(i))
        raise RuntimeError("ONNX parse failed")

    config = builder.create_builder_config()

    # Workspace
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE,
        workspace_gib << 30,
    )

    # Equivalent to trtexec --builderOptimizationLevel=5
    config.builder_optimization_level = optimization_level

    # Equivalent to trtexec --avgTiming=8
    if avg_timing_iterations > 0:
        config.avg_timing_iterations = avg_timing_iterations

    # Precision flags
    if not strongly_typed:
        if fp16:
            if builder.platform_has_fast_fp16:
                config.set_flag(trt.BuilderFlag.FP16)
            else:
                print("[warning] platform_has_fast_fp16 is False, skip FP16")

        if int8:
            if builder.platform_has_fast_int8:
                config.set_flag(trt.BuilderFlag.INT8)
                print("[warning] INT8 flag set, but no calibrator is configured")
            else:
                print("[warning] platform_has_fast_int8 is False, skip INT8")

    # TF32
    # TensorRT usually enables TF32 by default on Ampere+.
    # Only clear it when explicitly disabled.
    if not tf32:
        config.clear_flag(trt.BuilderFlag.TF32)

    # Precision constraints
    if prefer_precision_constraints:
        if hasattr(trt.BuilderFlag, "PREFER_PRECISION_CONSTRAINTS"):
            config.set_flag(trt.BuilderFlag.PREFER_PRECISION_CONSTRAINTS)

    # 2:4 structured sparsity
    if sparsity:
        config.set_flag(trt.BuilderFlag.SPARSE_WEIGHTS)

    # Builder thread hint
    if builder_threads > 0:
        os.environ["TRT_BUILDER_MAX_THREADS"] = str(builder_threads)

    # Timing cache
    timing_cache_blob = b""

    if timing_cache_path and os.path.exists(timing_cache_path):
        with open(timing_cache_path, "rb") as f:
            timing_cache_blob = f.read()

        print(
            f"[timing cache] loaded {len(timing_cache_blob)} bytes "
            f"from {timing_cache_path}"
        )

    timing_cache = config.create_timing_cache(timing_cache_blob)
    config.set_timing_cache(timing_cache, ignore_mismatch=False)

    print(
        "[builder] "
        f"onnx={onnx_path}, "
        f"engine={engine_path}, "
        f"level={optimization_level}, "
        f"fp16={fp16}, "
        f"int8={int8}, "
        f"strongly_typed={strongly_typed}, "
        f"sparsity={sparsity}, "
        f"tf32={tf32}, "
        f"workspace={workspace_gib}GiB, "
        f"avg_timing={avg_timing_iterations}"
    )

    serialized = builder.build_serialized_network(network, config)

    if serialized is None:
        raise RuntimeError("build_serialized_network returned None")

    serialized_bytes = bytes(serialized)

    with open(engine_path, "wb") as f:
        f.write(serialized_bytes)

    print(
        f"[engine] saved: {engine_path} "
        f"({len(serialized_bytes) / (1 << 20):.1f} MiB)"
    )

    if timing_cache_path:
        new_cache_blob = bytes(timing_cache.serialize())

        with open(timing_cache_path, "wb") as f:
            f.write(new_cache_blob)

        print(
            f"[timing cache] saved {len(new_cache_blob)} bytes "
            f"to {timing_cache_path}"
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--onnx",
        required=False,
        help="Input ONNX path",
    )

    parser.add_argument(
        "--engine",
        required=False,
        help="Output TensorRT engine path",
    )

    parser.set_defaults(fp16=True)
    parser.add_argument("--fp16", dest="fp16", action="store_true")
    parser.add_argument("--no-fp16", dest="fp16", action="store_false")

    parser.add_argument("--int8", action="store_true")
    parser.add_argument("--strongly-typed", action="store_true")

    parser.add_argument("--opt-level", type=int, default=5)
    parser.add_argument("--workspace-gib", type=int, default=8)
    parser.add_argument("--timing-cache", default="trt_timing.cache")
    parser.add_argument("--avg-timing", type=int, default=8)
    parser.add_argument("--builder-threads", type=int, default=0)

    parser.add_argument("--sparsity", action="store_true")

    parser.set_defaults(tf32=True)
    parser.add_argument("--tf32", dest="tf32", action="store_true")
    parser.add_argument("--no-tf32", dest="tf32", action="store_false")

    parser.add_argument(
        "--no-prefer-precision-constraints",
        action="store_true",
    )

    parser.add_argument("--verbose", action="store_true")

    parser.add_argument(
        "--list-plugins",
        action="store_true",
        help="List TensorRT NMS-related plugins and exit",
    )

    args = parser.parse_args()

    if args.list_plugins:
        list_nms_plugins()
        return

    if not args.onnx:
        raise ValueError("--onnx is required unless --list-plugins is used")

    if not args.engine:
        raise ValueError("--engine is required unless --list-plugins is used")

    build_engine(
        onnx_path=args.onnx,
        engine_path=args.engine,
        fp16=args.fp16,
        int8=args.int8,
        strongly_typed=args.strongly_typed,
        optimization_level=args.opt_level,
        workspace_gib=args.workspace_gib,
        timing_cache_path=args.timing_cache,
        avg_timing_iterations=args.avg_timing,
        builder_threads=args.builder_threads,
        prefer_precision_constraints=not args.no_prefer_precision_constraints,
        sparsity=args.sparsity,
        tf32=args.tf32,
        verbose=args.verbose,
    )


if __name__ == "__main__":
    main()
