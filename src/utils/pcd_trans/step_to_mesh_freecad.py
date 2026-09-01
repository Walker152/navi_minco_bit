#!/usr/bin/env python3
"""FreeCAD worker: tessellate one STEP file into a temporary STL mesh."""

from pathlib import Path
import os
import traceback

import MeshPart
import Part


def main():
    try:
        input_path = Path(os.environ["CLOUDLAB_STEP_INPUT"]).expanduser().resolve()
        output_path = Path(os.environ["CLOUDLAB_STEP_OUTPUT"]).expanduser().resolve()
        linear_deflection = float(os.environ["CLOUDLAB_STEP_LINEAR_DEFLECTION"])
        angular_deflection = float(os.environ["CLOUDLAB_STEP_ANGULAR_DEFLECTION"])
    except KeyError as exc:
        raise RuntimeError(f"缺少 FreeCAD worker 环境变量：{exc.args[0]}") from exc
    if not input_path.is_file():
        raise FileNotFoundError(input_path)
    shape = Part.read(str(input_path))
    if shape.isNull():
        raise RuntimeError("FreeCAD 没有从 STEP 中读到有效 Shape")
    mesh = MeshPart.meshFromShape(
        Shape=shape,
        LinearDeflection=linear_deflection,
        AngularDeflection=angular_deflection,
        Relative=False,
    )
    if mesh.CountFacets <= 0:
        raise RuntimeError("STEP 三角化没有生成面片")
    mesh.write(str(output_path))
    print(f"facets={mesh.CountFacets} output={output_path}")


# FreeCAD 0.19 以宏方式执行 .py，__name__ 不是 "__main__"。
try:
    main()
except Exception:
    traceback.print_exc()
    raise
