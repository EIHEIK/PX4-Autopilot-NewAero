"""FreeCAD-side STEP inspection worker.

This file is executed inside Windows FreeCADCmd.  The WSL launcher injects
AUDIT_RUNTIME_CONFIG, AUDIT_RAW_OUTPUT and AUDIT_SELECTED_STEP as globals so
that FreeCAD only sees Windows/UNC paths.
"""

from __future__ import annotations

import json
import os
import traceback

import FreeCAD as App
import Import


def _bbox(shape):
    box = shape.BoundBox
    return {
        "xmin": box.XMin,
        "xmax": box.XMax,
        "ymin": box.YMin,
        "ymax": box.YMax,
        "zmin": box.ZMin,
        "zmax": box.ZMax,
        "x_length": box.XLength,
        "y_length": box.YLength,
        "z_length": box.ZLength,
        "center": [
            0.5 * (box.XMin + box.XMax),
            0.5 * (box.YMin + box.YMax),
            0.5 * (box.ZMin + box.ZMax),
        ],
    }


def _shape_objects(doc):
    return [
        obj for obj in doc.Objects
        if hasattr(obj, "Shape") and obj.Shape is not None and not obj.Shape.isNull()
    ]


def _combined_bbox(objects):
    boxes = [obj.Shape.BoundBox for obj in objects]
    if not boxes:
        return None
    return {
        "xmin": min(box.XMin for box in boxes),
        "xmax": max(box.XMax for box in boxes),
        "ymin": min(box.YMin for box in boxes),
        "ymax": max(box.YMax for box in boxes),
        "zmin": min(box.ZMin for box in boxes),
        "zmax": max(box.ZMax for box in boxes),
        "x_length": max(box.XMax for box in boxes) - min(box.XMin for box in boxes),
        "y_length": max(box.YMax for box in boxes) - min(box.YMin for box in boxes),
        "z_length": max(box.ZMax for box in boxes) - min(box.ZMin for box in boxes),
    }


def _find_exact(doc, candidates):
    by_label = {}
    for obj in doc.Objects:
        by_label.setdefault(obj.Label, []).append(obj)
    for label in candidates:
        matches = by_label.get(label, [])
        shaped = [obj for obj in matches if hasattr(obj, "Shape") and not obj.Shape.isNull()]
        if shaped:
            return shaped[0]
        if matches:
            return matches[0]
    raise RuntimeError("No object found for labels: " + ", ".join(candidates))


def _children(obj):
    for attr in ("Group", "Objects"):
        value = getattr(obj, attr, None)
        if value:
            return list(value)
    return []


def _descendant_shapes(root):
    stack = [root]
    seen = set()
    result = []
    while stack:
        obj = stack.pop()
        key = getattr(obj, "Name", str(id(obj)))
        if key in seen:
            continue
        seen.add(key)
        stack.extend(_children(obj))
        shape = getattr(obj, "Shape", None)
        if shape is not None and not shape.isNull():
            result.append(obj)
    return result


def _object_record(obj):
    record = {
        "name": obj.Name,
        "label": obj.Label,
        "type_id": obj.TypeId,
        "child_count": len(_children(obj)),
    }
    shape = getattr(obj, "Shape", None)
    if shape is not None and not shape.isNull():
        record["bbox_mm"] = _bbox(shape)
        record["solid_count"] = len(shape.Solids)
        record["volume_mm3"] = shape.Volume
    return record


def _inspect_one(model, labels, export_path=None):
    before = set(App.listDocuments())
    Import.open(model["path_windows"])
    after = set(App.listDocuments())
    created = sorted(after - before)
    doc = App.getDocument(created[-1]) if created else App.ActiveDocument
    if doc is None:
        raise RuntimeError("FreeCAD did not create a document for " + model["path_windows"])
    doc.recompute()

    wheels = {
        key: _object_record(_find_exact(doc, labels[key]))
        for key in ("left_main_wheel", "right_main_wheel", "nose_wheel")
    }
    roots = [_find_exact(doc, [label]) for label in labels["gear_roots"]]
    root_records = [_object_record(root) for root in roots]

    all_shapes = _shape_objects(doc)
    global_box = _combined_bbox(all_shapes)

    exported = None
    if export_path:
        selected = []
        selected_names = set()
        for root in roots:
            for obj in _descendant_shapes(root):
                if obj.Name not in selected_names:
                    selected.append(obj)
                    selected_names.add(obj.Name)
        if not selected:
            selected = [obj for obj in roots if hasattr(obj, "Shape") and not obj.Shape.isNull()]
        if not selected:
            raise RuntimeError("Gear roots contain no exportable shapes")
        Import.export(selected, export_path)
        exported = {
            "path": export_path,
            "object_count": len(selected),
            "labels": [obj.Label for obj in selected],
        }

    result = {
        "id": model["id"],
        "path": model["path_original"],
        "document": doc.Name,
        "object_count": len(doc.Objects),
        "shape_object_count": len(all_shapes),
        "global_bbox_mm": global_box,
        "wheels": wheels,
        "gear_roots": root_records,
        "selected_export": exported,
    }
    App.closeDocument(doc.Name)
    return result


def main():
    with open(AUDIT_RUNTIME_CONFIG, "r", encoding="utf-8") as stream:
        config = json.load(stream)
    results = []
    for index, model in enumerate(config["step_models"]):
        selected = AUDIT_SELECTED_STEP if index == 0 and AUDIT_SELECTED_STEP else None
        results.append(_inspect_one(model, config["object_labels"], selected))
    payload = {
        "ok": True,
        "freecad_version": App.Version(),
        "models": results,
    }
    with open(AUDIT_RAW_OUTPUT, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False)


try:
    main()
except Exception as exc:
    payload = {"ok": False, "error": str(exc), "traceback": traceback.format_exc()}
    with open(AUDIT_RAW_OUTPUT, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False)
    raise
