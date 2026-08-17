from __future__ import print_function


def apply_camera_runtime_overrides(config, source=None, model=None):
    """Apply CLI camera choices without mutating the loaded configuration."""
    result = dict(config or {})
    result["camera_fusion"] = dict(result.get("camera_fusion", {}) or {})
    result["camera_detection"] = dict(result.get("camera_detection", {}) or {})
    if source is not None:result["camera_fusion"]["source"] = str(source)
    if model is not None:result["camera_detection"]["model"] = str(model)
    return result
