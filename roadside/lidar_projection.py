from __future__ import print_function

import itertools


def _box_corners(center, extent):
    x, y, z = center
    ex, ey, ez = [max(0.3, float(v)) for v in extent]
    hx = max(0.5, ex * 0.6)
    hy = max(0.5, ey * 0.6)
    hz = max(0.5, ez * 0.6)
    return [(x + sx * hx, y + sy * hy, z + sz * hz)
            for sx, sy, sz in itertools.product((-1.0, 1.0), repeat=3)]


def _project_rect(projector, center, extent, width, height):
    pixels = []
    for x, y, z in _box_corners(center, extent):
        p = projector.project(x, y, z)
        if p is not None:
            pixels.append((int(p["u"]), int(p["v"])))
    if len(pixels) < 2:
        return None
    xs = [p[0] for p in pixels]; ys = [p[1] for p in pixels]
    x1 = max(0, min(xs)); y1 = max(0, min(ys))
    x2 = min(int(width) - 1, max(xs)); y2 = min(int(height) - 1, max(ys))
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def _project_rect_diagnostic(projector, center, extent, width, height):
    pixels=[]
    for x,y,z in _box_corners(center,extent):
        p=projector.project(x,y,z)
        if p is not None:pixels.append((float(p["u"]),float(p["v"])))
    if len(pixels)<2:return None,"behind_camera"
    xs=[p[0] for p in pixels];ys=[p[1] for p in pixels]
    if max(xs)<0:return None,"left"
    if min(xs)>=float(width):return None,"right"
    if max(ys)<0:return None,"above"
    if min(ys)>=float(height):return None,"below"
    x1=max(0,int(min(xs)));y1=max(0,int(min(ys)))
    x2=min(int(width)-1,int(max(xs)));y2=min(int(height)-1,int(max(ys)))
    if x2<=x1 or y2<=y1:return None,"degenerate"
    return [x1,y1,x2,y2],None


def project_lidar_tracks(projector, tracked_candidates, width, height):
    """Return camera-visible tracks with their original source index preserved."""
    result = []
    for source_index, item in enumerate(tracked_candidates or []):
        rect = _project_rect(projector,
                             (item.get("x", 0.0), item.get("y", 0.0), item.get("z", 0.0)),
                             item.get("extent", [2.0, 1.0, 1.0]), width, height)
        if rect is None:
            continue
        out = dict(item)
        out["bbox"] = rect
        out["source_index"] = source_index
        result.append(out)
    return result


def project_lidar_tracks_with_diagnostics(projector, tracked_candidates,
                                          width, height):
    """Return visible projections plus per-source off-screen reasons."""
    result=[];rejections={}
    for source_index,item in enumerate(tracked_candidates or []):
        rect,reason=_project_rect_diagnostic(
            projector,(item.get("x",0.0),item.get("y",0.0),item.get("z",0.0)),
            item.get("extent",[2.0,1.0,1.0]),width,height)
        if rect is None:
            rejections[source_index]=reason;continue
        out=dict(item);out["bbox"]=rect;out["source_index"]=source_index
        result.append(out)
    return result,rejections
