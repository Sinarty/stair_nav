"""ROS-independent transforms and remaining-path geometry for flat obstacle sensing."""
import math


def rigid_transform(translation, quaternion):
    """Return a point transform for an XYZW unit quaternion and XYZ translation."""
    tx, ty, tz = translation
    x, y, z, w = quaternion
    if not all(math.isfinite(v) for v in (*translation, *quaternion)):
        raise ValueError('Nonfinite cloud transform')
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if abs(norm-1.) > .05:
        raise ValueError('Cloud transform quaternion is not normalized')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    matrix = (
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))

    def apply(point):
        px, py, pz = map(float, point)
        return (matrix[0][0]*px+matrix[0][1]*py+matrix[0][2]*pz+tx,
                matrix[1][0]*px+matrix[1][1]*py+matrix[1][2]*pz+ty,
                matrix[2][0]*px+matrix[2][1]*py+matrix[2][2]*pz+tz)
    return apply


def remaining_path(points, progress=None, backtrack=.3, forward=1.5):
    """Project the local origin onto a polyline and keep only its remaining part.

    The caller transforms the reference path using the *cloud timestamp*. The
    initial connector also covers displacement between the robot and its path.
    No speed is used, so the detection corridor persists throughout a pause.
    """
    if not points:
        return None
    if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('Invalid obstacle reference path')
    if progress is not None and (not math.isfinite(progress) or not 0 <= progress <= 1):
        raise ValueError('Invalid obstacle path progress')
    total = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    lower = 0. if progress is None else max(0., progress*total-backtrack)
    upper = total if progress is None else min(total, progress*total+forward)
    best, offset = None, 0.
    for index, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        if length2 < 1e-12:
            continue
        length = math.sqrt(length2)
        if offset+length < lower or offset > upper:
            offset += length
            continue
        t = max(max(0., (lower-offset)/length),
                min(min(1., (upper-offset)/length), -(a[0]*dx+a[1]*dy)/length2))
        projection = a[0]+t*dx, a[1]+t*dy
        distance2 = projection[0]**2+projection[1]**2
        if best is None or distance2 < best[0]:
            best = distance2, index, projection
        offset += length
    if best is None:
        return [(0., 0.), tuple(points[-1])]
    _, index, projection = best
    result = [(0., 0.), projection, *points[index+1:]]
    return [p for i, p in enumerate(result) if not i or math.dist(p, result[i-1]) > 1e-8]
