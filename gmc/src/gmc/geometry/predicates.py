"""Shapely usage conventions (Guide §7.2) and geometry validity checks."""
import numpy as np
import shapely
from shapely.geometry import MultiPolygon, Polygon


def set_precision_all(geoms, grid_size: float):
    return [shapely.set_precision(g, grid_size) for g in geoms]


def set_precision_outer(geom, grid_size: float):
    """Snap an outer approximation without ever shrinking it.

    GEOS precision reduction rounds coordinates to the nearest grid point.  A
    plain ``set_precision`` can therefore move an outer boundary inward and
    invalidate ``O_true subset O_plus``.  A pre-snap outward pad larger than
    the maximum rounding displacement retains the required superset direction
    while keeping the result on the configured precision grid.
    """
    if grid_size <= 0.0 or geom.is_empty:
        return clean(geom)
    # Coordinate rounding moves a point by at most sqrt(2)/2 grid cells.  Pad
    # by two cells before reduction so the final fixed-precision polygon stays
    # an outer approximation even at diagonal or topologically merged edges.
    padded = geom.buffer(2.0 * grid_size, join_style="mitre")
    if padded.is_empty:
        raise FloatingPointError(
            "outward precision padding collapsed a nonempty geometry")
    padded_bounds = np.asarray(padded.bounds, dtype=float)
    if (padded_bounds.shape != (4,)
            or not np.all(np.isfinite(padded_bounds))):
        raise FloatingPointError(
            "outward precision padding produced non-finite bounds")
    snapped = clean(shapely.set_precision(padded, grid_size))
    if snapped.is_empty:
        raise FloatingPointError(
            "outer precision reduction collapsed a nonempty geometry")
    snapped_bounds = np.asarray(snapped.bounds, dtype=float)
    if (snapped_bounds.shape != (4,)
            or not np.all(np.isfinite(snapped_bounds))
            or not snapped.covers(geom)):
        raise FloatingPointError(
            "outer precision reduction could not prove containment")
    return snapped


def set_precision_inner(geom, grid_size: float):
    """Snap an inner approximation without ever expanding it.

    A pre-snap erosion larger than the maximum rounding displacement keeps the
    quantized result inside the original certified inner approximation.
    """
    if grid_size <= 0.0 or geom.is_empty:
        return clean(geom)
    # Symmetrically erode before rounding.  Collapse to empty is conservative
    # for an inner obstacle approximation and therefore preferable to an
    # outward-rounded false collision certificate.
    eroded = geom.buffer(-2.0 * grid_size, join_style="mitre")
    if eroded.is_empty:
        return eroded
    snapped = clean(shapely.set_precision(eroded, grid_size))
    if snapped.is_empty:
        return snapped
    snapped_bounds = np.asarray(snapped.bounds, dtype=float)
    if (snapped_bounds.shape != (4,)
            or not np.all(np.isfinite(snapped_bounds))
            or not geom.covers(snapped)):
        raise FloatingPointError(
            "inner precision reduction could not prove containment")
    return snapped


def union_all(geoms):
    return shapely.union_all(list(geoms))


def clean(geom):
    """normalize + make_valid where needed (§7.2)."""
    if not geom.is_valid:
        geom = shapely.make_valid(geom)
    return shapely.normalize(geom)


def polygon_components(geom, area_min: float) -> list[Polygon]:
    """Split into Polygon components, dropping numeric slivers below the
    documented area_min threshold (§7.2)."""
    area_min = float(area_min)
    if not np.isfinite(area_min) or area_min < 0.0:
        raise ValueError("area_min must be finite and non-negative")
    if not geom.is_valid:
        raise FloatingPointError(
            "component extraction requires a valid Boolean result")
    area = float(geom.area)
    if not np.isfinite(area) or area < 0.0:
        raise FloatingPointError(
            "component extraction requires finite non-negative area")
    if geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        parts = [geom]
    elif isinstance(geom, MultiPolygon):
        parts = list(geom.geoms)
    else:
        parts = [g for g in getattr(geom, "geoms", [])
                 if isinstance(g, Polygon)]
    retained = []
    for p in parts:
        bounds = np.asarray(p.bounds, dtype=float)
        if (not p.is_valid or not np.isfinite(p.area)
                or bounds.shape != (4,) or not np.all(np.isfinite(bounds))):
            raise FloatingPointError("invalid polygonal Boolean component")
        if p.area > area_min:
            retained.append(shapely.normalize(p))
    # Component indices become artifact and graph IDs.  Area alone is not a
    # total ordering, so equal-area components otherwise inherit GEOS/input
    # traversal order and break reproducibility.
    return sorted(retained,
                  key=lambda p: (-float(p.area), tuple(map(float, p.bounds)),
                                 p.wkb_hex))


def check_workspace(ws: Polygon) -> bool:
    return bool(ws.is_valid and not ws.is_empty and ws.area > 0)


def scale_lengths(supports, factor: float):
    """Unit-scale equivariance helper (M0 acceptance §4.3): lengths scale by
    a, covariances by a^2."""
    from ..types import GaussianSupport2D
    return tuple(GaussianSupport2D(mean=np.asarray(s.mean) * factor,
                                   covariance=np.asarray(s.covariance) * factor ** 2,
                                   level=s.level, primitive_id=s.primitive_id)
                 for s in supports)
