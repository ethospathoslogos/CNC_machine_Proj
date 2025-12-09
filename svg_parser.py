from svgpathtools import svg2paths2
from pathlib import Path
from typing import List, Tuple


# Sample a single SVG segment into points
def sample_segment(segment, resolution: float = 0.5) -> List[Tuple[float, float]]:
    length = segment.length(error=1e-5)
    num_samples = max(2, int(length / resolution))

    pts = []
    for i in range(num_samples + 1):
        t = i / num_samples
        z = segment.point(t)
        pts.append((z.real, z.imag))
    return pts

# Extract polylines from a full path
def path_to_polylines(path, resolution: float = 0.5) -> List[List[Tuple[float, float]]]:
    polylines = []
    current_polyline = []

    for segment in path:
        seg_points = sample_segment(segment, resolution)

        # Avoid duplicating the first point
        if current_polyline and seg_points:
            seg_points = seg_points[1:]

        current_polyline.extend(seg_points)

        # If segment is a Close object, finalize polyline
        if segment.__class__.__name__ == "Close":
            polylines.append(current_polyline)
            current_polyline = []

    if current_polyline:
        polylines.append(current_polyline)

    return polylines

# Parse an SVG file's paths into list of polylines
def parse_svg_to_polylines(svg_path: str, resolution: float = 0.5) -> List[List[Tuple[float, float]]]:
    paths, attributes, svg_attr = svg2paths2(svg_path)

    all_polylines = []
    for path, attr in zip(paths, attributes):
        if "d" not in attr:
            continue
        polylines = path_to_polylines(path, resolution)
        all_polylines.extend(polylines)

    return all_polylines

# ----------------------------
# Added: polylines -> G-code
# ----------------------------
def _fmt(v: float) -> str:
    #Format numeric values consistently for G-code (3 decimals).
    return f"{v:.3f}"

def _dedupe_points(poly: List[Tuple[float, float]], tol: float = 1e-6) -> List[Tuple[float, float]]:
    """Remove consecutive points that are closer than tol (Euclidean)."""
    import math
    if not poly:
        return []
    out: List[Tuple[float, float]] = [poly[0]]
    for p in poly[1:]:
        last = out[-1]
        if math.hypot(p[0] - last[0], p[1] - last[1]) > tol:
            out.append(p)
    return out

def polylines_to_gcode(
    polylines: List[List[Tuple[float, float]]],
    safe_z: float = 5.0,
    cut_z: float = -1.0,
    plunge_feed: float = 200.0,
    travel_feed: float = 1000.0,
    spindle_s: int | None = None,
    comment: str | None = None,
    dedupe_tol: float = 1e-6,
    include_header: bool = True,
    include_footer: bool = True,
) -> List[str]:
    """
    Convert polylines -> list of G-code lines.

    - polylines: list of polylines; each polyline is a list of (x, y) points in machine units (e.g. mm).
    - safe_z: Z used for rapid travel (positive above work).
    - cut_z: Z used while cutting (negative plunges).
    - plunge_feed: feed rate for plunges and cutting (units/min).
    - travel_feed: feed rate for rapids (used as F on G0 lines for compatibility).
    - spindle_s: optional S value to emit with M3 (if provided).
    - comment: optional top-level comment string.
    - dedupe_tol: consecutive points closer than this are removed.
    - include_header/footer: include standard header/footer lines.
    Returns a List[str] where each element is one G-code line (no trailing newline).
    """
    lines: List[str] = []

    if include_header:
        if comment:
            lines.append(f"({comment})")
        lines.append("G21")   # mm
        lines.append("G90")   # absolute coordinates
        lines.append(f"G0 Z{_fmt(safe_z)}")
        if spindle_s is not None:
            lines.append(f"M3 S{int(spindle_s)}")
            lines.append("G4 P0.1")  # brief dwell to allow spindle/laser to spin up

    for poly in polylines:
        if not poly:
            continue
        work_poly = _dedupe_points(poly, dedupe_tol) if dedupe_tol is not None else list(poly)
        if len(work_poly) == 0:
            continue
        # Move rapid to first point at safe Z
        x0, y0 = work_poly[0]
        lines.append(f"G0 X{_fmt(x0)} Y{_fmt(y0)} F{_fmt(travel_feed)}")
        # Plunge to cut depth
        lines.append(f"G1 Z{_fmt(cut_z)} F{_fmt(plunge_feed)}")
        # Cut along polyline (skip the first point)
        for (x, y) in work_poly[1:]:
            lines.append(f"G1 X{_fmt(x)} Y{_fmt(y)} F{_fmt(plunge_feed)}")
        # Retract
        lines.append(f"G0 Z{_fmt(safe_z)} F{_fmt(travel_feed)}")

    if include_footer:
        if spindle_s is not None:
            lines.append("M5")
        lines.append("G0 X0 Y0")
        lines.append("M2")

    return lines

def svg_to_gcode(
    svg_path: str,
    resolution: float = 0.5,
    safe_z: float = 5.0,
    cut_z: float = -1.0,
    plunge_feed: float = 200.0,
    travel_feed: float = 1000.0,
    spindle_s: int | None = None,
    comment: str | None = None,
    dedupe_tol: float = 1e-6,
    include_header: bool = True,
    include_footer: bool = True,
) -> List[str]:
    """
    Convenience wrapper: parse svg_path into polylines (using existing parse_svg_to_polylines)
    and convert them to G-code using polylines_to_gcode.
    """
    polys = parse_svg_to_polylines(svg_path, resolution=resolution)
    return polylines_to_gcode(
        polys,
        safe_z=safe_z,
        cut_z=cut_z,
        plunge_feed=plunge_feed,
        travel_feed=travel_feed,
        spindle_s=spindle_s,
        comment=comment,
        dedupe_tol=dedupe_tol,
        include_header=include_header,
        include_footer=include_footer,
    )

if __name__ == "__main__":
   

    test_svg = "C:/Users/nana1/OneDrive/Desktop/donuts-cake-svgrepo-com.svg"
    polys = parse_svg_to_polylines(test_svg, resolution=0.5)
  
    # Generate and print G-code so you can inspect the instructions
    gcode_lines = polylines_to_gcode(polys, safe_z=5.0, cut_z=-1.0, plunge_feed=150.0, travel_feed=1200.0, spindle_s=800, comment="generated from svg")
    print("\nGenerated G-code (one line per entry):")
    for ln in gcode_lines:
        print(ln)

    # Write the G-code lines to a .gcode text file next to the SVG input
    try:
        svg_path_obj = Path(test_svg)
        out_path = svg_path_obj.with_suffix(".gcode")
        # Ensure parent directory exists for the output
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as fh:
            for ln in gcode_lines:
                fh.write(ln + "\n")
        print(f"\nG-code written to: {out_path}")
    except Exception as e:
        print("Failed to write G-code file:", e)
