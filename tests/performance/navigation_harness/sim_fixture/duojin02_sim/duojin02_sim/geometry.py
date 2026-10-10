"""Pure geometry for a convex chassis footprint (coordinates in base frame)."""
def inside_convex(x, y, polygon, tolerance=1e-4):
    if len(polygon) < 3:
        return False
    crosses = [(b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])
               for a,b in zip(polygon, polygon[1:]+polygon[:1])]
    return min(crosses) >= -tolerance or max(crosses) <= tolerance
