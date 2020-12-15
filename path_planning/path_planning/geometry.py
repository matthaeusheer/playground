def intersects(segment_1: list, segment_2: list) -> bool:
    """Assumes line segments are stored in the format [(x0,y0),(x1,y1)]"""
    dx0 = segment_1[1][0] - segment_1[0][0]
    dx1 = segment_2[1][0] - segment_2[0][0]
    dy0 = segment_1[1][1] - segment_1[0][1]
    dy1 = segment_2[1][1] - segment_2[0][1]
    p0 = dy1 * (segment_2[1][0] - segment_1[0][0]) - dx1 * (segment_2[1][1] - segment_1[0][1])
    p1 = dy1 * (segment_2[1][0] - segment_1[1][0]) - dx1 * (segment_2[1][1] - segment_1[1][1])
    p2 = dy0 * (segment_1[1][0] - segment_2[0][0]) - dx0 * (segment_1[1][1] - segment_2[0][1])
    p3 = dy0 * (segment_1[1][0] - segment_2[1][0]) - dx0 * (segment_1[1][1] - segment_2[1][1])
    return (p0 * p1 <= 0) & (p2 * p3 <= 0)
