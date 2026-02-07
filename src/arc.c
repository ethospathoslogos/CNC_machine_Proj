/* arc.c - Arc interpolation implementation for 2D CNC engraver
 *
 * Converts G02/G03 circular arcs into a series of short linear segments.
 * Uses angular stepping to produce uniform segment lengths along the arc.
 */

#include "arc.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ----------------------------- I/J center-offset arc ----------------------------- */

bool arc_generate_ij(float start_x, float start_y,
                     float end_x, float end_y,
                     float i_offset, float j_offset,
                     bool clockwise,
                     arc_segment_cb_t cb, void *user)
{
    if (!cb) return false;

    /* Arc center in absolute coordinates */
    float cx = start_x + i_offset;
    float cy = start_y + j_offset;

    /* Radii from center to start and end points */
    float r_start = sqrtf((start_x - cx) * (start_x - cx) +
                          (start_y - cy) * (start_y - cy));
    float r_end   = sqrtf((end_x - cx)   * (end_x - cx) +
                          (end_y - cy)   * (end_y - cy));

    /* Use average radius for segment calculation */
    float radius = 0.5f * (r_start + r_end);
    if (radius < ARC_RADIUS_MIN_MM) return false;

    /* Start and end angles */
    float theta_start = atan2f(start_y - cy, start_x - cx);
    float theta_end   = atan2f(end_y   - cy, end_x   - cx);

    /* Compute angular sweep */
    float angular_travel;
    if (clockwise) {
        angular_travel = theta_start - theta_end;
        if (angular_travel <= 0.0f)
            angular_travel += (float)(2.0 * M_PI);
    } else {
        angular_travel = theta_end - theta_start;
        if (angular_travel <= 0.0f)
            angular_travel += (float)(2.0 * M_PI);
    }

    /* Full circle detection: if start ~= end, assume full circle */
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    if (fabsf(dx) < ARC_RADIUS_MIN_MM && fabsf(dy) < ARC_RADIUS_MIN_MM) {
        angular_travel = (float)(2.0 * M_PI);
    }

    /* Number of segments based on arc length and desired segment length */
    float arc_length = radius * angular_travel;
    int num_segments = (int)(arc_length / ARC_SEGMENT_LEN_MM);
    if (num_segments < 1) num_segments = 1;
    if (num_segments > 10000) num_segments = 10000; /* sanity clamp */

    /* Angular step per segment */
    float theta_step = angular_travel / (float)num_segments;
    if (clockwise) theta_step = -theta_step;

    /* Generate segment endpoints */
    float theta = theta_start;
    for (int i = 1; i <= num_segments; i++) {
        float seg_x, seg_y;

        if (i == num_segments) {
            /* Last segment snaps to exact endpoint */
            seg_x = end_x;
            seg_y = end_y;
        } else {
            theta += theta_step;
            seg_x = cx + radius * cosf(theta);
            seg_y = cy + radius * sinf(theta);
        }

        if (!cb(seg_x, seg_y, user)) return false;
    }

    return true;
}

/* ----------------------------- R (radius) arc ----------------------------- */

bool arc_generate_r(float start_x, float start_y,
                    float end_x, float end_y,
                    float radius,
                    bool clockwise,
                    arc_segment_cb_t cb, void *user)
{
    if (!cb) return false;

    float abs_r = fabsf(radius);
    if (abs_r < ARC_RADIUS_MIN_MM) return false;

    /* Midpoint between start and end */
    float mid_x = 0.5f * (start_x + end_x);
    float mid_y = 0.5f * (start_y + end_y);

    /* Half-chord vector */
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float half_chord = 0.5f * sqrtf(dx * dx + dy * dy);

    if (half_chord > abs_r) return false; /* chord longer than diameter */

    /* Distance from midpoint to center along perpendicular */
    float h = sqrtf(abs_r * abs_r - half_chord * half_chord);

    /* Perpendicular unit vector (rotated 90 degrees from chord direction) */
    float chord_len = 2.0f * half_chord;
    if (chord_len < ARC_RADIUS_MIN_MM) return false;

    float perp_x = -dy / chord_len;
    float perp_y =  dx / chord_len;

    /* Choose center side based on CW/CCW and sign of R:
     * - Positive R + CW: center on right side of chord
     * - Positive R + CCW: center on left side of chord
     * - Negative R: major arc (opposite side)
     */
    bool use_left = !clockwise;
    if (radius < 0.0f) use_left = !use_left;

    float cx, cy;
    if (use_left) {
        cx = mid_x + h * perp_x;
        cy = mid_y + h * perp_y;
    } else {
        cx = mid_x - h * perp_x;
        cy = mid_y - h * perp_y;
    }

    /* Convert to I/J offset form and delegate */
    float i_offset = cx - start_x;
    float j_offset = cy - start_y;

    return arc_generate_ij(start_x, start_y, end_x, end_y,
                           i_offset, j_offset, clockwise, cb, user);
}
