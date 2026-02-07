/* arc.h - Arc interpolation (G02/G03) for 2D CNC engraver
 *
 * Purpose:
 *  - Convert circular arc moves into sequences of short linear segments
 *  - Support I/J (center offset) and R (radius) arc specification
 *  - Integrate with the kinematics segment_move interface
 *
 * Arc parameters:
 *  G02 Xn Yn In Jn Fn   - Clockwise arc with center offset I,J
 *  G03 Xn Yn In Jn Fn   - Counter-clockwise arc with center offset I,J
 *  G02 Xn Yn Rn Fn      - Clockwise arc with radius R
 *  G03 Xn Yn Rn Fn      - Counter-clockwise arc with radius R
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum arc segment length in mm (smaller = smoother curves) */
#ifndef ARC_SEGMENT_LEN_MM
#define ARC_SEGMENT_LEN_MM 0.5f
#endif

/* Minimum arc radius to avoid degenerate arcs */
#ifndef ARC_RADIUS_MIN_MM
#define ARC_RADIUS_MIN_MM 0.001f
#endif

/* Arc segment callback: called for each linear segment of the arc.
 * Return false from callback to abort arc generation.
 */
typedef bool (*arc_segment_cb_t)(float x, float y, void *user);

/* Compute arc segments from I/J center-offset form.
 *
 *  start_x, start_y  - current position
 *  end_x, end_y      - target position (from X,Y words)
 *  i_offset, j_offset - arc center offsets from start position
 *  clockwise          - true for G02, false for G03
 *  cb                 - callback invoked for each segment endpoint
 *  user               - opaque pointer passed to callback
 *
 * Returns true if arc was generated successfully.
 */
bool arc_generate_ij(float start_x, float start_y,
                     float end_x, float end_y,
                     float i_offset, float j_offset,
                     bool clockwise,
                     arc_segment_cb_t cb, void *user);

/* Compute arc segments from R (radius) form.
 *
 *  start_x, start_y  - current position
 *  end_x, end_y      - target position
 *  radius             - arc radius (positive = minor arc, negative = major arc)
 *  clockwise          - true for G02, false for G03
 *  cb                 - callback invoked for each segment endpoint
 *  user               - opaque pointer passed to callback
 *
 * Returns true if arc was generated successfully.
 */
bool arc_generate_r(float start_x, float start_y,
                    float end_x, float end_y,
                    float radius,
                    bool clockwise,
                    arc_segment_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif
