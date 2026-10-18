/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Example OHS plugin: a DFM rule that flags holes smaller than the smallest drill you own.
 * Build:  cc -shared -fPIC -I include -o libmin_hole.so examples/plugins/min_hole.c -lm
 * Use:    ohs dfm project.hsproj --plugin ./libmin_hole.so --process wheel-cutting
 */
#include <math.h>
#include <stdio.h>

#include "ohs/plugin.h"

#ifndef MIN_HOLE_ABI_MINOR
#define MIN_HOLE_ABI_MINOR OHS_PLUGIN_ABI_MINOR
#endif
#ifndef MIN_HOLE_ABI_MAJOR
#define MIN_HOLE_ABI_MAJOR OHS_PLUGIN_ABI_MAJOR
#endif

#define SMALLEST_DRILL_MM 0.5

static char message_buffer[8][200];
static char fix_buffer[8][120];

static double ring_diameter(const double* xy, int n) {
    double cx = 0, cy = 0, r = 0;
    for (int i = 0; i < n; ++i) {
        cx += xy[2 * i];
        cy += xy[2 * i + 1];
    }
    cx /= n;
    cy /= n;
    for (int i = 0; i < n; ++i) r += hypot(xy[2 * i] - cx, xy[2 * i + 1] - cy);
    return 2.0 * r / n;
}

static int32_t check(const ohs_part* part, ohs_finding* out, int32_t capacity, void* user) {
    (void)user;
    int32_t found = 0;
    int offset = part->ring_point_counts[0];
    for (int ring = 1; ring < part->ring_count; ++ring) {
        const int n = part->ring_point_counts[ring];
        const double d = ring_diameter(part->xy + 2 * offset, n);
        const int slot = found < 8 ? found : 7;
        if (d < SMALLEST_DRILL_MM) {
            snprintf(message_buffer[slot], sizeof message_buffer[slot], "hole %d is %.3f mm across, below the %.2f mm drill", ring, d, SMALLEST_DRILL_MM);
            snprintf(fix_buffer[slot], sizeof fix_buffer[slot], "open it to at least %.2f mm", SMALLEST_DRILL_MM);
            if (found < capacity) {
                ohs_finding f = {sizeof(ohs_finding), "min_hole", OHS_SEVERITY_ERROR, message_buffer[slot], fix_buffer[slot], d, SMALLEST_DRILL_MM,
                                 part->xy[2 * offset], part->xy[2 * offset + 1], 1};
                out[found] = f;
            }
            ++found;
        }
        offset += n;
    }
    if (found == 0 && capacity > 0) {
        ohs_finding f = {sizeof(ohs_finding), "min_hole", OHS_SEVERITY_INFO, "no hole below the smallest drill", NULL, 0, SMALLEST_DRILL_MM, 0, 0, 0};
        out[0] = f;
        return 1;
    }
    return found;
}

static const ohs_dfm_rule_set rule_set = {sizeof(ohs_dfm_rule_set), MIN_HOLE_ABI_MAJOR, MIN_HOLE_ABI_MINOR, "min-hole-example", "Flags holes smaller than the smallest drill (example plugin)", "1.0.0",
                                          check, NULL};

OHS_PLUGIN_EXPORT const ohs_dfm_rule_set* ohs_plugin_dfm_rule_set(void) { return &rule_set; }
