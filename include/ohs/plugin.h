/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Open Horology Studio plugin ABI, version 1.0.
 *
 * A plugin is a shared library with C linkage and no dependency on OHS's own libraries, so it can
 * be written in C, C++, Rust or anything that can export C symbols. OHS loads it with
 * `ohs dfm --plugin <file>` and runs its rules next to the built-in ones.
 *
 * Compatibility rules (semantic versioning of the ABI):
 *  - OHS refuses a plugin whose abi_major differs from OHS_PLUGIN_ABI_MAJOR.
 *  - A plugin with a higher abi_minor than OHS knows is refused; a lower one is accepted.
 *  - Within one major version structures only grow at the end; `struct_size` tells how much of a
 *    structure the other side filled in or understands. Never read past it.
 *  - Strings returned by a plugin must stay valid until the next call into that plugin; OHS copies
 *    them straight away. Numbers are in millimetres and degrees; the dial plane is x/y.
 */
#ifndef OHS_PLUGIN_H
#define OHS_PLUGIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OHS_PLUGIN_ABI_MAJOR 1
#define OHS_PLUGIN_ABI_MINOR 0

#if defined(_WIN32)
#define OHS_PLUGIN_EXPORT __declspec(dllexport)
#else
#define OHS_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

typedef enum ohs_severity {
    OHS_SEVERITY_INFO = 0,     /* the check passed */
    OHS_SEVERITY_WARNING = 1,
    OHS_SEVERITY_ERROR = 2
} ohs_severity;

/* A flat part as seen in the section at mid-height. */
typedef struct ohs_part {
    uint32_t struct_size;               /* sizeof(ohs_part) in the caller's version */
    const char* name;
    const char* role;                   /* "wheel", "pinion", "plate", "bridge", ... */
    double thickness_mm;
    int32_t teeth;                      /* 0 for parts without teeth */
    double module_mm;                   /* 0 when unknown */
    double pressure_angle_deg;
    int32_t ring_count;                 /* ring 0 is the outer outline (counter-clockwise), the rest are holes (clockwise) */
    const int32_t* ring_point_counts;   /* ring_count entries */
    const double* xy;                   /* x0, y0, x1, y1, ... for all rings in order */
} ohs_part;

typedef struct ohs_finding {
    uint32_t struct_size;               /* the plugin sets this to sizeof(ohs_finding) in its version */
    const char* rule;                   /* short rule name, e.g. "min_hole" */
    int32_t severity;                   /* ohs_severity */
    const char* message;                /* what was found, with the numbers */
    const char* fix;                    /* what to change; may be NULL when severity is INFO */
    double value;                       /* the measured quantity */
    double limit;                       /* the limit it was compared with */
    double x, y;                        /* where, in the part's coordinates */
    int32_t has_location;
} ohs_finding;

/* A set of design-for-manufacturing rules. */
typedef struct ohs_dfm_rule_set {
    uint32_t struct_size;
    uint32_t abi_major;                 /* must be OHS_PLUGIN_ABI_MAJOR */
    uint32_t abi_minor;                 /* the minor version the plugin was written against */
    const char* name;                   /* also used as the process name in reports */
    const char* description;
    const char* version;                /* the plugin's own version */
    /* Writes up to `capacity` findings into `out` and returns how many the part produced. When
     * the return value exceeds `capacity` OHS calls again with a larger buffer. Return a negative
     * number for an internal error (OHS reports it and carries on). */
    int32_t (*check)(const ohs_part* part, ohs_finding* out, int32_t capacity, void* user);
    void* user;
} ohs_dfm_rule_set;

/* The one symbol a DFM plugin must export. */
OHS_PLUGIN_EXPORT const ohs_dfm_rule_set* ohs_plugin_dfm_rule_set(void);

#ifdef __cplusplus
}
#endif

#endif /* OHS_PLUGIN_H */
