# Plugins

A plugin is a shared library with a C interface (`include/ohs/plugin.h`), so you can write it in C,
C++, Rust or anything that exports C symbols, without linking any OHS library. Plugin ABI **1.0** lets
you add **DFM rule sets**: your own checks, run next to the built-in ones.

## Using a plugin

```
ohs dfm movement.hsproj --plugin ./libmin_hole.so --process wheel-cutting
```

The plugin's rules run on every part, and their findings appear with the plugin's name as the
process, in the same output (and in `--json`) as the built-in rules. In Python:
`project.dfm(plugins=["./libmin_hole.so"])`.

## Writing one

A DFM plugin exports one function:

```c
#include "ohs/plugin.h"

static int32_t check(const ohs_part* part, ohs_finding* out, int32_t capacity, void* user) {
    /* look at part->xy (all rings), part->teeth, part->thickness_mm ... */
    /* write up to `capacity` findings into `out`; return how many there are */
}

static const ohs_dfm_rule_set rules = {
    sizeof(ohs_dfm_rule_set), OHS_PLUGIN_ABI_MAJOR, OHS_PLUGIN_ABI_MINOR,
    "my-rules", "What these rules check", "1.0.0", check, NULL};

OHS_PLUGIN_EXPORT const ohs_dfm_rule_set* ohs_plugin_dfm_rule_set(void) { return &rules; }
```

`examples/plugins/min_hole.c` is a complete, working example: it flags holes smaller than the
smallest drill you own. Build it with
`cc -shared -fPIC -I include -o libmin_hole.so examples/plugins/min_hole.c -lm`.

The part is given as its mid-height section: ring 0 is the outer outline (counter-clockwise), the
following rings are holes (clockwise), all points in millimetres in the dial plane. A finding has
a rule name, a severity (`OHS_SEVERITY_INFO` means the check passed), a message, a fix, the
measured value and its limit, and optionally a location.

## Compatibility rules

- OHS refuses a plugin whose `abi_major` differs from its own, and one whose `abi_minor` is newer
  than OHS knows, with a message that names both versions.
- Within major version 1, structures only grow at the end. Every structure starts with its
  `struct_size`; never read past the size the other side declares.
- Strings you return must stay valid until the next call into your plugin; OHS copies them at
  once. If you return more findings than the buffer holds, OHS calls again with a bigger one.
- Return a negative number for an internal error: OHS reports it as a finding and carries on.

Solver and escapement plugins will use the same mechanism in later minor versions, with their own
entry points.
