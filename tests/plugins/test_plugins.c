/* SPDX-License-Identifier: Apache-2.0 */
/* Plugins for testing the loader. Built several times with different -D options:
 *   TEST_PLUGIN_MANY   returns 40 findings (more than the loader's first buffer holds)
 *   TEST_PLUGIN_ERROR  returns a negative number from check()
 *   TEST_PLUGIN_NO_ENTRY exports nothing the loader looks for
 *   TEST_PLUGIN_INCOMPLETE returns a rule set without a check function
 */
#include <stdio.h>

#include "ohs/plugin.h"

#ifdef TEST_PLUGIN_NO_ENTRY
OHS_PLUGIN_EXPORT int ohs_something_else(void) { return 42; }
#else

static char texts[64][48];

static int32_t check_many(const ohs_part* part, ohs_finding* out, int32_t capacity, void* user) {
    (void)part;
    (void)user;
    for (int i = 0; i < 40 && i < capacity; ++i) {
        snprintf(texts[i], sizeof texts[i], "finding number %d", i);
        ohs_finding f = {sizeof(ohs_finding), "many", i % 3, texts[i], "fix it", (double)i, 0.0, (double)i, 0.0, 1};
        out[i] = f;
    }
    return 40;
}

static int32_t check_error(const ohs_part* part, ohs_finding* out, int32_t capacity, void* user) {
    (void)part;
    (void)out;
    (void)capacity;
    (void)user;
    return -7;
}

#if defined(TEST_PLUGIN_ERROR)
#define CHECK_FN check_error
#else
#define CHECK_FN check_many
#endif

#ifdef TEST_PLUGIN_INCOMPLETE
static const ohs_dfm_rule_set set = {sizeof(ohs_dfm_rule_set), OHS_PLUGIN_ABI_MAJOR, OHS_PLUGIN_ABI_MINOR, "incomplete", "no check function", "0", 0, 0};
#else
static const ohs_dfm_rule_set set = {sizeof(ohs_dfm_rule_set), OHS_PLUGIN_ABI_MAJOR, OHS_PLUGIN_ABI_MINOR, "test-plugin", "loader test plugin", "0.1", CHECK_FN, 0};
#endif

OHS_PLUGIN_EXPORT const ohs_dfm_rule_set* ohs_plugin_dfm_rule_set(void) { return &set; }
#endif
