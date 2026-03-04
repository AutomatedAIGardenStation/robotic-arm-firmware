#include "ZoneRegistry.h"
#include "../../config/Config.h"
#include <string.h>

static const ZoneEntry ZONE_REGISTRY[] = {
    { "home",    ZONE_HOME_ANGLES },
    { "inspect", ZONE_INSPECT_ANGLES },
    { "harvest", ZONE_HARVEST_ANGLES },
    { "deposit", ZONE_DEPOSIT_ANGLES }
};

static const int NUM_ZONES = sizeof(ZONE_REGISTRY) / sizeof(ZONE_REGISTRY[0]);

bool resolve_zone(const char* name, float out_angles[6]) {
    if (!name || !out_angles) return false;

    for (int i = 0; i < NUM_ZONES; i++) {
        if (strncmp(name, ZONE_REGISTRY[i].name, sizeof(ZONE_REGISTRY[i].name)) == 0) {
            for (int j = 0; j < 6; j++) {
                out_angles[j] = ZONE_REGISTRY[i].angles[j];
            }
            return true;
        }
    }
    return false;
}
