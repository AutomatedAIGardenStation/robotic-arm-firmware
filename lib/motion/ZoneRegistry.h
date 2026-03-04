#ifndef ZONE_REGISTRY_H
#define ZONE_REGISTRY_H

struct ZoneEntry {
    char name[16];
    float angles[6];
};

bool resolve_zone(const char* name, float out_angles[6]);

#endif // ZONE_REGISTRY_H
