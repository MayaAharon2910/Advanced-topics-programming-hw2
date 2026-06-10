#pragma once

struct ScanResult {
    double distance;  // cm, 0 if too close; missing beams are omitted from ScanResult
    double azimuth_deg;
    double elevation_deg;
    double dir_x, dir_y, dir_z;  // unit direction vector
};