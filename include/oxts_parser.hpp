#pragma once

#include <print>
#include <string>
#include <vector>
#include <tuple>
#include <thread>
#include <future>
#include <filesystem>
#include <fstream>

namespace oxts_parser {
/**
 *
lat:   latitude of the oxts-unit (deg)
lon:   longitude of the oxts-unit (deg)
alt:   altitude of the oxts-unit (m)
roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
vn:    velocity towards north (m/s)
ve:    velocity towards east (m/s)
vf:    forward velocity, i.e. parallel to earth-surface (m/s)
vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
ay:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
af:    forward acceleration (m/s^2)
al:    leftward acceleration (m/s^2)
au:    upward acceleration (m/s^2)
wx:    angular rate around x (rad/s)
wy:    angular rate around y (rad/s)
wz:    angular rate around z (rad/s)
wf:    angular rate around forward axis (rad/s)
wl:    angular rate around leftward axis (rad/s)
wu:    angular rate around upward axis (rad/s)
pos_accuracy:  velocity accuracy (north/east in m)
vel_accuracy:  velocity accuracy (north/east in m/s)
navstat:       navigation status (see navstat_to_string)
numsats:       number of satellites tracked by primary GPS receiver
posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
49.015003823272 8.4342971002335 116.43032836914 0.035752 0.00903 -2.6087069803847 -6.811441479104 -11.275641809511 13.172716663769 -0.12475264293164 -0.032919903047354 -0.44519814607457 0.042957369847256 10.209865300506 -0.34030092211055 -0.31686915378551 10.209117821189 0.0090951755733632 -0.023140741253985 -0.017909034508194 0.0089018002187228 -0.022495299354602 -0.018809330937153 0.027658633371879 0.012727922061358 4 11 6 6 6

 */
struct OxtsData {
        float lat;
        float lon;
        float alt;
        float roll;
        float pitch;
        float yaw;
        float vn;
        float ve;
        float vf;
        float vl;
        float vu;
        float ax;
        float ay;
        float az;
        float af;
        float al;
        float au;
        float wx;
        float wy;
        float wz;
        float wf;
        float wl;
        float wu;
        float pos_accuracy;
        float vel_accuracy;
        int navstat;
        int numsats;
        int posmode;
        int velmode;
        int orimode;
};

OxtsData parse_oxts_file(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::string line;
    OxtsData data;
    // read a line
    std::getline(file, line);
    // parse the line
    if (sscanf(line.c_str(),
               "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %d %d %d", &data.lat,
               &data.lon, &data.alt, &data.roll, &data.pitch, &data.yaw, &data.vn, &data.ve, &data.vf, &data.vl,
               &data.vu, &data.ax, &data.ay, &data.az, &data.af, &data.al, &data.au, &data.wx, &data.wy, &data.wz,
               &data.wf, &data.wl, &data.wu, &data.pos_accuracy, &data.vel_accuracy, &data.navstat, &data.numsats,
               &data.posmode, &data.velmode, &data.orimode) != 30) {
        assert(false && "parsing otxs error");
    }

    return data;
}

std::vector<OxtsData> process_all_oxts_files(int n) {
    std::vector<OxtsData> datas;
    std::vector<std::future<void>> futures;
    datas.resize(n);
    for (size_t i = 0; i < n; i++) {
        futures.emplace_back(std::async(std::launch::async, [&, i]() {
            try {
                auto path = std::filesystem::path("../bin_datas/oxts/data/0000000" + std::format("{:03}", i) + ".txt");

                datas[i] = parse_oxts_file(path);
            } catch (const std::exception& e) { std::println("Error processing frame {}: {}", i, e.what()); }
        }));
    }

    for (auto& future : futures) { future.wait(); }
    return datas;
}

}; // namespace oxts_parser
