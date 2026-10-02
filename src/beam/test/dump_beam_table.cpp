// BeamTable 내용을 CSV로 출력하는 시각화용 도구
//
// 사용법: beam_table_dump [roll_deg pitch_deg yaw_deg]
// 출력  : idx,beamID,az_deg,el_deg,az_width_deg,el_width_deg   (안테나 기준 각도)
//
// visualize_beam_table.py 가 이 프로그램을 실행해서 결과를 그린다.

#include "beam_table.hpp"

#include <beam/domain/antenna_to_enu.hpp>
#include <beam/error/attitude_error.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>

int main(int argc, char** argv)
{
    int first = 1;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    if(argc - first == 3) {
        roll  = std::atof(argv[first]);
        pitch = std::atof(argv[first + 1]);
        yaw   = std::atof(argv[first + 2]);
    } else if(argc - first != 0) {
        std::fprintf(stderr, "usage: %s [roll_deg pitch_deg yaw_deg]\n", argv[0]);
        return 2;
    }

    beam::RadarAttitude att{};
    att.radar_lat_deg = 37.0;
    att.radar_lon_deg = 127.0;
    att.radar_alt_km  = 0.1;
    att.roll_deg      = roll;
    att.pitch_deg     = pitch;
    att.yaw_deg       = yaw;

    beam::AntennaToEnu xform = [&]() {
        try {
            return beam::AntennaToEnu(att, beam::AttitudeConfig{});
        } catch(const beam::AttitudeError& e) {
            std::fprintf(stderr, "makeAntennaToEnu failed: code=%d (%s)\n",
                         static_cast<int>(e.code()), e.what());
            std::exit(3);
        }
    }();

    try {
        const beam::BeamTable table(xform);
        std::printf("idx,beamID,az_deg,el_deg,az_width_deg,el_width_deg\n");
        for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
            const beam::BeamInfo& b = table.get(i);
            std::printf("%zu,%u,%.6f,%.6f,%.2f,%.2f\n",
                        i, b.beamID, b.az_deg, b.el_deg, b.az_width_deg, b.el_width_deg);
        }
    } catch(const std::exception& e) {
        std::fprintf(stderr, "BeamTable failed: %s\n", e.what());
        return 4;
    }
    return 0;
}
