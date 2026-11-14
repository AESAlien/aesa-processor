// SearchGrid 내용을 CSV로 출력하는 시각화용 도구
//
// 사용법: beam_table_dump [--matrix] [roll_deg pitch_deg yaw_deg]
// 출력  : idx,beamID,az_deg,el_deg,az_width_deg,el_width_deg   (ENU 기준 각도)
//
// visualize_beam_table.py 가 이 프로그램을 실행해서 결과를 그린다.

#include "search/search_grid.hpp"

#include <pose/domain/ant_enu_transform.hpp>
#include <pose/domain/factory/radar_attitude_factory.hpp>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>

int main(int argc, char** argv)
{
    using namespace math::literals;

    int first = 1;
    const auto printMatrix = argc > first && std::strcmp(argv[first], "--matrix") == 0;
    if (printMatrix)
    {
        ++first;
    }

    auto roll = 0_deg;
    auto pitch = 0_deg;
    auto yaw = 0_deg;
    if (argc - first == 3)
    {
        roll = math::Angle::fromDegrees(std::atof(argv[first]));
        pitch = math::Angle::fromDegrees(std::atof(argv[first + 1]));
        yaw = math::Angle::fromDegrees(std::atof(argv[first + 2]));
    }
    else if (argc - first != 0)
    {
        std::fprintf(stderr, "usage: %s [--matrix] [roll_deg pitch_deg yaw_deg]\n", argv[0]);
        return 2;
    }

    auto xform = [&]() {
        try
        {
            const pose::RadarAttitudeFactory factory(pose::RadarAttitudeFactoryConfig(89_deg));
            const auto att = factory.create(roll, math::TiltAngle(pitch), yaw);
            return pose::AntEnuTransform(
                att,
                pose::AntEnuTransformConfig(math::SquareMatrix::identity(3)));
        }
        catch (const std::invalid_argument& e)
        {
            std::fprintf(stderr, "makeAntToEnu failed: %s\n", e.what());
            std::exit(3);
        }
    }();

    if (printMatrix)
    {
        // 안테나 좌표축 세 개를 ENU로 변환하면 회전 행렬의 열을 얻는다.
        const std::array columns{
            xform.antToEnu(math::Vector({1.0, 0.0, 0.0})),
            xform.antToEnu(math::Vector({0.0, 1.0, 0.0})),
            xform.antToEnu(math::Vector({0.0, 0.0, 1.0}))
        };
        for (std::size_t row = 0; row < columns.size(); ++row)
        {
            std::printf("%.17g %.17g %.17g\n", columns[0](row), columns[1](row), columns[2](row));
        }
        return 0;
    }

    try
    {
        const auto config = beam::SearchGridConfig(
            21,
            9,
            math::CircularAngle(-42_deg),
            4.2_deg,
            math::TiltAngle(3_deg),
            4.4_deg,
            beam::BeamWidth(6_deg, 6_deg));
        auto grid = beam::SearchGrid(xform, config);
        const auto beamCount = config.azimuthCount() * config.elevationCount();
        std::printf("idx,beamID,az_deg,el_deg,az_width_deg,el_width_deg\n");
        for (std::size_t i = 0; i < beamCount; ++i)
        {
            const auto& b = grid.nextBeam();
            std::printf("%zu,%u,%.6f,%.6f,%.2f,%.2f\n",
                        i, b.beamId, b.direction_enu.azimuth.deg(), b.direction_enu.elevation.deg(),
                        b.beamWidth.azimuth.deg(), b.beamWidth.elevation.deg());
        }
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "SearchGrid failed: %s\n", e.what());
        return 4;
    }
    return 0;
}
