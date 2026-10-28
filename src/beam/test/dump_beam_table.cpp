// SearchGrid 내용을 CSV로 출력하는 시각화용 도구
//
// 사용법: beam_table_dump [--matrix] [roll_deg pitch_deg yaw_deg]
// 출력  : idx,beamID,az_deg,el_deg,az_width_deg,el_width_deg   (ENU 기준 각도)
//
// visualize_beam_table.py 가 이 프로그램을 실행해서 결과를 그린다.

#include "search_grid.hpp"

#include "attitude/ant_enu_transform.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>

int main(int argc, char** argv)
{
    using namespace math::literals;

    int first = 1;
    const bool printMatrix = argc > first && std::strcmp(argv[first], "--matrix") == 0;
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

    beam::AntEnuTransform xform = [&]() {
        try
        {
            const beam::RadarAttitude att(37_deg, 127_deg, 0.1_km, roll, pitch, yaw);
            return beam::AntEnuTransform(
                att,
                beam::AttitudeConfig(
                    89_deg,
                    math::SquareMatrix::identity(3),
                    math::Tolerance{1e-6, 0.0}));
        }
        catch (const std::invalid_argument& e)
        {
            std::fprintf(stderr, "makeAntToEnu failed: %s\n", e.what());
            std::exit(3);
        }
    }();

    if (printMatrix)
    {
        const auto& matrix = xform.rotationMatrix();
        for (std::size_t row = 0; row < matrix.rows(); ++row)
        {
            for (std::size_t column = 0; column < matrix.columns(); ++column)
            {
                std::printf("%.17g%c", matrix(row, column), column + 1 == matrix.columns() ? '\n' : ' ');
            }
        }
        return 0;
    }

    try
    {
        const beam::SearchGridConfig config(
            21, 9, -42_deg, 4.2_deg, 3_deg, 4.4_deg, 6_deg);
        beam::SearchGrid grid(xform, config);
        const std::size_t beamCount = config.azimuthCount * config.elevationCount;
        std::printf("idx,beamID,az_deg,el_deg,az_width_deg,el_width_deg\n");
        for (std::size_t i = 0; i < beamCount; ++i)
        {
            const beam::SearchBeam& b = grid.nextBeam();
            std::printf("%zu,%u,%.6f,%.6f,%.2f,%.2f\n",
                        i, b.beamId, b.direction_enu.azimuth.deg(), b.direction_enu.elevation.deg(),
                        b.azimuthBeamWidth.deg(), b.elevationBeamWidth.deg());
        }
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "SearchGrid failed: %s\n", e.what());
        return 4;
    }
    return 0;
}
