#include <beam/config/beam_config.hpp>

#include <stdexcept>
#include <utility>

namespace beam
{

// 하위 설정은 각 생성자에서 이미 검증된 값이므로 여기서는 추가 검증이 없다.
BeamConfig::BeamConfig(
    AttitudeConfig attitude,
    SearchGridConfig searchGrid,
    RequestBeamConfig requestBeam,
    RequestCoverageConfig requestCoverage
) : attitude(std::move(attitude)),
    searchGrid(std::move(searchGrid)),
    requestBeam(std::move(requestBeam)),
    requestCoverage(std::move(requestCoverage))
{
}

BeamConfig BeamConfig::Builder::build() const
{
    if (!_attitude || !_searchGrid || !_requestBeam || !_requestCoverage)
    {
        throw std::logic_error("BeamConfig::Builder requires every value");
    }

    return BeamConfig(
        *_attitude,
        *_searchGrid,
        *_requestBeam,
        *_requestCoverage);
}

} // namespace beam
