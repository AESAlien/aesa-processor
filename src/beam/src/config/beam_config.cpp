#include <beam/config/beam_config.hpp>

#include <utility>

namespace beam
{

BeamConfig::BeamConfig() : BeamConfig(Builder().build())
{
}

// 하위 설정은 각 생성자에서 이미 검증된 값이므로 여기서는 추가 검증이 없다.
BeamConfig::BeamConfig(
    AttitudeConfig attitude,
    SearchGridConfig searchGrid,
    RequestBeamConfig requestBeam
) : attitude(std::move(attitude)),
    searchGrid(std::move(searchGrid)),
    requestBeam(std::move(requestBeam))
{
}

BeamConfig BeamConfig::Builder::build() const
{
    return BeamConfig(
        _attitude.value_or(AttitudeConfig{}),
        _searchGrid.value_or(SearchGridConfig{}),
        _requestBeam.value_or(RequestBeamConfig{}));
}

} // namespace beam
