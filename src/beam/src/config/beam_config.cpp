#include <beam/config/beam_config.hpp>

#include <utility>

namespace aesa_processor::beam
{

BeamConfig::BeamConfig() : BeamConfig(Builder().build())
{
}

// 하위 설정은 각 생성자에서 이미 검증된 값이므로 여기서는 추가 검증이 없다.
BeamConfig::BeamConfig(
    AttitudeConfig attitude,
    SearchGridConfig searchGrid,
    RequestBeamConfig requestBeam
) : _attitude(std::move(attitude)),
    _searchGrid(std::move(searchGrid)),
    _requestBeam(std::move(requestBeam))
{
}

const AttitudeConfig& BeamConfig::attitude() const noexcept
{
    return _attitude;
}

const SearchGridConfig& BeamConfig::searchGrid() const noexcept
{
    return _searchGrid;
}

const RequestBeamConfig& BeamConfig::requestBeam() const noexcept
{
    return _requestBeam;
}

BeamConfig BeamConfig::Builder::build() const
{
    return BeamConfig(_attitude, _searchGrid, _requestBeam);
}

} // namespace aesa_processor::beam
