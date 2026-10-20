#include "scheduler/request_beam_policy_factory.hpp"

#include <stdexcept>

namespace aesa_processor::beam
{
namespace
{

class ConfirmationBeamPolicy final : public IRequestBeamPolicy
{
public:
    explicit ConfirmationBeamPolicy(aesa_processor::math::Angle beamWidth) : _beamWidth(beamWidth)
    {
    }

    BeamType transmitBeamType() const override
    {
        return BeamType::CONFIRMATION;
    }

    aesa_processor::math::Angle beamWidth() const override
    {
        return _beamWidth;
    }

    int priority() const override
    {
        return 1;
    }

private:
    aesa_processor::math::Angle _beamWidth;
};

class TrackingBeamPolicy final : public IRequestBeamPolicy
{
public:
    explicit TrackingBeamPolicy(aesa_processor::math::Angle beamWidth) : _beamWidth(beamWidth)
    {
    }

    BeamType transmitBeamType() const override
    {
        return BeamType::TRACKING;
    }

    aesa_processor::math::Angle beamWidth() const override
    {
        return _beamWidth;
    }

    int priority() const override
    {
        return 0;
    }

private:
    aesa_processor::math::Angle _beamWidth;
};

} // namespace

RequestBeamPolicyFactory::RequestBeamPolicyFactory(const RequestBeamConfig& config)
    : _confirmationPolicy(std::make_shared<const ConfirmationBeamPolicy>(config.confirmationBeamWidth())),
      _trackingPolicy(std::make_shared<const TrackingBeamPolicy>(config.trackingBeamWidth()))
{
}

// 요청 빔 종류에 따른 분기는 여기에만 둔다.
std::shared_ptr<const IRequestBeamPolicy> RequestBeamPolicyFactory::create(RequestBeamType beamType) const
{
    switch (beamType)
    {
    case RequestBeamType::CONFIRMATION:
        return _confirmationPolicy;

    case RequestBeamType::TRACKING:
        return _trackingPolicy;
    }

    throw std::invalid_argument("Unknown beam request type");
}

} // namespace aesa_processor::beam
