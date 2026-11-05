#include <beam/config/beam_config.hpp>

#include <type_traits>

namespace
{

static_assert(std::is_copy_assignable_v<beam::BeamConfig>);

} // namespace
