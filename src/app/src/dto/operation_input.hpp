#pragma once

#include <beam/dto/set_operation_state_command.hpp>

namespace app
{
// 수신 워커가 받은 운용 명령을 틱 워커로 전달한다.
using OperationInput = beam::SetOperationStateCommand;
} // namespace app
