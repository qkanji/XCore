#include "xcore/command.hpp"
#include <utility>

namespace xcore {

Trigger along(double progress, std::function<void()> action) { return Trigger{progress, std::move(action)}; }

} // namespace xcore
