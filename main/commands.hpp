#pragma once

namespace wallter {

// Keep numeric values stable to preserve behavior.
enum Command {
    CMD_STOP = 0,
    CMD_EXTEND = 1,
    CMD_RETRACT = 2,
    CMD_HOME = 3,
};

} // namespace wallter
