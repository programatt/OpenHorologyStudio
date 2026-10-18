// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdexcept>
#include <string>

namespace ohs {

/// Base class for all errors OHS reports to the user (bad input, missing file, ...).
class Error : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

}  // namespace ohs
