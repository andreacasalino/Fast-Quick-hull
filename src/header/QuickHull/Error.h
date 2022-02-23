/**
 * Author:    Andrea Casalino
 * Created:   06.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <stdexcept>

namespace qh {
class Error : public std::runtime_error {
public:
  explicit Error(const std::string &what) : std::runtime_error(what){};
};
} // namespace qh
