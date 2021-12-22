/**
 * Author:    Andrea Casalino
 * Created:   06.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef QHULL_ERROR_H
#define QHULL_ERROR_H

#include <stdexcept>

namespace qh {
    /** @brief A runtime error that can be raised when using any object in qh::
	 */
    class Error : public std::runtime_error {
    public:
        explicit Error(const std::string& what) : std::runtime_error(what) {
        };
    };
}

#endif
