
#ifndef CONENIENCE_HELPERS_HPP
#define CONENIENCE_HELPERS_HPP

#include <filesystem>
#include <iostream>
#include <string>

namespace helpers {
inline auto validate_filename(std::string filename) -> std::string
{
    if (!std::filesystem::exists(filename)) {
        std::string errorMessage = "File not found: " + filename;
        throw std::runtime_error(errorMessage);
    }
    std::cout << "Found file: " << filename << "\n";

    return filename;
}
};  // namespace helpers
#endif
