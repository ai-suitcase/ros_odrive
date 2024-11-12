#include "demangle.hpp"

std::string demangle(char const* mangle) {
    int status = 1;
    char* dem = abi::__cxa_demangle(mangle, 0, 0, &status);
    std::string ret;
    if (status == 0) {
        ret = std::string(dem);
    } else {
        ret = std::string("");
    }
    std::free(dem);
    return ret;
}
