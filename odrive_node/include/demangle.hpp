#ifndef DEMANGLE_HPP
#define DEMANGLE_HPP

#include <typeinfo>
#include <cxxabi.h>
#include <map>
#include <string>

std::string demangle(char const* mangle);

#endif // DEMANGLE_HPP
