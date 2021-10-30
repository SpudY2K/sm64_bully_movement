#ifndef INT128_HPP
#define INT128_HPP

#include <algorithm>
#include <string>

using int128_t = __int128;
using uint128_t = unsigned __int128;

inline std::string to_string(uint128_t x) {
  std::string out;
  out.reserve(40);
  while (x > 0) {
    out.push_back((x % 10) | 0x30);
    x /= 10;
  }
  std::reverse(out.begin(), out.end());
  return out;
}

#endif