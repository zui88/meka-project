#ifndef __CONV_H_
#define __CONV_H_

#include <math.h>

namespace my {

/// convert to integer
/// @return upper bound
template <class T> int toi(T val) { return int(ceil(val)); }

// bsp.
// float test {169.33};
// double test ...
// cout << "toi test -> 169.33: " << toi(test) << '\n';

/// convert to char
/// @return upper bound
template <typename T> char toc(T val) { return char(floor(val)); }

} // namespace my

#endif // __CONV_H_
