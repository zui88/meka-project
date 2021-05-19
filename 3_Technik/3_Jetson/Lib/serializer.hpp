#ifndef __SERIALIZER_H_
#define __SERIALIZER_H_

#include <sstream>
#include <iostream>

namespace my {

    // holding value -> underlying type
    template<typename T> struct bits_t { T t; };

    // function for infer type -> l value
    template<typename T> bits_t<T&> bits(T& val) {
        return bits_t<T&>{val};
    }
    // for const
    template<typename T> bits_t<const T&> bits(const T& val) {
        return bits_t<const T>{val};
    }

    template<typename S, typename T>
    S& operator<< (S &s, const bits_t<T&> t) {
        return s.write(reinterpret_cast<const char *>(&t.t), sizeof(T));
    }

    template<typename S, typename T>
    S& operator>> (S& s, bits_t<T&> t) {
        return s.read(reinterpret_cast<char *>(&t.t), sizeof(T));
    }

    // stringstream specialization
    template<typename T>
    void operator>> (std::stringstream& s, bits_t<T&> t) {
        s.read(reinterpret_cast<char *>(&t.t), sizeof(T));
    }

}

#endif // __SERIALIZER_H_
