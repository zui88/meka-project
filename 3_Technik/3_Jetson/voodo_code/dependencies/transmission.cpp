#include "transmission.hpp"
#include "serializer.hpp"
#include <ostream>

using my::bits;

void my::sync_receive_flag(std::istream &s, unsigned char flag) {

  unsigned char counter = 0, not_finished = 1, raw_c;
  enum struct state_e : unsigned char { init, valid } state;

  while (not_finished) {
    // sync read input stream to raw 8 bits, for char has 8 bits
    s >> bits(raw_c);
    if (raw_c == flag) {
      state = state_e::valid;
    } else {
      state = state_e::init;
    }

    switch (state) {
    case state_e::init: {
      counter = 0;
    } break;
    case state_e::valid: {
      ++counter;
      if (counter >= 4) {
        not_finished = 0;
      }
    } break;
    default:
      abort();
    }
  }
}

void my::sync_send_flag(std::ostream &out, unsigned char flag) {
  for (unsigned char i = 0; i < 4; ++i) {
    out << bits(flag);
  }
  out.flush();
}
