#include "filter1.hpp"
#include <iostream>
#include <sstream>
#include <string.h>

#include "../Lib/serializer.hpp"
#include "../Lib/transmission.hpp"


enum class OUTPUT { TEST, WORK };
static OUTPUT out_mode { OUTPUT::WORK };
static std::stringstream output_stream;

/// entscheidet zwischen test-mode (String output)
/// und work-mode (output serializer bit-weisse)
void my_out(int x, int y, int height, int width, const char *msg)
{
  if (out_mode == OUTPUT::TEST)
  {
    // im test mode einfach string ausgabe auf std::out
    std::cout << msg << std::endl;
  }
  else
  {
    // maxterm: sobald ein nicht 0, dann in ausfuehrungsblock
    if (x || y || height || width) {
      // Serializer
      // Raw String
      output_stream.clear();
      output_stream << x << y << height << width;
      std::cout << my::bits(output_stream);
      std::cout.flush();
    }
    // wenn alle 0, dann keine Nachricht auf stream
  }
}

int main(int argc, const char **argv) {

  // ToDo: Detector input simulate values
  if (argc == 2 && !strcmp(argv[1], "--test")) {
    out_mode = OUTPUT::TEST;
  }

  // check for beginning transmission
  my::sync_receive_flag(std::cin, 0xAA);

  unsigned long long receive;
  // doom loop
  for (;;) {

    // blocking read
    std::cin >> my::bits(receive);

    unsigned short x = GET_VAL(receive, POS_X);
    unsigned short y = GET_VAL(receive, POS_Y);
    unsigned short height = GET_VAL(receive, POS_HEIGHT);
    unsigned short width = GET_VAL(receive, POS_WIDTH);
    unsigned short score = GET_VAL(receive, POS_SCORE);

    // test input - for syntax correctness
    switch (Filter1(width, height, score)) {

    case OBJ_STATE::NEW_OBJECT:
      // neues objekt erkannt -> nachricht
      my_out(x, y, height, width, "New Objekt");
      break;

    case OBJ_STATE::OBJECT_FAULT:
      // kein Objekt erkannt -> ~nachricht
      my_out(0, 0, 0, 0, "Objekt Fault");
      break;

    case OBJ_STATE::OBJECT_CLOSE:
      // ist noch da, aber nahe -> nachricht
      my_out(x, y, height, width, "Objekt Close");
      break;

    case OBJ_STATE::OBJECT_FAST:
      // ist noch da & schnell -> nachricht
      my_out(x, y, height, width, "Objekt Fast");
      break;

    case OBJ_STATE::OBJECT_GONE:
      // verschwunden -> nachricht
      my_out(x, y, height, width, "Objekt Gone");
      break;

    case OBJ_STATE::OBJECT_SLOW:
      // noch da, aber langsam -> ~nachricht
      my_out(0, 0, 0, 0, "Objekt Slow");
      break;

    default:
      my_out(x, y, height, width, "No Handled State");
      break;
    }
  }

}
