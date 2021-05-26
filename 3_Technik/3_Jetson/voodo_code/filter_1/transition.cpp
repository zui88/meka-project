#include "filter1.hpp"
#include <iostream>
#include <sstream>
#include <string.h>

#include "../dependencies/include/serializer.hpp"
#include "../dependencies/include/transmission.hpp"


enum class OUTPUT { TEST, WORK };
static OUTPUT out_mode { OUTPUT::WORK };
static std::stringstream output_stream;


///////////////////////////////////////////
//
// Helper Functions
//
///////////////////////////////////////////

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
    // if (x || y || height || width) {
      // Serializer
      // Raw String
      output_stream.clear();
      output_stream << x << y << height << width;
      std::cout << my::bits(output_stream);
      std::cout.flush();
    // }
    // wenn alle 0, dann keine Nachricht auf stream
  }
}


///////////////////////////////////////////
//
// Main
//
///////////////////////////////////////////

int main(int argc, const char **argv) {

  ///////////////////////////////////////////
  //
  // Parse Command Line
  //
  ///////////////////////////////////////////
  if (argc == 2 && !strcmp(argv[1], "--visual-mode")) {
    out_mode = OUTPUT::TEST;
  }

  ///////////////////////////////////////////
  //
  // Check For Beginning Transmission
  //
  ///////////////////////////////////////////
  my::sync_receive_flag(std::cin, 0xAA);

  // receive buffer: raw bit receive
  unsigned long long receive {0};

  // store last objekt state
  OBJ_STATE last_state {OBJ_STATE::OBJECT_FAULT};
  OBJ_STATE actual_state {OBJ_STATE::OBJECT_FAULT};

  ///////////////////////////////////////////
  //
  // Doom Loop
  //
  ///////////////////////////////////////////
  for (;;) {

    // blocking read
    std::cin >> my::bits(receive);

    // order: [x y height width score]
    unsigned short x = GET_VAL(receive, POS_X);
    unsigned short y = GET_VAL(receive, POS_Y);
    unsigned short height = GET_VAL(receive, POS_HEIGHT);
    unsigned short width = GET_VAL(receive, POS_WIDTH);
    unsigned short score = GET_VAL(receive, POS_SCORE);


    last_state = actual_state;
    actual_state = Filter1(width, height, score);


    switch (actual_state) {

    case OBJ_STATE::NEW_OBJECT:
      // neues objekt erkannt -> nachricht
      my_out(x, y, height, width, "New Objekt");
      break;

    case OBJ_STATE::OBJECT_FAULT:
      // kein Objekt erkannt -> ~nachricht
      // nur einmal nachricht ausgeben
      if (last_state != actual_state)
      {
        my_out(0, 0, 0, 0, "Objekt Fault");
      }
      break;

    case OBJ_STATE::OBJECT_CLOSE:
      // ist noch da, aber nahe -> nachricht
      // jedes mal nachricht ausgeben
      my_out(x, y, height, width, "Objekt Close");
      break;

    case OBJ_STATE::OBJECT_FAST:
      // ist noch da & schnell -> nachricht
      // nur einmal nachricht ausgeben
      if (last_state != actual_state)
      {
        my_out(x, y, height, width, "Objekt Fast");
      }
      break;

      //ToDo: evtl. spezieller byte-code fuer diesen fall..?
    case OBJ_STATE::OBJECT_GONE:
      // verschwunden -> nachricht
      my_out(x, y, height, width, "Objekt Gone");
      break;

    case OBJ_STATE::OBJECT_SLOW:
      // noch da, aber langsam -> ~nachricht
      // nur einmal nachricht ausgeben
      if (last_state != actual_state)
      {
        my_out(0, 0, 0, 0, "Objekt Slow");
      }

      break;

    default:
      my_out(x, y, height, width, "No Handled State");
      break;
    }
  }
}
