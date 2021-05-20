#include<stdint.h>


enum class OBJ_STATE {
OBJECT_FAULT,  // kein Objekt erkannt
NEW_OBJECT,    // neues Obejekt erkannt
OBJECT_GONE,   // verschwunden
OBJECT_FAST,   // ist noch da
OBJECT_SLOW,   // ist noch da
OBJECT_CLOSE   // ist noch da
};


// using OBJ_STATE = uint8_t;

// static constexpr OBJ_STATE OBJECT_FAULT = 1; // kein Objekt erkannt
// static constexpr OBJ_STATE NEW_OBJECT   = 1; // neues Obejekt erkannt
// static constexpr OBJ_STATE OBJECT_GONE  = 2; // verschwunden
// static constexpr OBJ_STATE OBJECT_FAST  = 3; // ist noch da
// static constexpr OBJ_STATE OBJECT_SLOW  = 4; // ist noch da
// static constexpr OBJ_STATE OBJECT_CLOSE = 5; // ist noch da


// Max: 224 x 224 --> Picture Size
OBJ_STATE Filter1(uint8_t i_width, uint8_t i_height, uint8_t i_score);
