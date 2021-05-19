#ifndef __TRANSMITTION_H_
#define __TRANSMITTION_H_

#include <istream>


//////////////////////////////////////////////////
//
// Makro Definitions
//
//////////////////////////////////////////////////
#define POS_X           0x00
#define POS_Y           0x08
#define POS_HEIGHT      0x10
#define POS_WIDTH       0x18
#define POS_SCORE       0x20

#define GET_VAL(VALUE, POS) \
((VALUE & (0xFFULL << POS)) >> POS)

namespace my {

    // static constexpr unsigned char flag = 0xAA;

    /**
     * @brief checks the inputstream if transmission can be started
     *
     * internally the flag will be repeatet four times resulting in
     * a 32 bits code.
     *
     * @note  this is a blocking function
     *
     * @param s     inputstream that will be checked
     * @param flag  when the pattern matches transmittion will be started
     *
     * @code        if(begin_transmit(i_stream, 0xAA))
     */
    void sync_receive_flag(std::istream& s, unsigned char flag);

    /**
     *@brief sends the appropriate formated flag that the counter
     *       part function can it recognize
     *
     *@param out  an output stream: can be ofstream or std::out or others
     *@param flag must be the same as the counter part function
     */
    void sync_send_flag(std::ostream& out, unsigned char flag);

}

#endif // __BEGIN_TRANSMITTION_H_
