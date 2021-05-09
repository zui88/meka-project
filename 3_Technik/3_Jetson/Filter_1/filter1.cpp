#include"filter1.hpp"


namespace {
    // Max 224 X 224 --> Frame
    struct Position { static const uint8_t Width_Treshold { 150 }, Height_Treshold { 150 }; };
    struct Velocity { static const uint8_t dW_Treshold { 10 }, dH_Treshold { 10 }; };
    enum class State { _Objekt, Objekt };
}


OBJ_STATE Filter1(uint8_t i_width, uint8_t i_height, uint8_t i_score)
{

    static State Filter_State { State::_Objekt };


    static uint8_t s_width_old { i_width }, s_height_old { i_height };
    uint8_t width_old { s_width_old }, height_old { s_height_old };
    s_width_old = i_width; s_height_old = i_height;


    switch(Filter_State)
    {
        case State::_Objekt :
        {

            if (i_score > 70)
            {

                // Message
                Filter_State = State::Objekt;
                return OBJ_STATE::NEW_OBJECT;

            }
            else
            {
                return OBJ_STATE::OBJECT_FAULT;
            }

        }; break; /* _Objekt */


        case State::Objekt :
        {

            if ( i_score < 60 )
            {

                // Message --- Objekt gone
                Filter_State = State::_Objekt;
                return OBJ_STATE::OBJECT_GONE;

            }

            // Objekt weit genug weg -> Geschwindigkeit pruefen
            if ( i_width <= Position::Width_Treshold && i_height <= Position::Height_Treshold )
            {

                //Filter_State = State::V;
                {

                    // Betrag
                    auto abs = [](int16_t val) -> uint8_t { return val<0?val*-1:val; };
                    uint8_t dW { abs(width_old - i_width) }, dH { abs(height_old - i_height) };


                    // V > V_Tresh
                    if ( dW > Velocity::dW_Treshold && dH > Velocity::dH_Treshold )
                    {

                        // Message
                        return OBJ_STATE::OBJECT_FAST;

                    }

                    // V <= V_Tresh
                    //if ( dW <= Velocity::dW_Treshold && dH <= Velocity::dH_Treshold )
                    else
                    {

                        // No Message
                        return OBJ_STATE::OBJECT_SLOW;

                    }

                }

            }

            // Objekt zu nah
            else
            {

                return OBJ_STATE::OBJECT_CLOSE;

            }

        }; break; /* Objekt */

        default: return OBJ_STATE::OBJECT_FAULT;

    } /* FSM --- switch-case */

}
