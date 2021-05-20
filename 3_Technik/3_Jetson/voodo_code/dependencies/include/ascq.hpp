/////////////////////////////////////////////////////////////////////////
// Asynchronous Command Queue - ASCQ
//
// Created by Marco Burkhardt - 06/09/2020
//
// Version 0.1.1
//
/////////////////////////////////////////////////////////////////////////


#ifndef ASCQ_HPP
#define ASCQ_HPP


#include <thread>
#include "ring.hpp"


__BEGIN_MY__


    /////////////////////////////////////////////////////////////////////////
    // Interface
    //
    // every concrete command needs to implement this interface
    //
    /////////////////////////////////////////////////////////////////////////
    struct Command {
       virtual void excecute() = 0;
    };


    /////////////////////////////////////////////////////////////////////////
    // Async command queue
    //
    // Manages the incoming commands and invokes them asynchronously by ASCQ::invoke()
    //
    /////////////////////////////////////////////////////////////////////////
    template<size_t SIZE, class T = Command*> class ASCQ final {
    private:
        my::Ring<T, SIZE> ring;

    public:
        /////////////////////////////////////////////////////////////////////////
        // Attach command
        //
        /////////////////////////////////////////////////////////////////////////
        template<class Type> void assign() { ring.push(new Type); }

        /////////////////////////////////////////////////////////////////////////
        // Async invoker
        //
        /////////////////////////////////////////////////////////////////////////
        void invoke();

    };


    /////////////////////////////////////////////////////////////////////////
    // Definitions
    //
    /////////////////////////////////////////////////////////////////////////
    template<size_t SIZE, class T>
    void ASCQ<SIZE, T>::invoke() {
        while (!ring.empty()) {
            std::thread t(
                    [](T command) {
                        command->excecute();
                        delete command;
                    },
                    ring.next());
            t.detach();
        }
    }


    /////////////////////////////////////////////////////////////////////////
    // Specialization
    //
    /////////////////////////////////////////////////////////////////////////
    using AQ8 = ASCQ<256>;
    typedef ASCQ<8> AQ4;


    /////////////////////////////////////////////////////////////////////////
    // Macro
    //
    /////////////////////////////////////////////////////////////////////////
    #define CATCH_OVERFLOW(...)                     \
        try                                         \
        {                                           \
        __VA_ARGS__;                                \
        }                                           \
        catch (std::overflow_error error)           \
        {                                           \
            std::cout << error.what() << std::endl; \
        }


__END_MY__


#endif // ASCQ_HPP
