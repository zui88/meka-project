////////////////////////////////////////////////////////////////
//
// Header Sektion
//
////////////////////////////////////////////////////////////////
#include <detectFunction.h> // MATLAB Static Library
#include <convert.hpp>      // Own Library
#include <serializer.hpp>   // Own Library
#include <transmission.hpp> // Own Library

// Own Library are placed in "$ ./../dependencies"

#include <fmt/core.h>    // formatting library
#include <fmt/color.h>   // formatting library 
#include <chrono>        // for time measurements        
#include <thread>        // pause
#include <signal.h>      // Asynchronous Signals


////////////////////////////////////////////////////////////////
//
// Global Variables
//
////////////////////////////////////////////////////////////////

// Timings
static auto currentTime = std::chrono::steady_clock::now();
static double fps{0.0};

// Detection
static double x, y, width, height;
static float  score;

// for signal handler
static int detection_abort = 0;


////////////////////////////////////////////////////////////////
//
// Mode Deklarations --- State Machines
//
////////////////////////////////////////////////////////////////
void visual_mode();
void trans_mode();


////////////////////////////////////////////////////////////////
//
// Signal Handler --- for asynchronous interrupts
//
////////////////////////////////////////////////////////////////
void sig_handler(int sig);


////////////////////////////////////////////////////////////////
//
// Main Funktion
//
////////////////////////////////////////////////////////////////
int main(int argc, char const ** argv){ 


  ////////////////////////////////////////////////////////////////
  //
  // Init 
  //
  ////////////////////////////////////////////////////////////////
  detectFunction_init();

  // standart: transition mode
  enum struct state_e { v_mode, t_mode } state;
  state = state_e::t_mode;
  typedef void (*run_mode_call)(void);
  run_mode_call run_mode = nullptr;

  // Init Signal Handler
  // SIGINT -> ctrl+c
  signal(SIGINT, &sig_handler);

	
  ////////////////////////////////////////////////////////////////
  //
  // Read commandline input --- Command Line Parser (CLP)
  //
  ////////////////////////////////////////////////////////////////
  // exactly one argument
  if(argc > 1 && argc < 3) { 
    if(strcmp("--visual-mode", argv[1]) == 0)
      state = state_e::v_mode;
  }


  ////////////////////////////////////////////////////////////////
  //
  // Assigning mode
  //
  ////////////////////////////////////////////////////////////////
  switch(state) {
  case state_e::t_mode: {
    run_mode = &trans_mode;
    fmt::print(fg(fmt::color::honey_dew),
	       R"foo(

////////////////////////////////////////////////////////////////
//
// Raw Bit Transmission Mode
//
////////////////////////////////////////////////////////////////

)foo"
	       );
  } break;
  case state_e::v_mode: {
    run_mode = &visual_mode;
    fmt::print(fg(fmt::color::honey_dew),
	       R"foo(

////////////////////////////////////////////////////////////////
//
// Visual Mode
//
////////////////////////////////////////////////////////////////

)foo"
	       );
  } break;
  default: abort();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  // must be excecuted one time before regular detection for
  // proper initialization
  detectFunction(&x, &y, &width, &height, &score);
  // send flag and sign: now will be transmitted data
  // the flag '0xAA' has to correspond with the receiver
  // just 8 bit flag, then internally the flag will four times repeated
  my::sync_send_flag(std::cout, 0xAA);


  ////////////////////////////////////////////////////////////////
  //
  // Doom Loop -> There Is No Escape
  //
  ////////////////////////////////////////////////////////////////
  if (&run_mode != nullptr)
    {
      while(detection_abort == 0)
	{

	  ////////////////
	  //
	  // detection
	  //
	  ////////////////
	  detectFunction(&x, &y, &width, &height, &score);

	  run_mode();

	}
    }
  else
    {
      std::cerr << "no valid state" << std::endl;
    }

  fmt::print(fg(fmt::color::dark_golden_rod),
	     R"(

////////////////////////////////////////////////////////////////
//
// Detector Aborted
//
////////////////////////////////////////////////////////////////

)"
	     );
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  detectFunction_free();
  return 0;
}


////////////////////////////////////////////////////////////////
//
// State Definitions
//
////////////////////////////////////////////////////////////////

void visual_mode()
{
  ////////////////
  //
  // init measure time
  //
  ////////////////
  auto const newTime       = std::chrono::steady_clock::now();
  double const frameTime   = std::chrono::duration_cast<std::chrono::milliseconds>(newTime - currentTime).count();
  currentTime              = newTime;
  fps         = 1000 / frameTime;

  ////////////////
  //
  // detection
  //
  ////////////////
  if(score == -1)
    {
	
      ////////////////
      //
      // nothing detected
      //
      ////////////////
      fmt::print(fg(fmt::color::crimson),
		 "fps: {:.2f}\n", fps); 
	
    }
  else
    {
	
      ////////////////
      //
      // detected
      //
      ////////////////
      fmt::print(fg(fmt::color::dark_sea_green),
		 "fps: {:.2f}, x: {:.2f}, y: {:.2f}, width: {:.2f}, height: {:.2f}, score: {:.2f}\n", fps, x, y, width, height, score);
	
    }
	
}
	
	
void trans_mode()
{
	
  ////////////////
  //
  // init
  //
  ////////////////
  std::stringstream s;
  unsigned char xc, yc, widthc, heightc, scorec;
  // nothing detected -> 0x00000000
  unsigned long long val_to_send{0};
	
  ////////////////
  //
  // detection
  //
  ////////////////
  if (score == -1)
    {
	 
      ////////////////
      //
      // nothing detected
      //
      ////////////////
      std::cout << my::bits(val_to_send);
      std::cout.flush();
	 
    }
  else
    {
	 
      ////////////////
      //
      // detected
      //
      ////////////////
      // 5 * 8 Bit = 40 Bit
      xc=my::toc(x), yc=my::toc(y), widthc=my::toc(width), heightc=my::toc(height), scorec=my::toc(score * 100);
      // Single 40 bit value -> fifo
      s << heightc << widthc << yc << xc << scorec;
      // sizeof(unsigned long long) -> 64 Bit
      s >> my::bits(val_to_send);
      std::cout << my::bits(val_to_send);
      std::cout.flush();
	 
    }
	 
}


////////////////////////////////////////////////////////////////
//
// SigHandler Definition
//
////////////////////////////////////////////////////////////////

// SIGINT -> ctrl+c
void sig_handler(int sig)
{
  if (sig == SIGINT)
    {
      detection_abort = 1;
    }
}
