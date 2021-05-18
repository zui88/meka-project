#include "filter1.hpp"
#include <iostream>
#include <string.h>


int main(int argc, const char **argv) {

  // ToDo: Detector input simulate values
  if(!strcmp(argv[1], "--test"))
  {

  }


  switch (Filter1(2, 2, 2)) {
  case OBJ_STATE::NEW_OBJECT:
    std::cout << "New Objekt\n";
    break;
  case OBJ_STATE::OBJECT_FAULT:
    std::cout << "Objekt Fault\n";
    break;

  case OBJ_STATE::OBJECT_CLOSE:
    std::cout << "Objekt Close\n";
    break;

  case OBJ_STATE::OBJECT_FAST:
    std::cout << "Objekt Fast\n";
    break;

  case OBJ_STATE::OBJECT_GONE:
    std::cout << "Objekt Gone\n";
    break;

  case OBJ_STATE::OBJECT_SLOW:
    std::cout << "Objekt Slow\n";
    break;

    default: std::cout << "No Handled State\n"; break;
  }
}
