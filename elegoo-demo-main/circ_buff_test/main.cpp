#include <iostream>

#include "circular_buffer.h"


volatile circular_buffer buffer1;

int main()
{
  std::cout << "pushing 1" << std::endl;
  push(1, &buffer1);

  std::cout << "popping 1" << std::endl;
  int val = pop(&buffer1);

  std::cout << val << std::endl;
  return 0;
}