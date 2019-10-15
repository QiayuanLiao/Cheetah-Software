/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "TeleOperat_Controller.hpp"

int main(int argc, char** argv) {
  main_helper(argc, argv, new TeleOperat_Controller());
  return 0;
}
