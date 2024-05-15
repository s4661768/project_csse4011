/* GCU COMMAND INTERFACE */

#include "commands.h"
#include "../hci/hci.h"

/*
DEVICE ID | Device
--------------------
    1     |   User LEDs (gled)
    2     |   Pushbutton (gpb)
    3     |   DAC (dac)
    4     |   Lissajous Curve Parameters (lj)
    5     |   Point (point)
    6     |   Circle (circle)
*/

#define GLED_DEVICE_ID 1
#define GPB_DEVICE_ID 2
#define DAC_DEVICE_ID 3
#define LJ_DEVICE_ID 4
#define POINT_DEVICE_ID 9
#define CIRCLE_DEVICE_ID 10

// #define HTS221_TEMPERATURE_ID 5
// #define HTS221_HUMIDITY_ID 6
// #define LPS22_AIR_PRESSURE_ID 7
// #define CCS811_TVOC_ID 8
// #define SD_CARD_ID 13

LOG_MODULE_DECLARE(AHU);

/* function cmd_handle_point()
 * −−−−−−−−−−−−−−−
 * Draw a point at [x,y]
 *
 * argc: Number of arguments
 * argv: Array of arguments
 *
 * Returns: 0 on success, -1 on failure
 *
 * Errors: Prints error messages if invalid number of arguments or
 *   invalid point values */
int cmd_handle_point(const struct shell *sh, size_t argc, char **argv) {
  // Check the number of arguments
  if (argc != 3) {
    LOG_ERR("point_module: point <x> <y>");
    return -1;
  }

  int x = atoi(argv[1]);
  int y = atoi(argv[2]);

  // Check second and third argument for invalid values
  if (x < 0 || x > 4095 || y < 0 || y > 4095) {
    LOG_ERR("point_module: invalid point values");
    return -1;
  }

  // Create a string to represent the point coordinates
  char message[15]; // Maximum 15 bytes for the message
  snprintf(message, sizeof(message), "%d%d,%d", POINT_DEVICE_ID, x, y);

  // Add the message to the HCI queue
  add_to_hci_queue(message, true);

  return 0;
}

SHELL_CMD_ARG_REGISTER(point, NULL, "Draw a point at [x,y]", cmd_handle_point,
                       3, 0);

/* function cmd_handle_circle()
 * −−−−−−−−−−−−−−−
 * Draw a circle with radius r with a center at [x,y]
 *
 * argc: Number of arguments
 * argv: Array of arguments
 *
 * Returns: 0 on success, -1 on failure
 *
 * Errors: Prints error messages if invalid number of arguments or
 *   invalid point values */
int cmd_handle_circle(const struct shell *sh, size_t argc, char **argv) {
  // Check the number of arguments
  if (argc != 4) {
    LOG_ERR("Usage: circle <r> <x> <y>");
    return -1;
  }

  int r = atoi(argv[1]);
  int x = atoi(argv[2]);
  int y = atoi(argv[3]);

  // Check second, third and fourth argument for invalid values
  if (r < 0 || r > 33 || x < 0 || x > 4095 || y < 0 || y > 4095) {
    LOG_ERR("circle_module: invalid points"); // Log error message
    return -1;
  }

  // Create a string to represent the point coordinates
  char message[15]; // Maximum 15 bytes for the message
  snprintf(message, sizeof(message), "%d%d,%d,%d", CIRCLE_DEVICE_ID, r, x, y);

  // Add the message to the HCI queue
  add_to_hci_queue(message, true);

  return 0;
}

SHELL_CMD_ARG_REGISTER(circle, NULL,
                       "Draw a circle with radius r with a center at [x,y]",
                       cmd_handle_circle, 4, 0);

/* function cmd_handle_lj()
 * −−−−−−−−−−−−−−−
 * Set the a and b parameters for the Lissajous Curve
 *
 * argc: Number of arguments
 * argv: Array of arguments
 *
 * Returns: 0 on success, -1 on failure
 *
 * Errors: Prints error messages if invalid number of arguments or
 *   invalid points */
int cmd_handle_lj(const struct shell *sh, size_t argc, char **argv) {
  // Check the number of arguments
  if (argc != 4) {
    LOG_ERR("Usage: lj <a> <b> <t>");
    return -1;
  }

  // Convert arguments to integers
  int a = atoi(argv[1]);
  int b = atoi(argv[2]);
  int t = atoi(argv[3]);

  // Check second and third argument for invalid values
  if (a < 0 || a > 99 || b < 0 || b > 99) {
    LOG_ERR("lj_module: invalid points"); // Log error message
    return -1;
  }

  // Check fourth argument for invalid values
  if (t < 0 || t > 628) {
    LOG_ERR("lj_module: invalid points"); // Log error message
    return -1;
  }

  // Create a string to represent the point coordinates
  char message[15]; // Maximum 15 bytes for the message
  snprintf(message, sizeof(message), "%d%d,%d,%d", LJ_DEVICE_ID, a, b, t);

  // Add the message to the HCI queue
  add_to_hci_queue(message, true);

  return 0;
}

SHELL_CMD_ARG_REGISTER(
    lj, NULL, "Set the a, b AND t parameters for the Lissajous Curve for GCU",
    cmd_handle_lj, 4, 0);

/* function cmd_handle_dac()
 * −−−−−−−−−−−−−−
 * Set the voltage values on the X and Y DACs
 *
 * argc: Number of arguments
 * argv: Array of arguments
 *
 * Returns: 0 on success, -1 on failure
 *
 * Errors: Prints error messages if invalid number of arguments or
 *   invalid point values */
int cmd_handle_dac(const struct shell *sh, size_t argc, char **argv) {
  // Check the number of arguments
  if (argc != 3) {
    LOG_ERR("dac_module: dac <x> <y>");
    return -1;
  }

  // Convert arguments to integers
  int x = atoi(argv[1]);
  int y = atoi(argv[2]);

  // Check the second and third argument for invalid values
  if (x < 0 || x > 33 || y < 0 || y > 33) {
    LOG_ERR("dac_module: invalid point values");
    return -1;
  }

  // Create a string to represent the point coordinates
  char message[15]; // Maximum 15 bytes for the message
  snprintf(message, sizeof(message), "%d%d,%d", DAC_DEVICE_ID, x, y);

  // Add the message to the HCI queue
  add_to_hci_queue(message, true);

  return 0;
}

SHELL_CMD_ARG_REGISTER(dac, NULL,
                       "Set the voltage values on the X and Y DACs for GCU",
                       cmd_handle_dac, 3, 0);

/* function cmd_handle_gled()
 * −−−−−−−−−−−−−−−
 * Toggle the GCU User LEDs
 *
 * argc: Number of arguments
 * argv: Array of arguments
 *
 * Returns: 0 on success, -1 on failure
 *
 * Errors: Prints error messages if invalid number of arguments or
 *   invalid rgb values */
int cmd_handle_gled(const struct shell *sh, size_t argc, char **argv) {
  // Check the number of arguments
  if (argc != 4) {
    LOG_ERR("gled_module: rgb <r> <g> <b>");
    return -1;
  }

  // Convert arguments to integers
  int r = atoi(argv[1]);
  int g = atoi(argv[2]);
  int b = atoi(argv[3]);

  // Check the second and third argument for invalid values
  if (r < 0 || r > 1 || g < 0 || g > 1 || b < 0 || b > 1) {
    LOG_ERR("gled_module: invalid rgb values");
    return -1;
  }

  // Create a string to represent the point coordinates
  char message[15]; // Maximum 15 bytes for the message
  snprintf(message, sizeof(message), "%d%d,%d,%d", GLED_DEVICE_ID, r, g, b);

  // Add the message to the HCI queue
  add_to_hci_queue(message, true);

  return 0;
}

SHELL_CMD_ARG_REGISTER(gled, NULL,
                       "Toggle the GCU User LEDs -> gled <r> <g> <b>",
                       cmd_handle_gled, 4, 0);

/* function cmd_handle_gpb()
 * −−−−−−−−−−−−−−
 * Read GCU pushbutton state (0 or 1)
 *
 * argc: Number of arguments
 * argv: Array of arguments
 *
 * Returns: 0 on success, -1 on failure
 *
 * Errors: Prints error messages if invalid number of arguments */
int cmd_handle_gpb(const struct shell *sh, size_t argc, char **argv) {
  // Check the number of arguments
  if (argc != 1) {
    LOG_ERR("Usage: gpb");
    return -1;
  }

  // Create a string to represent the point coordinates
  char message[4]; // Maximum 4 bytes
  snprintf(message, sizeof(message), "%d", GPB_DEVICE_ID);

  // Add the message to the HCI queue
  add_to_hci_queue(message, true);

  return 0;
}

SHELL_CMD_ARG_REGISTER(gpb, NULL, "Read GCU pushbutton state (0 or 1)",
                       cmd_handle_gpb, 1, 0);


                