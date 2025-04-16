/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2024 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * controller_tinympc.c - Initial setup for TinyMPC controller integration.
 * Currently just wraps the PID controller as a starting point.
 */

#include "Eigen.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC"
#include "debug.h"

#include "controller.h"
#include "controller_pid.h"
#include "param.h"

#include "tinympc/admm.hpp"
#include "tinympc/types.hpp"

// minimal data structures needed, just to call a function
// the data types are different now!!!!!
// basically just memset
static TinyCache cache = TinyCache{};
static TinySettings settings = TinySettings{};
static TinyWorkspace workspace = TinyWorkspace{};
static TinySolver solver = TinySolver{};
static TinySolution solution = TinySolution{};

void controllerOutOfTreeInit(void) {
	
  // Initialize TinyMPC structures to zero or default
  solver.cache = &cache;
  solver.settings = &settings;
  solver.work = &workspace;
  solver.solution = &solution;

  DEBUG_PRINT("TinyMPC structures initialized\n");
  controllerPidInit(); // Keep PID for safety
}

bool controllerOutOfTreeTest(void) {
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, 
                        const sensorData_t *sensors, const state_t *state, 
                        const stabilizerStep_t stabilizerStep) {
  // Example: call the solver (with dummy data for now)
  solve(&solver);
  // For now, still use PID for output
  controllerPid(control, setpoint, sensors, state, stabilizerStep);
}

// Initialize the controller when the app starts
void appMain(void) {
  DEBUG_PRINT("Waiting for activation ...\n");
  
  // Set the controller type to out-of-tree

  // This is using the up to date method
  paramVarId_t controllerType = paramGetVarIdFromComplete("stabilizer.controller");
  paramSetInt(controllerType, ControllerTypeOot);
  
  while(1) {
    vTaskDelay(M2T(2000));
  }
}

#ifdef __cplusplus
} /* extern "C" */
#endif