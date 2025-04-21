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

// Add logging group for MPC metrics
#include "log.h"

// minimal data structures needed, just to call a function
// the data types are different now!!!!!
// basically just memset
static TinyCache cache = TinyCache{};
static TinySettings settings = TinySettings{};
static TinyWorkspace workspace = TinyWorkspace{};
static TinySolver solver = TinySolver{};
static TinySolution solution = TinySolution{};

// now tinyVector, not tiny_VectorNx
static tinyVector mpc_setpoint;

// Helper function to convert quaternion to Rodrigues parameters
static inline vec3_s quat_2_rp(quaternion_t q)
{
    vec3_s v;
    v.x = q.x / q.w;
    v.y = q.y / q.w;
    v.z = q.z / q.w;
    return v;
}

// Define problem dimensions
#define NSTATES 12
#define NINPUTS 4
#define NHORIZON 20

// Add logging variables for MPC metrics
// LOG_GROUP_START(tinympc)
// LOG_ADD(LOG_INT32, solver_iterations, &solver.solution->iter)
// LOG_ADD(LOG_INT32, solver_solved, &solver.solution->solved)
// LOG_GROUP_STOP(tinympc)

void controllerOutOfTreeInit(void) {
	
  // Initialize TinyMPC structures to zero or default
  solver.cache = &cache;
  solver.settings = &settings;
  solver.work = &workspace;
  solver.solution = &solution;

  // Set up workspace dimensions
  workspace.nx = NSTATES;
  workspace.nu = NINPUTS;
  workspace.N = NHORIZON;
  
  // Initialize settings with reasonable defaults
  settings.abs_pri_tol = 1e-3;
  settings.abs_dua_tol = 1e-3;
  settings.max_iter = 10;
  settings.check_termination = 1;
  settings.en_state_bound = 0;
  settings.en_input_bound = 1;
  
  // Initialize cache with default values
  cache.rho = 1.0;
  
  // Initialize solution
  solution.x = tinyMatrix::Zero(NSTATES, NHORIZON);
  solution.u = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  solution.iter = 0;
  solution.solved = 0;

  DEBUG_PRINT("TinyMPC structures initialized\n");
  controllerPidInit(); // Keep PID for safety
}

bool controllerOutOfTreeTest(void) {
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, 
                        const sensorData_t *sensors, const state_t *state, 
                        const stabilizerStep_t stabilizerStep) {
  // Update reference trajectory based on setpoint
  tinyVector ref_state = tinyVector::Zero(NSTATES);
  
  // Set position reference (first 3 elements)
  ref_state(0) = setpoint->position.x;
  ref_state(1) = setpoint->position.y;
  ref_state(2) = setpoint->position.z;
  
  // Set yaw reference (5th element)
  ref_state(5) = setpoint->attitude.yaw;
  
  // Replicate this reference across the horizon
  for (int i = 0; i < NHORIZON; i++) {
    workspace.Xref.col(i) = ref_state;
  }
  
  // Set initial state for MPC solver
  tinyVector current_state = tinyVector::Zero(NSTATES);
  
  // Set position (first 3 elements)
  current_state(0) = state->position.x;
  current_state(1) = state->position.y;
  current_state(2) = state->position.z;
  
  // Convert quaternion to Rodrigues parameters (elements 3-5)
  vec3_s phi = quat_2_rp(state->attitudeQuaternion);
  current_state(3) = phi.x;
  current_state(4) = phi.y;
  current_state(5) = phi.z;
  
  // Set velocity (elements 6-8)
  current_state(6) = state->velocity.x;
  current_state(7) = state->velocity.y;
  current_state(8) = state->velocity.z;
  
  // Set angular velocity (elements 9-11)
  current_state(9) = sensors->gyro.x;
  current_state(10) = sensors->gyro.y;
  current_state(11) = sensors->gyro.z;
  
  // Set the initial state in the workspace
  workspace.x.col(0) = current_state;
  
  // Example: call the solver (with dummy data for now)
  solve(&solver);
  // For now, still use PID for output
  controllerPid(control, setpoint, sensors, state, stabilizerStep);

  if (solver.solution) {
    mpc_setpoint = solver.solution->x.col(0); // or another column as needed
  }
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