/**
 * @file app_state.h
 * @brief Application-wide runtime state shared between main and transports.
 */

#pragma once

#include <stdatomic.h>
#include <stdbool.h>

/** @brief True while the selected transport reports an active mesh. Owned by main.c. */
extern _Atomic bool g_mesh_active;
