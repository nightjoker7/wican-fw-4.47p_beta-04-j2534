/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Internal WireGuard backend for VPN manager */
#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include "include/vpn_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

// Init/Deinit backend (creates wg context)
esp_err_t vpn_wg_init(const vpn_wireguard_config_t *cfg);
esp_err_t vpn_wg_deinit(void);

// Start/Stop connection
esp_err_t vpn_wg_start(void);
esp_err_t vpn_wg_stop(void);

// Helpers
bool vpn_wg_is_peer_up(void);
esp_err_t vpn_wg_set_default_route(void);

#ifdef __cplusplus
}
#endif
