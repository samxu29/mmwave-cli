/**
 * @file config.h
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief TOML configuration file parser
 * @version 0.1
 * @date 2022-08-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MMWAVE_CONFIG_H
#define MMWAVE_CONFIG_H

/**
 * PATCHED NOTE: devConfig_t.profileCfg is now an array of 3 (mimo.h), to
 * support the 3-profile TDM setup (idle time differs per chirp: 175/7/7us).
 * This TOML parser (read_config/read_mimo_config) only ever wrote a SINGLE
 * [mimo.profile] block - it now writes into profileCfg[0] specifically.
 * Profiles [1] and [2] (chirp1/chirp2, both 7us idle) are NOT settable via
 * TOML - they keep their hardcoded defaults from mimo.c (profileCfgArgs1/2),
 * which already match the target Lua config exactly. If you need those
 * TOML-configurable too, read_mimo_config needs extending to parse
 * [[mimo.profile]] as an array-of-tables instead of a single table.
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "toml.h"
#include "../mimo.h"

/* Read the configuration from the TOML file  */
int read_config(unsigned char *filename, devConfig_t *config);

/* Read all the config related to MIMO configuration */
void read_mimo_config(toml_table_t* configfile, devConfig_t *config);

#endif
