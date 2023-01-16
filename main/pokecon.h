#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define JOYCON_L         (0x01)
#define JOYCON_R         (0x02)
#define PRO_CON          (0x03)

/* CONTROLLER_TYPE : PRO_CON / JOYCON_L / JOYCON_R */
#define CONTROLLER_TYPE  (PRO_CON)

// #define _DEBUG           (1)

#define HAT_TOP          (0x00)
#define HAT_TOP_RIGHT    (0x01)
#define HAT_RIGHT        (0x02)
#define HAT_BOTTOM_RIGHT (0x03)
#define HAT_BOTTOM       (0x04)
#define HAT_BOTTOM_LEFT  (0x05)
#define HAT_LEFT         (0x06)
#define HAT_TOP_LEFT     (0x07)
#define HAT_CENTER       (0x08)

#define STICK_MIN        (0)
#define STICK_CENTER     (128)
#define STICK_MAX        (255)

typedef enum {
    SWITCH_Y       = 0x0001,
    SWITCH_B       = 0x0002,
    SWITCH_A       = 0x0004,
    SWITCH_X       = 0x0008,
    SWITCH_L       = 0x0010,
    SWITCH_R       = 0x0020,
    SWITCH_ZL      = 0x0040,
    SWITCH_ZR      = 0x0080,
    SWITCH_MINUS   = 0x0100,
    SWITCH_PLUS    = 0x0200,
    SWITCH_LCLICK  = 0x0400,
    SWITCH_RCLICK  = 0x0800,
    SWITCH_HOME    = 0x1000,
    SWITCH_CAPTURE = 0x2000,
} pokecon_buttons_t;

typedef struct
{
    uint16_t btns;
    uint8_t hat;
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t dummy;
} pokecon_report_input_t;

void pokecon_resetdirections(pokecon_report_input_t *pc_report);
int pokecon_parseline(char* line, pokecon_report_input_t *pc_report);
