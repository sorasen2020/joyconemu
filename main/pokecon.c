#include "pokecon.h"

void pokecon_resetdirections(pokecon_report_input_t *pc_report)
{
	pc_report->lx  = STICK_CENTER;
	pc_report->ly  = STICK_CENTER;
	pc_report->rx  = STICK_CENTER;
	pc_report->ry  = STICK_CENTER;
	pc_report->hat = HAT_CENTER;
}

int pokecon_parseline(char* line, pokecon_report_input_t *pc_report)
{
	char cmd[16];
	uint16_t btns;
	uint8_t hat;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;

  	// get command
	int ret = sscanf(line, "%s", cmd);
	if (ret == EOF) {

	} else if (strncmp(cmd, "end", 16) == 0) {
		pokecon_resetdirections(pc_report);
	} else if (cmd[0] >= '0' && cmd[0] <= '9') {
		memset(pc_report, 0, sizeof(uint16_t));

		// format [button LeftStickX LeftStickY RightStickX RightStickY HAT]
		// button: Y | B | A | X | L | R | ZL | ZR | MINUS | PLUS | LCLICK | RCLICK | HOME | CAP
		// LeftStick : 0 to 255
		// RightStick: 0 to 255
		sscanf(line, "%hx %hhx %hhx %hhx %hhx %hhx", &btns, &hat, &lx, &ly, &rx, &ry);

   		 // HAT : 0(TOP) to 7(TOP_LEFT) in clockwise | 8(CENTER)
		pc_report->hat = hat;

    	// we use bit array for buttons(2 Bytes), which last 2 bits are flags of directions
		bool use_right = btns & 0x1;
		bool use_left  = btns & 0x2;

		// Left stick
		if (use_left) {
			pc_report->lx = lx;
			pc_report->ly = ly;
		}

    	// Right stick
		if (use_right & use_left) {
			pc_report->rx = rx;
			pc_report->ry = ry;
		} else if (use_right) {
			pc_report->rx = lx;
			pc_report->ry = ly;
		}

		pc_report->btns |= btns;
	}

    return (ret);
}
