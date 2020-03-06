/*
 * display.c
 *
 * Created: 29.10.2019 07:14:08
 *  Author: Kenan
 */ 
#include "display.h"
#include "u8g.h"

void renderDisplay(u8g_t *u8g, char a[20])
{
	u8g_DrawStr90(u8g, 110, 0, "Test");
	u8g_DrawStr90(u8g, 90, 0, a);
}
