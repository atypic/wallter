#include <stdio.h>
#include "demo.hpp"              // LCD demo
#include "cytron_md_demo.h"      // Cytron demo

void app_main(void)
{
	espidf_lcd_demo();
	cytron_md_demo();
    while(1);
}