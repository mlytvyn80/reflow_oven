#include "overcon_command.h"

void init_oven_command(void)
{
    oven_command.buffer[0] = '\0';
    oven_command.length = 0;
    oven_command.ready_to_parse = false;
}

