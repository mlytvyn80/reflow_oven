#ifndef OVEN_COMMAND_H_
#define OVEN_COMMAND_H_

#define MAX_COMMAND_BUF_SIZE 255

static struct oven_command
{
    char buffer[MAX_COMMAND_BUF_SIZE];
    uint8_t length = 0;
    bool ready_to_parse = false;
};

void init_oven_command(void);





#endif // OVEN_COMMAND_H_