

#ifndef SERIAL_FUNCT
#define SERIAL_FUNCT

#define SEPARATOR ' '
#define EOL '\0'

struct SerialCommand
{
    /* Struct for saving the incoming command data*/

    
    unsigned int command;
    short  nParams = 0;
    String params[4];
};

SerialCommand decodeSerialCommand(String *inputString);

String getValue(String data, char separator, int index);

#endif
