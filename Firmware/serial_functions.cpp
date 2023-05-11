#include <Arduino.h>
#include "serial_functions.h"

SerialCommand decodeSerialCommand(String *inputString)
{
    SerialCommand inCommand;
    //Serial.println(*inputString);

    //Read the incoming command
    inCommand.command=(unsigned int)((*inputString).charAt(0));


    //Divide the paramaters in separates strings
    unsigned char index = 2;
    unsigned char ind_last_sep = 2;
    inCommand.nParams = 0;

    do{
        index++;
        if((*inputString).charAt(index)==SEPARATOR || (*inputString).charAt(index)==EOL){
            inCommand.params[inCommand.nParams] = (*inputString).substring(ind_last_sep, index);
            
            ind_last_sep = index;
            inCommand.nParams ++;
        }
        
    }while ((*inputString).charAt(index)!=EOL);
    

    return inCommand;
}

