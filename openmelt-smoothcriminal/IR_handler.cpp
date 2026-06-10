#include "arduino.h"
#include "melty_config.h"
#include "IR_handler.h"

volatile unsigned long edgeTime = 0;
volatile bool newEdge = false;

void init_IR_Sense (){
    pinMode(IR_SENSOR_PIN, INPUT); //Set pinMode for sensor pin
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), logEdgeTime, RISING); //Attach interrupt function to rising edge
}

void logEdgeTime (){
    edgeTime = micros(); //Update time
    newEdge = true; //Set flag
}

bool hasNewEdge(){
    return newEdge;
}

unsigned long getEdgeTime(){
    newEdge = false; //Reset flag
    return edgeTime;
}


