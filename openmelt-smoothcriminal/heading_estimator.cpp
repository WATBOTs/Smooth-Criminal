#include "arduino.h"
#include "melty_config.h"
#include "heading_estimator.h"

uint32_t sensorEdgeTime[2] = {0};
uint8_t  numEdges = 0;
int16_t heading = 0;
float rpm = 0;

//Used to access timestamps for debugging if defined
#ifdef SERIAL_DEBUG_OUTPUT
  uint32_t getEstimatorEdgeTime(int index) {
    return sensorEdgeTime[index];
  }
#endif

//Called when IR sensor detects a new edge. Updates stored timestamps 
void updateHeading(uint32_t newEdge) {
  //Check if the robot is spining fast enough to begin recording data
  if(numEdges > 0 && (micros() - sensorEdgeTime[0]) > REV_TIMEOUT){
    numEdges = 0; //Reset if too slow
  } else if(numEdges == 2 && (micros() - sensorEdgeTime[0]) / (sensorEdgeTime[0] - sensorEdgeTime[1]) > MISSED_EDGE_RATIO){
    numEdges = 0; //Reset estimator if missed edge based on ratio of current period vs. previous period
  }else if(numEdges < 2){
    numEdges++; //Increment
  }
   
  //Shift array and insert new edge
  sensorEdgeTime[1] = sensorEdgeTime[0];
  sensorEdgeTime[0] = newEdge;
}

int16_t getHeadingDeg() {
  //Check for sufficient data
  if(numEdges == 2 && (micros() - sensorEdgeTime[0]) < REV_TIMEOUT){
    //Calculate heading based on linear extrapolation of previous edge times
    //Note that this works on the assumption that each edge is 360 degrees apart
    heading =  (((uint32_t)(micros() - sensorEdgeTime[1]) * 360UL) / (uint32_t)(sensorEdgeTime[0] - sensorEdgeTime[1])) % 360;
  }
  return heading;
}

float getRPM() {
  //Check for sufficient data
  if(numEdges == 2 && (micros() - sensorEdgeTime[0]) < REV_TIMEOUT){
    //Calculate rpm based on difference in previous edge times
    //This also relies on a 360 offset between edges, as it is basing rpm off the time for one full rotation between edges.
    rpm = (60000000.0f) / (sensorEdgeTime[0] - sensorEdgeTime[1]);
  }
  return rpm;
}