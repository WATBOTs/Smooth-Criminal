// Initialize sensor pin and ISR
void init_IR_Sense ();

// Interrupt function, logs time and sets flag
void logEdgeTime ();

// Returns flag from new rising edge
bool hasNewEdge();

// Returns time from most recent rising edge
unsigned long getEdgeTime();
