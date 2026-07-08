//Used to access timestamps for debugging if defined
#ifdef SERIAL_DEBUG_OUTPUT
    uint32_t getEstimatorEdgeTime(int index);
#endif

//Called when IR sensor detects a new edge. Updates stored timestamps
void updateHeading(unsigned long newEdge);

//Returns heading degree based on algorithm
float getHeadingDeg();

//Returns RPM based on algorithm
float getRPM();