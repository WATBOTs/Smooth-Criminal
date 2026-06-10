//Called when IR sensor detects a new edge. Updates stored timestamps
void updateHeading(unsigned long newEdge);

//Returns heading degree based on algorithm
float getHeadingDeg();

//Returns RPM based on algorithm
float getRPM();