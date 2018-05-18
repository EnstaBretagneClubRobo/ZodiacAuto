int jrkConnect(const char * device);
int jrkGetVariable(int fd, unsigned char command);
int jrkGetFeedback(int fd);
int jrkGetScalingFeedback(int fd);
int jrkGetTarget(int fd);
int jrkGetErrorFlagsHalting(int fd);
int jrkSetTarget(int fd, unsigned short target);
void jrkTest(int fd);

