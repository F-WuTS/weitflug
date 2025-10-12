#include "common/time.h"

void fspInit(void);
bool fspOpen(void);
void taskFspTx(timeUs_t);
void taskFspRx(timeUs_t);
