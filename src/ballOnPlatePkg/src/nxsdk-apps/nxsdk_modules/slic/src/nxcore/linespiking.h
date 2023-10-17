#include "nxsdk.h"
typedef unsigned char uint8_t;
typedef signed char int8_t;

extern int numCx;
extern int numAxon;
extern int coreId;
extern int timestepPerImage;
extern int numTrainIterations;
extern int imagePresentOffset;
extern int numTrainImages;
extern int bytesPerImage;
extern int firstTimestep;
extern int spikeInterval;

typedef struct ScanLines {
  uint8_t i1;
  uint8_t j1;
  uint8_t i2;
  uint8_t j2;
  int8_t m;
} ScanLine;

void GenScanLines(ScanLine *lines, int *numLines);
void GenDVSSpike(uint8_t *dvsspike, int *img1, ScanLine *lines, int numLines1, int N0, int pp);
int do_spiking(runState *s);
void run_spiking(runState *s);