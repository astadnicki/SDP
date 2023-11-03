#include "FreeSixIMU.h"
#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"



FreeSixIMU start(void){
    FreeSixIMU sixDOF = FreeSixIMU();
    sixDOF.init();
    return sixDOF;
}


extern "C" float * readIMU(int i){
    sixDOF = start();
    float data[4];
    switch (i){
        case 0:
            sixDOF.getEuler(data);
            break;
        case 1: 
            sixDOF.getValues(data);
            break;
    }

    return data;

}