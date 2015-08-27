#ifndef INVENSENSE_IMU_MLOS_H__
#define INVENSENSE_IMU_MLOS_H__

#include "../timer.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* - Error Codes. - */
#define MLOS_SUCCESS 0
#define MLOS_ERROR   1

#define MLOSSleep(ms) delay_ms(ms)
#define MLOSGetTickCount() (current_time)

#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_IMU_MLOS_H__
