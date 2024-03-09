/*
 * globalInts.h
 *
 *  Created on: Feb 10, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_GLOBALINTS_H_
#define SRC_GLOBALINTS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    machState_justPoweredOn=0,
    machState_startEngine,
    machState_runEngineHydIdle,
    machState_runEngineHydFwd,
    machState_runEngineHydRev,
    // TODO add any states here
    machState_killEngine // Keep this one last, to sanity check enum value on sets

}machineState_t;

machineState_t globalInts_getMachineState(void);
void globalInts_setMachineState(machineState_t st);

#ifdef __cplusplus
}
#endif

#endif /* SRC_GLOBALINTS_H_ */
