/* 
 * File:   CanIrma.h
 * Author: Craig Beiferman
 *
 * Created on May 4, 2013, 6:17 AM
 */

#ifndef CANIRMA_H
#define	CANIRMA_H

extern void SendIRMACounterStateRequest( void );
extern void SendIRMACounterStateRequest2(  unsigned long addr, int functionArea );

extern void SendIRMACountDataReset( unsigned long addr  );
extern void SendIRMAGetCanAddress( void );
extern void SendIRMASetDoorsOpen(unsigned long addr, int functionArea ); // turn on the sensor
extern void SendIRMASetDoorsClose(unsigned long addr, int functionArea ); // turn off the sensor
extern void SendIRMASensorStateRequest( unsigned long addr  );

extern void SendFunctionAreaStatusQuery( unsigned long addr );

extern void SendIRMASensorFirmwareRestart( void );

extern void GetFunctionID( unsigned long addr );

extern unsigned long sensorCanAddress;
#endif	/* CANIRMA_H */

