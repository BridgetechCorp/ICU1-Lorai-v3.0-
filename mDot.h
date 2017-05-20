/* 
 * File:   mDot.h
 * Author: Craig
 *
 * Created on November 7, 2015, 4:46 PM
 */

#ifndef MDOT_H
#define	MDOT_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "uart.h"


boolean InitMDot( void );    
    
boolean SendLoraPacket( unsigned char command, unsigned int incount, unsigned int outcount, unsigned char functionArea );
    
void mDot_Attention( void );

void mDot_SetFrequencySubBand( int band );

// ATI Request ID Shows firmware and library version information.
void mDot_FirmwareVersion( void );


//ATZ Reset CPU Reset the CPU.
void mDot_Reset( void );


//ATE0/1 Enable/Disable echo mode ATE0: disable, ATE1: enable.
void mDot_TurnEchoOff( void );


//ATV0/1 Enable/Disable verbose mode ATV0: disable, ATV1: enable.
void mDot_EnableVerboseMode( void );

//AT&F Reset factory defaults Reset current configuration to factory defaults.
void mDot_ResetToFactoryDefaults( void );

//AT&W Save configuration Save configuration to flash memory.
void mDot_SaveConfiguration( void );

//AT&V Display status/setting Displays current settings and status.
void mDot_GetStatus( void );

//AT&S Display stats Displays statistics
void mDot_DisplayStatistics( void );

//AT&R Reset stats Resets statistics
void mDot_ResetStatistics( void );

//AT+IPR Set serial baud rate Defaults to 115200.
void mDot_SetSerialDefaults( void );


//AT+SMODE Start up mode 0: AT command mode, 1: Serial data mode.
void mDot_SetATStartupMode( void );

//If this is set to 1(serial data mode), auto join needs to
//be enabled so that a join is performed on power up or
//reset.
//To exit serial data mode (after +SMODE has been set to
//1) reset or power cycle the mDot and input ?+++?
//immediately. There is a one second window to gather
//?+++?. That puts the device in command mode but
//leaves +SMODE=1 such that a subsequent reset starts it
//in serial mode again.
//AT+LOG Debug log level Enable or disable debug logging, 0: off, 1: Fatal - 6: Trace
//Network/Addressing Commands
//Command Description Details
void mDot_SetDebugLogLevelOff( void );

//AT+FREQ Frequency Band Read frequency band 868 or 915. Read only.
void mDot_GetFrequency( void );

//AT+FSB Frequency Sub-band Set the frequency sub-band for US 915. 0:ALL, 1-8.
void mDot_GetFrequencySubBand( void );


//AT+PN Public Network Enable/disable public network mode. 0: off, 1: on.
void mDot_TurnOffPublicNetwork( void );

//AT+DI Device ID Device EUI (unique, set at factory), 8 bytes. Read only.
void mDot_GetDeviceId( void );

//AT+NA Network Address Network address (devAddr in LoraMac), 4 bytes.
void mDot_GetNetworkAddress( void );

//AT+NSK Network Session Key Network session encryption key, 16 bytes.
void mDot_GetNetworkSessionKey( void );

//AT+DSK Data Session Key Data session encryption key (App session keyin LoraMac),
//16 bytes.
void mDot_GetDataSessionKey( void );

//AT+NK Network Key Configured network key/passphrase (App Key in LoraMac)
//## AT+NK=0,hex AT+NK=1,passphrase (Net key =
//cmac(passphrase)). 16 bytes.
void mDot_GetNetworkKey( void );

//AT+NI Network ID Configured Network EUI/Name (App EUI in LoraMac).
//AT+NI=0,hex AT+NI=1,network_name (Net ID =
//crc64(network_name), 8 bytes.
void mDot_GetNetworkID( void );


//AT+JOIN Join Network Join network (acquire network address and session keys).
void mDot_Join( void );

//AT+JR Join Retries Number of times to retry joining the network in an attempt
//(0 - 255).
void mDot_SetJoinRetry( void );

//AT+JBO Join Byte Order Send EUIs in join request with configured byte ordering.
void mDot_JoinByteOrder( void );

//AT+NJM Network Join Mode 0: Manual configuration, 1: OTA Network Join, 2: Auto OTA
//Network Join on start up. Default: 1. Note that 0 for manual
//configuration is part of LoRaWAN compliance.
void mDot_NetworkJoinMode( void );

//AT+NJS Network Join Status Displays last known state. 0: Not joined, 1: Joined.
void mDot_GetJoinStatus( void );

//AT+NLC Network Link Check Perform network link check, OK or ERROR.
void mDot_NetworkLinkCheck( void );

//AT+LCC Link Check Count Set number of packets between each link and check if ACKs
//are disabled.
void mDot_SetLinkCheckCount( void );

//AT+LCT Link Check Threshold Set threshold for number of link check or ACK failures to
//tolerate. 0: off, N: number of failures.
void mDot_SetLinkCheckThreshold( void );

//AT+ENC AES Encryption Enable/disable AES encryption. 0: off, 1: on.
void mDot_EnableAESEncryption( void );

//AT+RSSI Signal Strength Displays last received packet's signal strength.
void mDot_GetRSSI( void );

//AT+SNR Signal To Noise Ratio Display signal to noise ratio received packets last, minimum,
//maximum,average.
void mDot_GetSNR( void );

//AT+DP Data Pending Indicator of data in queue on server.
void mDot_GetDataPending( void );

//AT+PING Send Ping Sends ping, displays RSSI and SNR from gateway on pong.
//Radio Configuration
//Command Description Details
void mDot_Ping( void );

//AT+TXDR Set Tx data rate Set the Tx spread factor for all channels, 7-10.
void mDot_SetTxDataRateSpreadFactor( void );

//AT+TXP Set Tx Power Set the Tx power for all channels.
void mDot_SetTxPower( void );

//AT+TXF Tx Frequency Set Tx frequency.
void mDot_SetFrequency( int freq );

//AT+TXCH Set Tx Channel List channels for current sub-band. 0 = 902.7, 1 = 902.9, etc.
void mDot_SetTxChannel( void );

//AT+TOA Time On Air Get packet transmit time in milliseconds with current data
//rate.
void mDot_GetTimeOnAir( void );

//AT+TXI Set Tx inverted Set Tx signal inverted. Default: off.
void mDot_SetTxInverted( void );

//AT+TXW Tx Wait Enable/disable waiting for rx windows to expire after send.
//0: off, 1: on.
void mDot_SetTxWaitEnable( void );

//AT+RXDR Set Rx Data Rate Set the Rx spread factor for all channels, 7-10.
void mDot_SetRxSpreadFactor( void );

//AT+RXF Rx Frequency Set the Rx frequency for +RECV,+RECVC.
void mDot_SetRxFrequency( void );

//AT+RXI Set Rx inverted Set Rx signal inverted. Default :on
void mDot_SetRxInverted( void );

//AT+FEC Set Error Correction Configure Forward Error Correction. 1 to 4.
void mDot_SetErrorCorrection( void );

//AT+CRC Set CRC Check On/Off Enable/disable CRC checking received packets. 0: off, 1: on.
void mDot_SetCRCCheck( void );

//AT+ADR Adaptive Data Rate Enable/disable Adaptive Data Rate. 0: off, 1: on.
void mDot_SetAdaptiveDataRate( void );

//Send/Receive
//Command Description Details
//AT+ACK Require Send ACK Enable to require send acknowledgment. 0: off, N: number
//of retries until ACK received.
void mDot_SetAckRequired( void );

//AT+SEND Send Once Sends supplied packet data one time and returns response.
void mDot_Send( void );

//AT+SENDB Send Binary Sends supplied binary (hex) packet data one time and
//returns response.
void mDot_SendBinary( void );

//AT+SENDH Send Once High BW Sends supplied packet data one time and returns response
//using High Bandwidth channel. Max: 240 bytes.
void mDot_SendOnceHighBW( void );

//AT+SENDI Send Interval Sends supplied packet data on interval (escape sequence:
//+++).
void mDot_SendInterval( void );

//AT+RECV Receive Once Display last packet received.
void mDot_Receive( void );

//AT+RECVC Receive Continuous Continuously receive and display packets (escape sequence:
//+++).
void mDot_ReceiveContinuously( void );

//AT+TXN Tx Next Get time until next free channel in milliseconds
void mDot_GetTimeUntilNextFreeChannel( void );

//AT+TOA Time on Air Get time in ms of packet tx with current data rate
void mDot_GetTimeOfTransmit( void );

//AT+RXO Rx Output Set the Rx output type. 0:hexadecimal, 1: raw.
//Sleep/Serial Data Mode Options
//Command Description Details
void mDot_SetRxOutputType( void );

//AT+SD Enter serial data mode Reads serial data and sends LoRa packets (escape sequence:
//+++).
void mDot_EnterSerialDataMode( void );

//AT+SLEEP Sleep Mode Enter sleep mode, wake on INTERVAL or INTERRUPT. The
//mDot will wake when the wake interval expires. Must issue
//AT+SLEEP to re-enter sleep mode.
void mDot_EnterSleepMode( void );


//AT+WM Wake Mode Wake up mode; Interrupt uses DIO7 as wake up pin; 0:
//INTERRUPT, 1: INTERVAL.
void mDot_SetWakeMode( void );

//AT+WI Wake Interval Wake up interval (seconds).
void mDot_SetWakeInterval( void );

//AT+WD Wake Delay Maximum time to wait for data on serial port
//(milliseconds).
void mDot_SetWakeDelay( void );

//AT+WTO Wake Timeout Continue reading serial data until timeout (milliseconds).
//Mult
void mDot_SetWakeTimeout( void );



#ifdef	__cplusplus
}
#endif

#endif	/* MDOT_H */

