#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif

#include <string.h>
#include <stdio.h>
#include "uart.h"
#include "mDot.h"
#include "timer.h"
#include "user.h"

int txCount = 0;
int rxCount = 0;

#define RETRY_CNT 2

// these options must match the settings on your Conduit
// uncomment the following lines and edit their values to match your configuration
char * config_network_name = "bridgetechnology"; // <lora network id>
char * config_network_pass = "bridgetechnology"; // <lora network key>
int config_frequency_sub_band = 7;
int config_frequency = 915;


boolean CheckUartRxBufferForOK( void )
{
  int checkstate = 0;
  char c;
  while ( getch(&c) ) 
  {
      switch (checkstate)
      {
          case 0:
            checkstate =  (c == 'O')?1:0; 
            break;
          case 1:
            checkstate =  (c == 'K')?2:0;            
            break;
          case 2:
            if (c=='\n' || c=='\r') 
               return TRUE;
            else
               checkstate =  0;
            break;              
      }          
  }
  return FALSE;
}

boolean SendCommandWithResponseDelay( char *str, int delay )
{
    int repeat = RETRY_CNT;
    // ret
    while (repeat-- > 0)
    {
        LORA_LED = 0;
        ClearUartReceiveBuffer(); // clear any old data out from uart buffer
        putstr(str); // send the command to the mDot
        // now wait for response
        SetWaitDelay(delay); while (GetWaitDelay()!=0);
        LORA_LED = 1;
        SetWaitDelay(100); while (GetWaitDelay()!=0);
        if (CheckUartRxBufferForOK()==TRUE)
            return TRUE;
    }
    return FALSE;
}

// returns true if the OK was received
boolean SendCommand( char *str )
{
    return SendCommandWithResponseDelay( str, 25 );   
}


// returns TRUE if packet got OK, FALSE otherwise
boolean SendLoraPacket( unsigned char command, unsigned int incount, unsigned int outcount, unsigned char functionArea )
{
    int64 val;
    int i;//,start;
    char number[20];
    char buffer[100];
    char num;
    //int command = 1;
    txCount++;
    //         63..56                       55..48                      47..32                  31..16                    15..8               7..0                                                
    val = ((int64)command<<56) | ((int64)functionArea<<48) | (( int64)incount<<32) | ((int64)outcount<<16) | ((int64)txCount<<8) | (int64)rxCount;
      
   // val = 0x0123456789ABCDEF;
    // todo send packet here
    // now wait for response
    // if we get a response rxCount++
    // packet format
    // command:8 | function area:8 | incount:16 | outcount:16 | txCount | rxCount ;
    // transmit packet here
    //sprintf(buffer,"AT+SENDB EE01001E00213E3D\n" ); 
    sprintf(buffer,"AT+SENDB 0000000000000000\n" ); 
     // sprintf(buffer,"AT+SENDB 00000000000000\n" ); 
   // 01-00-1E-00-21-3E-3D
    //sprintf(buffer,"AT+SENDB %s\n",number );  
      // sprintf(buffer,"AT+SENDB EE01001E00213E3D\n" ); 
    for (i=0;i<16;i++)
    {
        num = (unsigned char )(val >> i*4) & 0x0f; 
        sprintf(number,"%x\n",num);
        buffer[24-i] = number[0]; 
    }
    
    
   // for (i=0;i<14;i++)
    //for (i=0;i<16;i++)
   // {
   //    num = (unsigned char )(val >> i*4) & 0x0f; 
   //    sprintf(number,"%x\n",num);
      // buffer[24-i] = number[0];
   //     buffer[22-i] = number[0];
  //  }
    
    
  //  lennum = strlen(number);
  //  start = 9 + (16-lennum);
  //  for (i=0;i<lennum;i++)
  //      buffer[start+i] = number[i];
    
    if (SendCommandWithResponseDelay(buffer,2000)== TRUE)
    {
        rxCount++;
        return TRUE;
    }
    return FALSE;
}


boolean InitMDot( void ) 
{   
    char buffer[100];
    
    // for maximum range, set mDot transmit power to 20
    // set spread factor to 10
    // set conduit transmit power to +26dBm
        
    
    SendCommand( "ATE0\n"); // turn echo off
    
    // set the transmit power to 20 for max range
    SendCommand("AT+TXP=20\n");
    
    //AT+TXDR Set Tx data rate Set the Tx spread factor for all channels, 7-10.
    SendCommand("AT+TXDR=10\n");
     
    sprintf(buffer,"AT+NI=1,%s\n",config_network_name);   
    SendCommand(buffer); // set network id
    
    sprintf(buffer,"AT+NK=1,%s\n",config_network_pass);
    SendCommand(buffer);
    
    // tell mDot to expect ACK, and retry up to 8 times
    SendCommand("AT+ACK=8\n"); // 10 is the retry count 
    
    // tell mDot that we want a private network
    SendCommand("AT+PN=0\n"); // disable the public network
   
     // Configure Frequency Sub-band (1:902.3-903.7, 2:903.9-905.3,..., 8:913.5-914.9)
    sprintf(buffer,"AT+FSB=%d\n",config_frequency_sub_band);
    SendCommand(buffer);
    
    
    // AT&W writes your configuration to non-volatile memory on the mDot.
   // SendCommand("AT&W\n");  // we might do this later
   
   // Join your Conduit's LoRa network so you can send and receive packets.
    if (SendCommandWithResponseDelay("AT+JOIN\n",3000) == TRUE)
    {
        return TRUE; // YEAH we joined the network.
    }
    
    return FALSE; // RATS failed to join network

    /*
AT+TXDR Set Tx data rate Set the Tx spread factor for all channels, 7-10.
AT+TXP Set Tx Power Set the Tx power for all channels.
AT+TXF Tx Frequency Set Tx frequency.
AT+TXCH Set Tx Channel List channels for current sub-band. 0 = 902.7, 1 = 902.9, etc.
AT+TOA Time On Air Get packet transmit time in milliseconds with current data rate.
AT+TXI Set Tx inverted Set Tx signal inverted. Default: off.
AT+TXW Tx Wait Enable/disable waiting for rx windows to expire after send. 0: off, 1: on.
AT+RXDR Set Rx Data Rate Set the Rx spread factor for all channels, 7-10.
AT+RXF Rx Frequency Set the Rx frequency for +RECV,+RECVC.
AT+RXI Set Rx inverted Set Rx signal inverted. Default :on
AT+FEC Set Error Correction Configure Forward Error Correction. 1 to 4.
AT+CRC Set CRC Check On/Off Enable/disable CRC checking received packets. 0: off, 1: on.
AT+ADR Adaptive Data Rate Enable/disable Adaptive Data Rate. 0: off, 1: on.

    
    
    // a higher spreading factor allows for longer range but lower throughput
    // in the 915 (US) frequency band, spreading factors 7 - 10 are available
    // in the 868 (EU) frequency band, spreading factors 7 - 12 are available
    setTxDataRate(mDot::SF_10);
    
    // request receive confirmation of packets from the gateway
    setAck(1); // enable ACKS
    
    // save this configuration to the mDot's NVM
    saveConfig();
   
 
    // attempt to join the network
    logInfo("joining network");
    while ((ret = dot->joinNetwork()) != mDot::MDOT_OK) {
        logError("failed to join network %d:%s", ret, mDot::getReturnCodeString(ret).c_str());
        // in the 868 (EU) frequency band, we need to wait until another channel is available before transmitting again
        osDelay(std::max((uint32_t)1000, (uint32_t)dot->getNextTxMs()));
    }
 
    // format data for sending to the gateway
    
    while (true) {
        // send the data to the gateway
        if ((ret = dot->send(data)) != mDot::MDOT_OK) {
            logError("failed to send", ret, mDot::getReturnCodeString(ret).c_str());
        } else {
            logInfo("successfully sent data to gateway");
        }
 
        // in the 868 (EU) frequency band, we need to wait until another channel is available before transmitting again
        osDelay(std::max((uint32_t)5000, (uint32_t)dot->getNextTxMs()));
    }
 
    return 0;
      */
}








void mDot_Attention( void )
{
    SendCommand( "AT\n");   
}

// ATI Request ID Shows firmware and library version information.
void mDot_FirmwareVersion( void )
{
    SendCommand( "ATI\n");   
}

//ATZ Reset CPU Reset the CPU.
void mDot_Reset( void )
{
    SendCommand( "ATZ\n");   
}

//ATE0/1 Enable/Disable echo mode ATE0: disable, ATE1: enable.
void mDot_TurnEchoOff( void )
{
    SendCommand( "ATE0\n");   
}

//ATV0/1 Enable/Disable verbose mode ATV0: disable, ATV1: enable.
void mDot_EnableVerboseMode( void )
{
    SendCommand( "ATV1\n");   
}
//AT&F Reset factory defaults Reset current configuration to factory defaults.
void mDot_ResetToFactoryDefaults( void )
{
    SendCommand( "AT&F\n");   
}
//AT&W Save configuration Save configuration to flash memory.
void mDot_SaveConfiguration( void )
{
    SendCommand( "AT&W\n");   
}
//AT&V Display status/setting Displays current settings and status.
void mDot_GetStatus( void )
{
    SendCommand( "AT&V\n");   
}
//AT&S Display stats Displays statistics
void mDot_DisplayStatistics( void )
{
    SendCommand( "AT&S\n");   
}
//AT&R Reset stats Resets statistics
void mDot_ResetStatistics( void )
{
    SendCommand( "AT&R\n");   
}
//AT+IPR Set serial baud rate Defaults to 115200.
void mDot_SetSerialDefaults( void )
{
    SendCommand( "AT+IPR\n");   
}

//AT+SMODE Start up mode 0: AT command mode, 1: Serial data mode.
void mDot_SetATStartupMode( void )
{
    SendCommand( "AT+SMODE0\n");   
}
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
void mDot_SetDebugLogLevelOff( void )
{
    SendCommand( "AT+LOG0\n");   
}
//AT+FREQ Frequency Band Read frequency band 868 or 915. Read only.
void mDot_GetFrequency( void )
{
    SendCommand( "AT+FREQ\n");   
}
//AT+FSB Frequency Sub-band Set the frequency sub-band for US 915. 0:ALL, 1-8.
void mDot_GetFrequencySubBand( void )
{
    SendCommand( "AT+FSB\n");   
}

//AT+PN Public Network Enable/disable public network mode. 0: off, 1: on.
void mDot_TurnOffPublicNetwork( void )
{
    SendCommand( "AT+PN0\n");   
}
//AT+DI Device ID Device EUI (unique, set at factory), 8 bytes. Read only.
void mDot_GetDeviceId( void )
{
    SendCommand( "AT+DI\n");   
}
//AT+NA Network Address Network address (devAddr in LoraMac), 4 bytes.
void mDot_GetNetworkAddress( void )
{
    SendCommand( "AT+NA\n");   
}
//AT+NSK Network Session Key Network session encryption key, 16 bytes.
void mDot_GetNetworkSessionKey( void )
{
    SendCommand( "AT+NSK\n");   
}
//AT+DSK Data Session Key Data session encryption key (App session keyin LoraMac),
//16 bytes.
void mDot_GetDataSessionKey( void )
{
    SendCommand( "AT+DSK\n");   
}
//AT+NK Network Key Configured network key/passphrase (App Key in LoraMac)
//## AT+NK=0,hex AT+NK=1,passphrase (Net key =
//cmac(passphrase)). 16 bytes.
void mDot_GetNetworkKey( void )
{
    SendCommand( "AT+NK\n");   
}
//AT+NI Network ID Configured Network EUI/Name (App EUI in LoraMac).
//AT+NI=0,hex AT+NI=1,network_name (Net ID =
//crc64(network_name), 8 bytes.
void mDot_GetNetworkID( void )
{
    SendCommand( "AT+NI\n");   
}

//AT+JOIN Join Network Join network (acquire network address and session keys).
void mDot_Join( void )
{
    SendCommand( "AT+JOIN\n");   
}
//AT+JR Join Retries Number of times to retry joining the network in an attempt
//(0 - 255).
void mDot_SetJoinRetry( void )
{
    SendCommand( "AT+JR\n");   
}
//AT+JBO Join Byte Order Send EUIs in join request with configured byte ordering.
void mDot_JoinByteOrder( void )
{
    SendCommand( "AT+JBO\n");   
}
//AT+NJM Network Join Mode 0: Manual configuration, 1: OTA Network Join, 2: Auto OTA
//Network Join on start up. Default: 1. Note that 0 for manual
//configuration is part of LoRaWAN compliance.
void mDot_NetworkJoinMode( void )
{
    SendCommand( "AT+NJM\n");   
}
//AT+NJS Network Join Status Displays last known state. 0: Not joined, 1: Joined.
void mDot_GetJoinStatus( void )
{
    SendCommand( "AT+NJS\n");   
}
//AT+NLC Network Link Check Perform network link check, OK or ERROR.
void mDot_NetworkLinkCheck( void )
{
    SendCommand( "AT+NLC\n");   
}
//AT+LCC Link Check Count Set number of packets between each link and check if ACKs
//are disabled.
void mDot_SetLinkCheckCount( void )
{
    SendCommand( "AT+LCC\n");   
}
//AT+LCT Link Check Threshold Set threshold for number of link check or ACK failures to
//tolerate. 0: off, N: number of failures.
void mDot_SetLinkCheckThreshold( void )
{
    SendCommand( "AT+LCT\n");   
}
//AT+ENC AES Encryption Enable/disable AES encryption. 0: off, 1: on.
void mDot_EnableAESEncryption( void )
{
    SendCommand( "AT+ENC\n");   
}
//AT+RSSI Signal Strength Displays last received packet's signal strength.
void mDot_GetRSSI( void )
{
    SendCommand( "AT+RSSI\n");   
}
//AT+SNR Signal To Noise Ratio Display signal to noise ratio received packets last, minimum,
//maximum,average.
void mDot_GetSNR( void )
{
    SendCommand( "AT+SNR\n");   
}
//AT+DP Data Pending Indicator of data in queue on server.
void mDot_GetDataPending( void )
{
    SendCommand( "AT+DP\n");   
}
//AT+PING Send Ping Sends ping, displays RSSI and SNR from gateway on pong.
//Radio Configuration
//Command Description Details
void mDot_Ping( void )
{
    SendCommand( "AT+PING\n");   
}
//AT+TXDR Set Tx data rate Set the Tx spread factor for all channels, 7-10.
void mDot_SetTxDataRateSpreadFactor( void )
{
    SendCommand( "AT+TXDR\n");   
}
//AT+TXP Set Tx Power Set the Tx power for all channels.
void mDot_SetTxPower( void )
{
    SendCommand( "AT+TXP\n");   
}
//AT+TXF Tx Frequency Set Tx frequency.
void mDot_SetFrequency( int freq  )
{
    if (freq == 915)
        SendCommand( "AT+TXF 915\n");
    else
        SendCommand( "AT+TXF 868\n");
        
}
//AT+TXCH Set Tx Channel List channels for current sub-band. 0 = 902.7, 1 = 902.9, etc.
void mDot_SetTxChannel( void )
{
    SendCommand( "AT+TXCH\n");   
}
//AT+TOA Time On Air Get packet transmit time in milliseconds with current data
//rate.
void mDot_GetTimeOnAir( void )
{
    SendCommand( "AT+TOA\n");   
}
//AT+TXI Set Tx inverted Set Tx signal inverted. Default: off.
void mDot_SetTxInverted( void )
{
    SendCommand( "AT+TXI\n");   
}
//AT+TXW Tx Wait Enable/disable waiting for rx windows to expire after send.
//0: off, 1: on.
void mDot_SetTxWaitEnable( void )
{
    SendCommand( "AT+TXW1\n");   
}

void mDot_SetFrequencySubBand( int band )
{
     // frequency sub band is only applicable in the 915 (US) frequency band
    // if using a MultiTech Conduit gateway, use the same sub band as your Conduit (1-8) - the mDot will use the 8 channels in that sub band
    // if using a gateway that supports all 64 channels, use sub band 0 - the mDot will use all 64 channels
     SendCommand( "AT+FSB 1\n");   
}
//AT+RXDR Set Rx Data Rate Set the Rx spread factor for all channels, 7-10.
void mDot_SetRxSpreadFactor( void )
{
    SendCommand( "AT+RXDR\n");   
}
//AT+RXF Rx Frequency Set the Rx frequency for +RECV,+RECVC.
void mDot_SetRxFrequency( void )
{
    SendCommand( "AT+RXF\n");   
}
//AT+RXI Set Rx inverted Set Rx signal inverted. Default :on
void mDot_SetRxInverted( void )
{
    SendCommand( "AT+RXI\n");   
}
//AT+FEC Set Error Correction Configure Forward Error Correction. 1 to 4.
void mDot_SetErrorCorrection( void )
{
    SendCommand( "AT+FEC\n");   
}
//AT+CRC Set CRC Check On/Off Enable/disable CRC checking received packets. 0: off, 1: on.
void mDot_SetCRCCheck( void )
{
    SendCommand( "AT+CRC\n");   
}
//AT+ADR Adaptive Data Rate Enable/disable Adaptive Data Rate. 0: off, 1: on.
void mDot_SetAdaptiveDataRate( void )
{
    SendCommand( "AT+ADR1\n");   
}
//Send/Receive
//Command Description Details
//AT+ACK Require Send ACK Enable to require send acknowledgment. 0: off, N: number
//of retries until ACK received.
void mDot_SetAckRequired( void )
{
    SendCommand( "AT+ACK\n");   
}
//AT+SEND Send Once Sends supplied packet data one time and returns response.
void mDot_Send( void )
{
    SendCommand( "AT+SEND\n");   
}
//AT+SENDB Send Binary Sends supplied binary (hex) packet data one time and
//returns response.
void mDot_SendBinary( void )
{
    SendCommand( "AT+SENDB\n");   
}
//AT+SENDH Send Once High BW Sends supplied packet data one time and returns response
//using High Bandwidth channel. Max: 240 bytes.
void mDot_SendOnceHighBW( void )
{
    SendCommand( "AT+SENDH\n");   
}
//AT+SENDI Send Interval Sends supplied packet data on interval (escape sequence:
//+++).
void mDot_SendInterval( void )
{
    SendCommand( "AT+SENDI\n");   
}
//AT+RECV Receive Once Display last packet received.
void mDot_Receive( void )
{
    SendCommand( "AT+RECV\n");   
}
//AT+RECVC Receive Continuous Continuously receive and display packets (escape sequence:
//+++).
void mDot_ReceiveContinuously( void )
{
    SendCommand( "AT+RECVC\n");   
}
//AT+TXN Tx Next Get time until next free channel in milliseconds
void mDot_GetTimeUntilNextFreeChannel( void )
{
    SendCommand( "AT+TXN\n");   
}
//AT+TOA Time on Air Get time in ms of packet tx with current data rate
void mDot_GetTimeOfTransmit( void )
{
    SendCommand( "AT+TOA\n");   
}
//AT+RXO Rx Output Set the Rx output type. 0:hexadecimal, 1: raw.
//Sleep/Serial Data Mode Options
//Command Description Details
void mDot_SetRxOutputType( void )
{
    SendCommand( "AT+RXO\n");   
}
//AT+SD Enter serial data mode Reads serial data and sends LoRa packets (escape sequence:
//+++).
void mDot_EnterSerialDataMode( void )
{
    SendCommand( "AT+SD\n");   
}
//AT+SLEEP Sleep Mode Enter sleep mode, wake on INTERVAL or INTERRUPT. The
//mDot will wake when the wake interval expires. Must issue
//AT+SLEEP to re-enter sleep mode.
void mDot_EnterSleepMode( void )
{
    SendCommand( "AT+SLEEP\n");   
}

//AT+WM Wake Mode Wake up mode; Interrupt uses DIO7 as wake up pin; 0:
//INTERRUPT, 1: INTERVAL.
void mDot_SetWakeMode( void )
{
    SendCommand( "AT+WM\n");   
}
//AT+WI Wake Interval Wake up interval (seconds).
void mDot_SetWakeInterval( void )
{
    SendCommand( "AT+WI\n");   
}
//AT+WD Wake Delay Maximum time to wait for data on serial port
//(milliseconds).
void mDot_SetWakeDelay( void )
{
    SendCommand( "AT+WD\n");   
}
//AT+WTO Wake Timeout Continue reading serial data until timeout (milliseconds).
//Mult
void mDot_SetWakeTimeout( void )
{
    SendCommand( "AT+WTO\n");   
}