/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
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
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */
#include "ecan.h"
#include "oscillator.h"
#include "CanIrma.h"
#include "timer.h"
#include "uart.h"
#include "mDot.h"


unsigned long GetCANAddressFromBuffer( void );
// DSPIC33EP256MU806 Configuration Bit Settings


// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF                // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF               // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = PRI              // Initial Oscillator Source Selection bits (Primary Oscillator (XT, HS, EC))
//#pragma config FNOSC = FRC              // Initial Oscillator Source Selection bits (Primary Oscillator (XT, HS, EC))
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Mode Select bits (HS Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Wait Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // Power-on Reset Timer Value Select bits (128ms)
#pragma config BOREN = ON               // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config RSTPRI = PF              // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF               // Auxiliary Segment Write-protect bit (Aux Flash may be written)
#pragma config APL = OFF                // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF               // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)


typedef enum
{
    INIT_STATE = 0,      
    RUNNING,
    LORA_TRANSMIT,
    LORA_WAIT
       
} my_state_t;

my_state_t state = INIT_STATE;

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
// declare a holder for the latest incoming CAN message
mID message;
mID messages[10];
mID txMessage;

#define MAX_SENSORS  8

unsigned long SensorAddr[MAX_SENSORS] = {0,0,0,0,0,0,0,0};
int FuncArea[MAX_SENSORS] = {0,0,0,0,0,0,0,0};
uint16 PeopleCountIn[MAX_SENSORS]  = {0,0,0,0,0,0,0,0};
uint16 PeopleCountOut[MAX_SENSORS] = {0,0,0,0,0,0,0,0};
uint16 LastPeopleCountIn[MAX_SENSORS]  = {0,0,0,0,0,0,0,0};
uint16 LastPeopleCountOut[MAX_SENSORS] = {0,0,0,0,0,0,0,0};
int SensorStatus[MAX_SENSORS] = {0,0,0,0,0,0,0,0};
int Sabotaged[MAX_SENSORS]  = {0,0,0,0,0,0,0,0};


extern boolean flashblink;
extern boolean slowblink;

boolean SensorEnabled = true;

unsigned long sensorCanAddress;

int system_disi_level;

void ClearAllMessages( void );

int (*RunLevel5Main)(void);


int GetCanAddrIndex( unsigned long addr )
{
   int i;   
    for (i=0;i<MAX_SENSORS;i++)
    {
      if (SensorAddr[i] == addr)
          return i;  
    }
    return 0; // give something valid if we have nothing 
}

// get the index of the can address message
int GetCanIndex( void )
{
    int i;
    unsigned long ca = GetCANAddressFromBuffer();
    for (i=0;i<MAX_SENSORS;i++)
    {
      if (SensorAddr[i] == ca)
          return i;  
    }
    return 0; // give something valid if we have nothing
}


int process_can_message(mID messageType)
{ // Accepts a CAN message from the PC and organizes it for the slave to read/write


//  uint8 isread, from_address,  registerNum;
//  unsigned char canData[6]; // Unsure if this initialization is actually needed, considering the CAN message is sent based on the register's byte size
//  int16 i;


   // copy the message based on first byte in data.
  if (messageType.data[0] < 0x6)
  {

    memcpy(&messages[messageType.data[0]],&messageType,sizeof(mID));
  }

  return messageType.data[0]; // return the last message id
  //for (i = 0; i < messageType.data_length - 2 ; i++)
  //  canData[i] = messageType.data[i + 2];
  //registerNum = messageType.data[1];
  //isread = messageType.data[0] >> 7;
  //from_address = messageType.data[0] & 0x7F;
}


/* i.e. uint16_t <variable_name>; */
// *** CheckForCanMessages ****************************************************************//
// return 1 if a can message was received, 0 otherwise
int CheckForCanMessages( void )
{ // Determines whether or not a CAN message is waiting in the buffer
  if (IsCanCommandAvailable())
  {
    //  OUT1 = !OUT1; // toggle out 1 everytim we get CAN message TODO remove
    can_messages_rcvd++;
    //	GreenLedToggle(LED_ALWAYS);
    retrieve_can_message(&message);
    // put messages into array based on first byte
    return process_can_message(message);
  }
  return 0;
}

int RunLevel5MainFunc(void);

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/


unsigned long GetCANAddressFromBuffer( void )
{  
   return (unsigned long) messages[1].id & 0x000FFFFF;
}

void FlushCanMessages( void )
{
    // flush all pending CAN message
    while (IsCanCommandAvailable())
    {
       retrieve_can_message(&message);
    }
    ClearAllMessages();
}

void ClearAllMessages( void )
{
    int i; //,j;

    for (i=0;i<10;i++)
    {
        memset(&messages[i] ,0xFF ,sizeof(mID));      
    }
}


// if readcmd == 0 then read the door count
// if readcmd == 1 then read the sensor status,
// if readcmd == 2 then read the functional area
// if readcmd == 3 then read nothing
void ReadCount( int readcmd )
{
//    static boolean MsgFlop = false;
    int k;
   // int functionArea;
    
 // Check to see if a status message arrived, but no status is available
    if (CheckForCanMessages() == 1)
    {
       if ((messages[1].data[4] == 'S') &&
           (messages[1].data[5] == 0x30) &&
           (messages[1].data[6] == 'G') &&
           (messages[1].data[7] == 0x00) ) // no status message waiting
       {
           // Sensor Status Query Response
               SensorStatus[GetCanIndex()] = 0; // no status is available
       }
       else if ((messages[1].data[4] == 'c') &&
                (messages[1].data[5] == 0x10) &&
                (messages[1].data[6] == 'B') )
       {           
            // this is a BROADCAST SCAN RESPONSE
           //
           sensorCanAddress = GetCANAddressFromBuffer();
           for (k=0;k<MAX_SENSORS;k++)
           {
               if (SensorAddr[k] == sensorCanAddress)
               {
                   break; // we already have this address
               }
               else if (SensorAddr[k] == 0 )
               {
                  SensorAddr[k] = sensorCanAddress;
                  FuncArea[k] = 0;
                  return;
               }
           }
       }
       else if  ((messages[1].data[4] == 'D') &&
                 (messages[1].data[5] == 0x30) &&
                 (messages[1].data[6] == 'S') )
       {
         // Door State Count Message (who requested this?)
            // this is sent when DoorOpen and DoorClose are requested
       }
       else if  ((messages[1].data[4] == 'C') &&
                 (messages[1].data[5] == 0x60) &&
                 (messages[1].data[6] == 'G') )
       {
         // Count Data Query Response
       }
       else
       {
           k=23; // what message is this
           k=4;
       }
     }


    // check to see if status arrived, and grab the
    // Sensor Status if available
    if (CheckForCanMessages() == 2)
    {
           if ((messages[1].data[4] == 'S') &&
               (messages[1].data[5] == 0x30) &&
               (messages[1].data[6] == 'G') )
           {
               SensorStatus[GetCanIndex()] = messages[2].data[1];
               if (SensorStatus[GetCanIndex()])
               {
                   // rats we have a sabotage message here
                   //
                   k=4;
               }
               //Sabotaged = 1;
               ClearAllMessages();
           }
           else if ((messages[1].data[4] == 'S') &&
               (messages[1].data[5] == 0x31) &&
               (messages[1].data[6] == 'G') )
           {               
               FuncArea[GetCanIndex()] = messages[2].data[1]; 
               //sensorCanAddress = GetCANAddressFromBuffer();
               //ClearAllMessages();
               //for (k=0;k<MAX_SENSORS;k++)
               //{
               //   if (SensorAddr[k] ==sensorCanAddress)
               //   {                   
               //     FuncArea[k] = functionArea;
               //     break;
               //   }
               //}           
           }
     }

     // Check for the people count
    if (CheckForCanMessages() == 3)
    {
        if ((messages[1].data[4] == 'C') &&
            (messages[1].data[5] == 0x60) &&
            (messages[1].data[6] == 'G') )
        {
            k = GetCanIndex();
            FuncArea[k] = messages[1].data[7];            
            PeopleCountIn[k] =  ((uint16)messages[2].data[7])<<8 | (uint16)messages[2].data[6];
            PeopleCountOut[k] = ((uint16)messages[3].data[2])<<8 | (uint16)messages[3].data[1];            
        }        
        ClearAllMessages();
    }

    if ( GetTimerInterrupts() > 50) // every 50 milliseconds read count
    {
        SetTimerInterrupts(0);
        
        switch (readcmd)
        {
            case 0: // read the door count
                ClearAllMessages(); // this may be bad
                SendIRMACounterStateRequest();          
                break;
            case 1: // read the sensor status
                // Update the Sensor Status, to decide if we are sabotaged or not
                //  NOTE: The Sensor Status is only updated when the door is closed
                // and was previously opened for at least 4 seconds
                ClearAllMessages(); // this may be bad
               // SendIRMASensorStateRequest();
                break;
            case 2: // read the functional area
                ClearAllMessages();
               // SendFunctionAreaStatusQuery();
                break;
            case 3: // read nothing
                ClearAllMessages();
                break; 
            case 4: // read count with new method
                ClearAllMessages(); // this may be bad
               // SendIRMACounterStateRequest2(1,2); 
                break; 
        }            
    }
}


boolean ExternalResetPressed( void )
{
    return (Input2Pressed); // || Input2Pressed);
}


boolean ExternalEnablePressed( void )
{
    return (Input1Pressed); // || Input4Pressed);
}

void EstablishLoraModuleConnection( void )
{
    // char c;
    LORA_RESET = 0; //  Disable the LORA module to reset it
     // give 100 milliseconds for the mDot to shut down
    SetWaitDelay(100);    while (GetWaitDelay()!=0);       
    LORA_RESET = 1; //  Enable the LORA module    
     // give 600 milliseconds for the mDot to come to life
    SetWaitDelay(600); while (GetWaitDelay()!=0);
          
    // clear any existing data in the uart
    ClearUartReceiveBuffer();
    InitMDot(); 
    return;           
}


void EstablishCanModuleConnections( void )
{
    int k;
  // establish communications to CAN bus
    while (true) 
    {
        CAN_LED = 1;
        LORA_LED = 1;
         // First Get the CAN address
        SendIRMAGetCanAddress();  
        SetWaitDelay(30);  while (GetWaitDelay()!=0);
        for (k=0;k<MAX_SENSORS;k++)
            ReadCount(3); // get CAN ids from all possiblemessages
      
        if (SensorAddr[0] == 0)
        {            
            // No CAN bus was detected, keep trying forever
            CAN_LED = 0;
            LORA_LED = 1;
            SetWaitDelay(20);  while (GetWaitDelay()!=0);
        }
        else
        {
            break; // Yeah we got at least one address
        }
    }   
}


  int GetFunctionArea(unsigned long addr)
  {
      
   
    int index = GetCanAddrIndex(addr);
    int fa=0;
    
    // scan all function areas looking for a valid response to
    // establish which function area the sensor is on.
    while (FuncArea[index] == 0)
    {
        for (fa=0;fa<255;fa++)
        {
            SendIRMACounterStateRequest2(addr,fa);
            SetWaitDelay(20);    
            while (GetWaitDelay()!=0);       
            ReadCount(3);
            if (FuncArea[index] != 0) 
                return FuncArea[index];
        }
    }
    return 0;
  }
  
  void OpenAllDoors( void )
  {
    int addr;
    for (addr=0;addr<MAX_SENSORS;addr++)
    {
        if (SensorAddr[addr] != 0)
        {
           SendIRMASetDoorsOpen(SensorAddr[addr],FuncArea[addr]);
           SetWaitDelay(100);    
           while (GetWaitDelay()!=0);  
        }
    }
  }
  
void CloseAllDoors( void )
{
   int addr;
   for (addr=0;addr<MAX_SENSORS;addr++)
   {
      if (SensorAddr[addr] != 0)
      {
         SendIRMASetDoorsClose(SensorAddr[addr],FuncArea[addr]); // Close doors to stop counting
         SetWaitDelay(100);    
         while (GetWaitDelay()!=0);  
      }
   }
}  

void ReadAllDoorCounts( void )
{
   int addr;
   for (addr=0;addr<MAX_SENSORS;addr++)
   {
      if (SensorAddr[addr] != 0)
      {
         SendIRMACounterStateRequest2(SensorAddr[addr],FuncArea[addr]);        
         SetWaitDelay(10);    
         while (GetWaitDelay()!=0); 
         ReadCount(3); // fetch the can data
      }
   }
}

  
    
void GetAllFunctionAreas( void )
{
    int addr;
    for (addr=0;addr<MAX_SENSORS;addr++)
    {
        if (SensorAddr[addr] != 0)
        {
           // GetFunctionID(SensorAddr[addr]); // does not work
            FuncArea[addr] = GetFunctionArea(SensorAddr[addr]);
        }
   
            //FuncArea[addr] = GetFunctionArea(SensorAddr[addr]);
    }  
}
  
  
 //int old_incount[MAX_SENSORS];
 //int old_outcount[MAX_SENSORS];
 //int incount[MAX_SENSORS];
 //int outcount[MAX_SENSORS];
 
 void ClearCountHistory( void )
 {
     int k;
     for (k=0;k<MAX_SENSORS;k++)
     { 
        PeopleCountIn[k]  = 0;
        PeopleCountOut[k] = 0;
        LastPeopleCountIn[k]  = 0;
        LastPeopleCountOut[k] = 0;       
     }
 }
 
 // tell the LORA server the startup messages when we bootup.
 void SendStartupLoraMessages( void )
 {
   int addr;
   for (addr=0;addr<MAX_SENSORS;addr++)
   {
      if (SensorAddr[addr] != 0)
      {
         SendLoraPacket( 0xFF, 0, 0, FuncArea[addr] );        
         SetWaitDelay(1500);    
         while (GetWaitDelay()!=0);  
      }
   }  
 }

int16_t main(void)
{    
    int k; 
    int sensorIndex;  // the current sensor index we are querying
    unsigned long sensorAddr;
    int sensorFunctionArea;
    /* Configure the oscillator for the device */
    InitOscillator();
    

    ANSELB = 0x0000;
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    ANSELE = 0x0000;
    ANSELG = 0x0000;

    /* Initialize IO ports and peripherals */
    InitApp(); 
    
    // give time for sensors to come on line
    for (k=0;k<10;k++)
    {
         CAN_LED = 0;  LORA_LED=1; SetWaitDelay(500);  while (GetWaitDelay()!=0);    
         CAN_LED = 1;  LORA_LED=0; SetWaitDelay(500);  while (GetWaitDelay()!=0);    
    }
    
    
    // find out which CAN devices are on the bus
    EstablishCanModuleConnections();      
     // for each matrix sensor get its function area
    GetAllFunctionAreas();
    
    // todo add back in these lines
    CloseAllDoors(); // this will reset all of the counts             
    OpenAllDoors();
    
    EstablishLoraModuleConnection();    
    SendStartupLoraMessages();
    
    // scan all function areas looking for a valid response to
    // establish which function area the sensor is on.
    k=1;
    k=2;
    
   // addr=3;
   // EstablishLoraModuleConnection();    
   // SendStartupLoraMessages();
   // SendIRMASensorFirmwareRestart();    
    // remove this code later, this is a test
   // SetWaitDelay(5000);    
   // while (GetWaitDelay()!=0)
   //     ReadCount(3); // read nothing, clear out old messages in can buffer
   // outcount=3;
    while (true)
    {   
        ReadAllDoorCounts();
        for (sensorIndex=0;sensorIndex<MAX_SENSORS;sensorIndex++)
        {           
            if ( (SensorAddr[sensorIndex] != 0) &&
                 ((PeopleCountIn[sensorIndex] != LastPeopleCountIn[sensorIndex]) ||
                 (PeopleCountOut[sensorIndex] != LastPeopleCountOut[sensorIndex])) )  
            {
                CAN_LED = 0;  //LED2 = 0;
               
                if (FALSE == SendLoraPacket( 0x1, PeopleCountIn[sensorIndex], PeopleCountOut[sensorIndex], FuncArea[sensorIndex] ))
                {
                  // oh oh, we did not receive a proper response
                  // try to reconnect to the Conduit
                    
                    EstablishLoraModuleConnection();
                }
                else
                {
                    // only set the last when we sucessfully transmitted
                    LastPeopleCountIn[sensorIndex]  = PeopleCountIn[sensorIndex];
                    LastPeopleCountOut[sensorIndex] = PeopleCountOut[sensorIndex];
                    LORA_LED = 0; // yes we got a LORA response
                    // slow down transmission of packets to once every 500ms
                    SetWaitDelay(500);    
                    while (GetWaitDelay()!=0);                  
                }
            }
        }
        CAN_LED = 1;  //LED2 = 1;
        LORA_LED = 1; // turn off LORA LED                
    }
    
    
    
  
}


