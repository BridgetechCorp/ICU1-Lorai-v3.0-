#include "ecan.h"
#include "CanIrma.h"

#define LOW_BYTE(x) ((unsigned char)((x)&0xff))
#define HIGH_BYTE(x) ((unsigned char)(((x)>>8)&0xff))

long MakeIrmaEID( long messageid, long receiverclass, long canirmaaddr)
{
    long val = 0;
    val = messageid << 22 | receiverclass<<19 | canirmaaddr;
    return val;
}

void FillInData(mID *msg, unsigned char d0,unsigned char d1, unsigned char d2, unsigned char d3,
                          unsigned char d4, unsigned char d5, unsigned char d6, unsigned char d7  )
{
    (*msg).data[0] = d0;
    (*msg).data[1] = d1;
    (*msg).data[2] = d2;
    (*msg).data[3] = d3;
    (*msg).data[4] = d4;
    (*msg).data[5] = d5;
    (*msg).data[6] = d6;
    (*msg).data[7] = d7;
}

void SendDatagram( long messageid, // typically 0x74
                   long receiverclass, // depends on message id 0x1,0x4, etc..
                   long canirmaaddr, // the bit18 always 0, and lower 17..0 bits of mac id
                   unsigned char UipSublevel, // 0x01=stack, 0x02=Service, 0x03=Application
                   boolean R, // repeat flag
                   unsigned char DK, // message type purpose 'C'=countdata,'D','S','K', etc..
                   unsigned char VER, // version of the message type 0x60,0x32, etc.
                   unsigned char CMD, // subcommand id 'g','r','c'
                   int payloadLengthDL, // length of payload 0 to 1400 bytes
                   unsigned char *payload ) // the payload data
{
     mID txMessage;
     unsigned char uipVersionLevel = (0x30 | UipSublevel);     
     unsigned char PartNumber = 0x01; // this increments for each can msg we send     
     long eid = MakeIrmaEID( messageid, receiverclass, canirmaaddr);
     int index=0;
     int realLength = payloadLengthDL + 3; // 3 extra for the DK,VER,CMD
     
     txMessage.message_type = CAN_FRAME_EXT;    
     txMessage.id = eid;
     if (payloadLengthDL > 0)
        txMessage.data_length = 8;
     else
        txMessage.data_length = 7;
     
     txMessage.data[0] = PartNumber++;
     txMessage.data[1] = uipVersionLevel;    
     txMessage.data[2] = (unsigned char)(realLength & 0x00ff);
     txMessage.data[3] = (unsigned char)((realLength >> 8)& 0x00ff);
     if (R == TRUE)
        txMessage.data[3] |= 0x80;
     txMessage.data[4] = DK;
     txMessage.data[5] = VER;
     txMessage.data[6] = CMD;
     if (payloadLengthDL > 0)
     {
        txMessage.data[7] = *payload++;  payloadLengthDL--;   
     }        
     sendECAN((mID *)&txMessage);
    
     
     while (payloadLengthDL>0)
     {
        if (payloadLengthDL<=7)
        {
            txMessage.data_length = payloadLengthDL+1;
            txMessage.data[0] = PartNumber++;
            for (index=1;index<payloadLengthDL+1;index++)
            {
                txMessage.data[index] = *payload++;
            }    
            payloadLengthDL=0; // we sent all remaining data
            sendECAN(&txMessage);           
        }
        else // send the 7 bytes
        {
            txMessage.data_length = 8;
            txMessage.data[0] = PartNumber++;
            txMessage.data[1] = *payload++; payloadLengthDL--;
            txMessage.data[2] = *payload++; payloadLengthDL--;
            txMessage.data[3] = *payload++; payloadLengthDL--;
            txMessage.data[4] = *payload++; payloadLengthDL--;
            txMessage.data[5] = *payload++; payloadLengthDL--;
            txMessage.data[6] = *payload++; payloadLengthDL--;
            txMessage.data[7] = *payload++; payloadLengthDL--;          
            sendECAN(&txMessage);           
        }
     }     
}
 



void SendIRMAGetCanAddress( void )
{
   mID txMessage;

    long eid;

   // eid = MakeIrmaEID( 0x74, 0x1, 0x0012C);
     //eid = MakeIrmaEID( 0x74, 0x1, 0x00FA1);
                 // message idenetifier = 0x74
                 // receiver class      = 0x01
                 // CAN IRMA address = 0x01d5a
     eid = MakeIrmaEID( 0x1E, 0x7, 0x604DF); // This is GLOBAL address??
     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.id = eid;
     txMessage.data_length = 8;
     FillInData(&txMessage, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 );
     sendECAN(&txMessage);
     //
     eid = MakeIrmaEID( 0x74, 0x5, 0x604DF);// This is GLOBAL address??
     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.id = eid;
     txMessage.data_length = 7;
     FillInData(&txMessage, 0x01, 0x31, 0x03, 0x00, 0x63, 0x10, 0x62, 0x00 );
     sendECAN(&txMessage);
}

void SendIRMACounterStateRequest( void )
{
    mID txMessage;

    long eid;

   // eid = MakeIrmaEID( 0x74, 0x1, 0x0012C);
     //eid = MakeIrmaEID( 0x74, 0x1, 0x00FA1);
                 // message idenetifier = 0x74
                 // receiver class      = 0x01
                 // CAN IRMA address = 0x01d5a
     eid = MakeIrmaEID( 0x74, 0x1, sensorCanAddress);

     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.data[0] = 0x1;
     txMessage.id = eid;
     txMessage.data_length = 8;
     //FillInData(&txMessage, 0x01, 0x33, 0x08, 0x00, 0x43, 0x60, 0x67, 0x01 );
     // todo: see if this requests all function areas
      FillInData(&txMessage, 0x01, 0x33, 0x08, 0x00, 0x43, 0x60, 0x67, 0x00 );
     sendECAN(&txMessage);
     txMessage.data_length = 5;
     FillInData(&txMessage, 0x02, 0x00, 0x00, 0x43, 0xFF, 0x00, 0x00, 0x00 );
     sendECAN(&txMessage);
}


//  ***********************************************************
// Get Sensor State (sabotages etc..)
//  ***********************************************************
void SendIRMASensorStateRequest( unsigned long addr  )
{
     mID txMessage;
     long eid;
                 // message idenetifier = 0x74
                 // receiver class      = 0x01
                 // CAN IRMA address = 0x01d5a
     eid = MakeIrmaEID( 0x74, 0x1, addr);
     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.id = eid;
     txMessage.data_length = 7;
                                 //3L   [ R|DL    ]   DK   VER  CMD
     FillInData(&txMessage, 0x01, 0x33, 0x03, 0x00,   'S', 0x30, 'g', 0x00 );
     sendECAN(&txMessage);
}

void SendIRMASensorFirmwareRestart( void )
{
    long messageid = 0x74; // typically 0x74
    long receiverclass = 0x01; // 0x01=single sensor?? 0x05=all sensors // depends on message id 0x1,0x4, etc..
    long canirmaaddr = sensorCanAddress; // the bit18 always 0, and lower 17..0 bits of mac id
    unsigned char UipSublevel = 0x02;  // 0x01=stack, 0x02=Service, 0x03=Application
    boolean R = FALSE; // repeat flag
    unsigned char DK = 'W'; // message type purpose 'C'=countdata,'D','S','K', etc..
    unsigned char VER = 0x10; // version of the message type 0x60,0x32, etc.
    unsigned char CMD = 'r'; // subcommand id 'g','r','c'
    int payloadLengthDL = 0; // length of payload 0 to 1400 bytes
    unsigned char payload[] = { 0 };  
    
    SendDatagram( messageid, // typically 0x74
                       receiverclass, // depends on message id 0x1,0x4, etc..
                       canirmaaddr, // the bit18 always 0, and lower 17..0 bits of mac id
                       UipSublevel, // 0x01=stack, 0x02=Service, 0x03=Application
                       R, // repeat flag
                       DK, // message type purpose 'C'=countdata,'D','S','K', etc..
                       VER, // version of the message type 0x60,0x32, etc.
                       CMD, // subcommand id 'g','r','c'
                       payloadLengthDL, // length of payload 0 to 1400 bytes
                       &payload[0] ); // the payload data   
}



// Send count data reset command.
void SendIRMACountDataReset( unsigned long addr  )
{
    mID txMessage;

    long eid;

   // eid = MakeIrmaEID( 0x74, 0x1, 0x0012C);
     //eid = MakeIrmaEID( 0x74, 0x1, 0x00FA1);
                 // message idenetifier = 0x74
                 // receiver class      = 0x01
                 // CAN IRMA address = 0x01d5a
     eid = MakeIrmaEID( 0x74, 0x1, addr);

       // sent extended ide
     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.id = eid;
     txMessage.data_length = 8;
       // data[0] = 0x01 datagram Number 1,2,3 etc..
       // data[1] = 0x33 UIP version number and level = 0x33
       // data[2] = 0x08 bits 0..7 of DL (length of message)
       // data[3] = 0x00 R and bits 14..8 of DL (length of message)
       // data[4] = 0x43 'C'
       // data[5] = 0x60 0x60
       // data[6] = 0x67 'g'
       // data[7] = 0x01 FA16 (Door 1)
     FillInData(&txMessage, 0x01, 0x33, 0x08, 0x00, 'C', 0x60, 'r', 0x01 );
     sendECAN(&txMessage);
     txMessage.data_length = 3;
     FillInData(&txMessage, 0x02, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00 );
     sendECAN(&txMessage);
}


void SendIRMASetDoorsOpen(unsigned long addr, int functionArea )
{
    mID txMessage;
    long eid;

                 // message idenetifier = 0x74
                 // receiver class      = 0x01
                 // CAN IRMA address = 0x01d5a
     eid = MakeIrmaEID( 0x74, 0x1, addr);

     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.id = eid;
     txMessage.data_length = 8;
     FillInData(&txMessage, 0x01, 0x33, 0x0A, 0x00, 'D', 0x30, 's', 0x01 );
     sendECAN(&txMessage);
     txMessage.data_length = 7;
                                  // fa1         fa2  door1        door2 leftwing rightwing
     FillInData(&txMessage, 0x02, functionArea, 0x00, functionArea, 0x00, 100,    100, 0x00 );
     //FillInData(&txMessage, 0x02, 0x01, 0x00, 0x01, 0x00, 100, 100, 0x00 );
     sendECAN(&txMessage);
}

void SendIRMASetDoorsClose(unsigned long addr, int functionArea )
{
    mID txMessage;
    long eid;

                 // message idenetifier = 0x74
                 // receiver class      = 0x01
                 // CAN IRMA address = 0x01d5a
     eid = MakeIrmaEID( 0x74, 0x1, addr);

     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.id = eid;
     txMessage.data_length = 8;
     FillInData(&txMessage, 0x01, 0x33, 0x0A, 0x00, 'D', 0x30, 's', 0x01 );
     sendECAN(&txMessage);
     txMessage.data_length = 7;
                                // fa1         fa2     door1        door2 leftwing rightwing
     FillInData(&txMessage, 0x02, functionArea, 0x00, functionArea, 0x00, 0x0, 0x0, 0x00 );
     sendECAN(&txMessage);
}

void SendFunctionAreaStatusQuery( unsigned long addr )
{
    long messageid = 0x74; // typically 0x74
    long receiverclass = 0x01; // 0x01=single sensor?? 0x05=all sensors // depends on message id 0x1,0x4, etc..
    long canirmaaddr = addr; // the bit18 always 0, and lower 17..0 bits of mac id
    unsigned char UipSublevel = 0x03;  // 0x01=stack, 0x02=Service, 0x03=Application
    boolean R = FALSE; // repeat flag
    unsigned char DK = 'S'; // message type purpose 'C'=countdata,'D','S','K', etc..
    unsigned char VER = 0x31; // version of the message type 0x60,0x32, etc.
    unsigned char CMD = 'g'; // subcommand id 'g','r','c'
    int payloadLengthDL = 0; // length of payload 0 to 1400 bytes
    unsigned char payload[] = { 0 };  
    
    SendDatagram( messageid, // typically 0x74
                       receiverclass, // depends on message id 0x1,0x4, etc..
                       canirmaaddr, // the bit18 always 0, and lower 17..0 bits of mac id
                       UipSublevel, // 0x01=stack, 0x02=Service, 0x03=Application
                       R, // repeat flag
                       DK, // message type purpose 'C'=countdata,'D','S','K', etc..
                       VER, // version of the message type 0x60,0x32, etc.
                       CMD, // subcommand id 'g','r','c'
                       payloadLengthDL, // length of payload 0 to 1400 bytes
                       &payload[0] ); // the payload data
}

void SendIRMACounterStateRequest2( unsigned long addr, int functionArea )
{
    long messageid = 0x74; // typically 0x74
    long receiverclass = 0x01; // 0x01=single sensor?? 0x05=all sensors // depends on message id 0x1,0x4, etc..
    long canirmaaddr = addr; //sensorCanAddress; // the bit18 always 0, and lower 17..0 bits of mac id
    unsigned char UipSublevel = 0x03;  // 0x01=stack, 0x02=Service, 0x03=Application
    boolean R = FALSE; // repeat flag
    unsigned char DK = 'C'; // message type purpose 'C'=countdata,'D','S','K', etc..
    unsigned char VER = 0x60; // version of the message type 0x60,0x32, etc.
    unsigned char CMD = 'g'; // subcommand id 'g','r','c'
    int payloadLengthDL = 5; // length of payload 0 to 1400 bytes
                               
    
    unsigned char fa1 = (unsigned char)(functionArea & 0x00ff);
    unsigned char fa2 = (unsigned char)((functionArea>>8) & 0x00ff);
                                 // fa16    resetflag   counttype  categoryid
    unsigned char payload[] = {    fa1, fa2,  0x00,      0x43,        0xff };  
    //unsigned char payload[] = { 0x01, 0x00,  0x00,      0x43,        0xff };  
                
    SendDatagram( messageid, // typically 0x74
                       receiverclass, // depends on message id 0x1,0x4, etc..
                       canirmaaddr, // the bit18 always 0, and lower 17..0 bits of mac id
                       UipSublevel, // 0x01=stack, 0x02=Service, 0x03=Application
                       R, // repeat flag
                       DK, // message type purpose 'C'=countdata,'D','S','K', etc..
                       VER, // version of the message type 0x60,0x32, etc.
                       CMD, // subcommand id 'g','r','c'
                       payloadLengthDL, // length of payload 0 to 1400 bytes
                       &payload[0] ); // the payload data       
}

void SendConfigurationDataQuery( unsigned long addr, unsigned int groupId, unsigned int paramId )
{      
    mID txMessage;
    long eid;

                 // message idenetifier = 0x76 for service
                 // receiver class      = 0x01
                 // CAN IRMA address = 0x01d5a
     eid = MakeIrmaEID( 0x76, 0x1, addr);

     txMessage.message_type = CAN_FRAME_EXT;
     txMessage.id = eid;
     txMessage.data_length = 8;
     FillInData(&txMessage, 0x01, 0x32, 0x07, 0x00, 'K', 0x10, 'g', HIGH_BYTE(groupId) );
     sendECAN(&txMessage);
     txMessage.data_length = 4;
                                // fa1         fa2     door1        door2 leftwing rightwing
     FillInData(&txMessage, 0x02, LOW_BYTE(groupId), HIGH_BYTE(paramId), LOW_BYTE(paramId), 0x00, 0x0, 0x0, 0x00 );
     sendECAN(&txMessage);
    
}

void GetFunctionID( unsigned long addr )
{
    unsigned char low = LOW_BYTE(0x1234);
    unsigned char hi = HIGH_BYTE(0x1234);
   SendConfigurationDataQuery(addr,0x700,0x100);
}
