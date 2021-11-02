////////////////////////////////////////////////////////////////////////////////
////                               EX_J1939.c                               ////
////                                                                        ////
//// Example of CCS's J1939 driver.  This example was tested using the CCS  ////
//// CAN Bus 24 Development kit.                                            ////
////                                                                        ////
//// This example will send a message once every second commanding Node B   ////
//// to toggle it's LED once every second.  Requires that EX_J1939B.c be    ////
//// programmed onto Node B of the development kit.  Also pressing the push ////
//// will cause Node A to send out a Global Address Request, causing all    ////
//// device to respond with their claimed address.  Which will be displayed ////
//// over RS232.                                                            ////
////                                                                        ////
//// This example will work with the PCM, PCH and PCD compilers.  The       ////
//// is written to work with the PCD and PCH compiler.  Change the device,  ////
//// clock, RS232 settings, PIN defines and tick timer setting for hardware ////
//// if needed.                                                             ////
////                                                                        ////
////////////////////////////////////////////////////////////////////////////////
////        (C) Copyright 1996,2012 Custom Computer Services                ////
//// This source code may only be used by licensed users of the CCS         ////
//// C compiler.  This source code may only be distributed to other         ////
//// licensed users of the CCS C compiler.  No other use,                   ////
//// reproduction or distribution is permitted without written              ////
//// permission.  Derivative programs created using this software           ////
//// in object code form are not restricted in any way.                     ////
////////////////////////////////////////////////////////////////////////////////

#if defined(__PCD__)
#include <24HJ256GP610.h>
#elif defined(__PCH__)
#include "18F4580.h"
#endif
#fuses NOWDT
#use delay(crystal=16MHz)
#use rs232(UART1,baud=9600)

#define LED_PIN      PIN_B1
#define PUSH_BUTTON  PIN_A4

#include <stdint.h>

void InitJ1939Address(void);
void InitJ1939Name(void);

//////////////////////////////////////////////////////////////////////////////// Tick Timer
#define TICKS_PER_SECOND 1000

typedef uint32_t TICK_TYPE;
static TICK_TYPE TimerTick;

TICK_TYPE GetTick(void)
{
   return(TimerTick);
}

TICK_TYPE GetTickDifference(TICK_TYPE current,TICK_TYPE previous)
{
   return(current - previous);
}

#INT_TIMER2
void tick_time_isr(void)
{
   TimerTick++;
}

//////////////////////////////////////////////////////////////////////////////// J1939 Settings

//Following Macros used to initialize unit's J1939 Address and Name - Required
#define J1939InitAddress()    InitJ1939Address()
#define J1939InitName()       InitJ1939Name()

//Following defines what the CAN's baud rate it, not required defaults to 250Kbit 
//only necessary if using non standard baud rate.  Was changed to 125Kbit to
//work on CCS CAN Bus and CAN Bus 24 development kit.

//#define  Set_1000K_Baud TRUE
//#define  Set_500K_Baud TRUE
#define  Set_250K_Baud TRUE
//#define  Set_200K_Baud True
//#define  Set_125K_Baud True
//#define J1939_BAUD_RATE    125000 // del original para 20Mhz

//Following defines sets up the CAN baud rate, not required if using 250Kbit or 
//500Kbit and clock rates of 8, 16, 20, 32 or 40MHz.
#if defined(__PCD__)
#define CAN_BRG_PRESCALAR           4
#define CAN_BRG_PHASE_SEGMENT_1     2
#define CAN_BRG_PHASE_SEGMENT_2     2
#define CAN_BRG_SYNCH_JUMP_WIDTH    0
#define CAN_BRG_PROPAGATION_TIME    0
#elif defined(__PCH__)
#define CAN_BRG_PRESCALAR           4
#define CAN_BRG_PHASE_SEGMENT_1     6
#define CAN_BRG_PHASE_SEGMENT_2     6
#define CAN_BRG_SYNCH_JUMP_WIDTH    0
#define CAN_BRG_PROPAGATION_TIME    0
#endif

//Following defines/macros used to associate your tick timer to J1939 tick timer
// defines/macro's - Required
#define J1939GetTick()                 GetTick()
#define J1939GetTickDifference(a,b)    GetTickDifference(a,b)
#define J1939_TICKS_PER_SECOND         TICKS_PER_SECOND
#define J1939_TICK_TYPE                TICK_TYPE

//Include the J1939 driver
#include "j1939.c"

//Defines for J1939 Commands used in this example
#define LED_ON       50
#define LED_OFF      51
#define LED_TOGGLE   52

//Define for other Node's J1939 address used in this example
#define OTHER_NODE_ADDRESS    129

//Function used to initialize this unit's J1939 Address
void InitJ1939Address(void)
{
   g_MyJ1939Address = 128;
}

//Function used to initialize this unit's J1939 Name
void InitJ1939Name(void)
{
   g_J1939Name[0] = 0;
   g_J1939Name[1] = 0;
   g_J1939Name[2] = 0;
   g_J1939Name[3] = 0;
   g_J1939Name[4] = 0;
   g_J1939Name[5] = 0;
   g_J1939Name[6] = 0;
   g_J1939Name[7] = 128;
}


//IMPLEMENTACIÓN DE LAS FUNCIONES PARA SPN Y PGN
//PARAMETROS PGN
#define PGN_DASH_DISPLAY                   0xFEFC
#DEFINE PGN_ELECTRONIC_ENGINE_CONTROLLER_1 0XF004
#DEFINE PGN_FUEL_ECONOMY                   0XFEF2
#DEFINE PGN_ENGINE_TEMPERATURE             0XFEEE
#define PGN_AMBIENT_CONDITIONS             0xFEF5
#DEFINE PGN_VEHICLE_POSITION               0xFEF3



//PARAMETROS SPN

#DEFINE SPN_ENGINE_THROTTLE_POSITION      51
#DEFINE SPN_FUEL_LEVEL_1                  96
#DEFINE SPN_ENGINE_COOLANT_TEMPERATURE    110
#DEFINE SPN_ENGINE_FUEL_TEMPERATURE_1     174
#DEFINE SPN_ENGINE_OIL_TEMPERATURE_1      175
#DEFINE SPN_ENGINE_FUEL_RATE              183
#DEFINE SPN_ENGINE_SPEED                  190


//#######################################
// FUNCIONES PARA EL CALCULO DE LAS SEÑALES 

int16 engineFuelRate(int8 Bytes[]){ /*PGN = 65266 = 0xFEF2   IDE = 18FEF200*/
    float resolution  = 0.05;
    float offset      = 0;
    int16 auxFuelRate   = 0; /*ENTERO PARA DESPLAZAR LUEGO LOS BITS*/
    float fuelRate    = 0;
        auxFuelRate   = Bytes[0]; /*Bytes[1]*/
        auxFuelRate   = auxFuelRate<<8 | Bytes[1]; /*Bytes[2]*/
        fuelRate      = auxFuelRate*resolution + offset;
    return (int16)fuelRate;
}

int16 fuelTemperature (int8 Bytes[]){ /*PGN = 65262 = 0xFEEE  IDE=0x18FEEE00*/
    float resolution         =   1;
    float offset             = -40;
    float fuelTemperature    =   0;
          fuelTemperature    = Bytes[1]*resolution + offset; /*#define FUEL_TEMPERATURE  1*/
    return (int16)fuelTemperature;
}


int16 engineSpeed(int8 Bytes[]){ /*PGN = 61444 = 0xF004   IDE=0x18F00400*/
    float resolution  = 0.125;
    float offset      = 0;
    float velocidad   = 0;
    int16 auxVelocidad  = 0;                                       /*DECLARACION ENTERA PARA DESPLAZAR LOS BITS*/
        auxVelocidad  = Bytes[3];                   /*#define ENGINE_SPEED_HIGH 3 , Bytes_4*/
        auxVelocidad  = auxVelocidad <<8 | Bytes[4]; /*#define ENGINE_SPEED_LOW  4 , Bytes_5*/
        velocidad     = auxVelocidad*resolution + offset;
    return (int16)velocidad;
}

int16 fuelLevel(int8 Bytes[]){ /*FEFC*/
    /*DATOS*/
    float resolution = 0.4;
    float offset     = 0;
    /*VARIABLES AUXILIARES*/
    float nivelCombustible=0;
    /*FORMULACION O PREVIA*/
    nivelCombustible= ((float)Bytes[1])*resolution + offset;
    /*ASIGNACION*/
    /*RETORNO*/
    return (int16)nivelCombustible;
}

int16 throttlePosition(int8 Bytes[]){ /*PGN = 65266 = 0xFEF2  IDE = 0x18FEF200*/
    float resolution  = 0.4;
    float offset      = 0;
    float throttleAux;
          throttleAux     = Bytes[6]*resolution + offset; /*#define THROTTLE          6*/
    return (int16)throttleAux;
}



//###### bloque de la funcion spn y pgn#################################
void lecturaDelParametro(int16 pgn, int spn, int8 Bytes[], int16* dato)
{  
   switch (pgn)
   {
      //case PGN_VEHICLE_POSITION:       *dato = 100; break;
      //case PGN_AMBIENT_CONDITIONS:     *dato = 200; break;
      case PGN_DASH_DISPLAY: 
            switch (spn)
            {
               case SPN_FUEL_LEVEL_1:  *dato = fuelLevel(Bytes); break;
               //default: *dato = 0;
            }
            break;
            //FIN DE PGN_DASH_DISPLAY
      case PGN_ELECTRONIC_ENGINE_CONTROLLER_1: 
            switch(spn)
            {
               case SPN_ENGINE_SPEED : *dato = engineSpeed(Bytes); break;
               //default:*dato = 0;
               
            }
               
            break;
      case PGN_FUEL_ECONOMY: 
            switch(spn)
            {
               case SPN_ENGINE_FUEL_RATE:          *dato = engineFuelRate(Bytes); break;
               case SPN_ENGINE_THROTTLE_POSITION:  *dato = throttlePosition(Bytes); break;
               //default:*dato = 0;
            }
           break;
      case PGN_ENGINE_TEMPERATURE: 
           switch (spn)
           {
            case SPN_ENGINE_COOLANT_TEMPERATURE:   *dato = SPN_ENGINE_COOLANT_TEMPERATURE; break;
            case SPN_ENGINE_FUEL_TEMPERATURE_1:    *dato = fuelTemperature (Bytes);  break;
            case SPN_ENGINE_OIL_TEMPERATURE_1:     *dato = SPN_ENGINE_OIL_TEMPERATURE_1;   break;
            //default:*dato = 0;
           }
            break;
      
      default: *dato = 0; break;
      
   }
}

// FIN DEL BLOQUE DE LAS FUNCIONES SPN Y PGN
/*###########
VARIABLE GLOBAL DEFINIDA PARA RECIBIR DATOS DEL COMPUTADOR
*/
int rcvPID = 0x0d; 

//J1939 Task function for this example
void J1939Task(void)
{  
   int16 dato; 
   float dato_f; 
   int16 pgn;
   uint8_t i;
   uint8_t Data[8];
   uint8_t Length;
   J1939_PDU_STRUCT Message;
   
   //TRANSMISOR
   uint8_t sendData[8];
   J1939_PDU_STRUCT MessageT;
   //>>> 
   
   
   J1939ReceiveTask();  //J1939ReceiveTask() needs to be called often
   //J1939XmitTask();     //J1939XmitTask() needs to be called often
   
   if(J1939Kbhit())  //Checks for new message in J1939 Receive buffer
   {
      J1939GetMessage(Message,Data,Length);  //Gets J1939 Message from receive buffer
      
      pgn = Message.PDUFormat;
      pgn = pgn <<8 | (int16)Message.DestinationAddress;
     
     //<<< MODO CONSULTA
     if (rcvPID == 0xfe){
      //send message to other unit once every 250ms to toggle pin
      MessageT.SourceAddress = g_MyJ1939Address;          //set PDU Source Address, this units address (g_MyJ1939Address)
      MessageT.DestinationAddress = 0x00;   //set PDU Destination Address, address of other unit
      MessageT.PDUFormat = 0xEA; //0xEA                    //set PDU Formate, LED_TOGGLE command
      MessageT.DataPage = 0;                              //set PDU Data Page can be either 0 or 1, this message uses 0
      MessageT.ExtendedDataPage = 0;                      //set PDU Extended Data Page, must be zero for J1939 Messages
      MessageT.Priority = J1939_CONTROL_PRIORITY;         //set Priority, can be 0 to 7 (0 highest priority) Control default is 3
      //lista
      /*
      0x00FEF2: Fuel economy
      0x00FEFC: VIN
      0x00FEFF: water in fuel indicator WIF
      */
      sendData[0] = 0xFC;
      sendData[1] = 0xFE;
      sendData[2] = 0x00;
      J1939PutMessage (MessageT,sendData,3);//loads J1939 Message into Xmit buffer
      delay_ms(2000);
      J1939XmitTask();     //J1939XmitTask() needs to be called often
      printf("{\"PGN\": \"%LX\",  \"DA\": \"%x\",  \"SA\": \"%x\",   \"p\": %d,  \"Data\":[ \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\"],\"valor\":%d}\r\n", pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],Data[7],100);
      
     }
     //>>> MODO CONSULTA
     
     if (rcvPID == 0xff){
      printf("{\"PGN\": \"%LX\",  \"DA\": \"%x\",  \"SA\": \"%x\",   \"p\": %d,  \"Data\":[ \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\"],\"valor\":%d}\r\n", pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],Data[7],100);
     }
     
     if(pgn==PGN_FUEL_ECONOMY && rcvPID==0x5e){
        //SPN_ENGINE_FUEL_RATE: SPN_ENGINE_THROTTLE_POSITION
        lecturaDelParametro(pgn,SPN_ENGINE_FUEL_RATE,Data, &dato );
        //printf("PGN: %LX --- dato: %X --- rcvPID: %x\r\n", pgn,dato,rcvPID);
        //printf("{\"valor\": \"%Ld\"}\r\n",dato);
        //printf("{\"PGN\": \"%LX\",  \"DA\": \"%x\",  \"SA\": \"%x\",   \"p\": %d,  \"Data\":[ \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\"],\"valor\":%d}\r\n", pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],Data[7],dato);
        //pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],Data[7],dato);
        printf("{\"PGN\": \"%LX\",  \"DA\": \"%x\",  \"SA\": \"%x\",   \"p\": %d,  \"Data\":[ \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\"],\"valor\":\"%x\"}\r\n", pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[2],Data[3],Data[4],Data[3],Data[2],Data[1],Data[0],dato);
        //printf("{ \"SA\": %x}\r\n",Message.SourceAddress);
     }
     
     if(pgn==PGN_DASH_DISPLAY && rcvPID==0x2f){
        //SPN_FUEL_LEVEL_1
        lecturaDelParametro(pgn,SPN_FUEL_LEVEL_1,Data, &dato );
        //printf("PGN: %LX --- dato: %X --- rcvPID: %x\r\n", pgn,dato,rcvPID);
        //printf("{\"valor\": \"%Ld\"}\r\n",dato);
        printf("{\"PGN\": \"%LX\",  \"DA\": \"%x\",  \"SA\": \"%x\",   \"p\": %d,  \"Data\":[ \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\"],\"valor\":\"%x\"}\r\n", pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],Data[7],dato);
     }
     
     if(pgn==PGN_ELECTRONIC_ENGINE_CONTROLLER_1 && rcvPID==0x0d){
         // SPN_ENGINE_SPEED
        lecturaDelParametro(pgn,SPN_ENGINE_SPEED,Data, &dato );
        //printf("PGN: %LX --- dato: %X --- rcvPID: %x\r\n", pgn,dato,rcvPID);
        //dato_f = (float)dato;
        //printf("{\"valor\": \"%f\"}\r\n",dato_f);
        printf("{\"PGN\": \"%LX\",  \"DA\": \"%x\",  \"SA\": \"%x\",   \"p\": %d,  \"Data\":[ \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\"],\"valor\":\"%x\"}\r\n", pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],Data[7],dato);
     }
      
      if(pgn==PGN_ENGINE_TEMPERATURE && rcvPID==0x5c){
        //SPN_ENGINE_COOLANT_TEMPERATURE: SPN_ENGINE_FUEL_TEMPERATURE_1  SPN_ENGINE_OIL_TEMPERATURE_1
        lecturaDelParametro(pgn,SPN_ENGINE_FUEL_TEMPERATURE_1,Data, &dato );
        //printf("PGN: %LX --- dato: %X --- rcvPID: %x\r\n", pgn,dato,rcvPID);
        //printf("{\"valor\": \"%Ld\"}\r\n",dato);
        printf("{\"PGN\": \"%LX\",  \"DA\": \"%x\",  \"SA\": \"%x\",   \"p\": %d,  \"Data\":[ \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\", \"%x\"],\"valor\":\"%x\"}\r\n", pgn,Message.DestinationAddress, Message.SourceAddress,Message.Priority,Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],Data[7],dato);
     }
     
   

   }      // END KBHIT()
   
  
}//-j1939Task()

/*################## OBTENEMOS EL PID DEL SERVIDOR #######################################*/
//VARIABLES GLOBALES PARA OBTENER EL PID DEL SERVIDOR
int valor; 
int16 ayuda=0; 


void procesar (int vector[]){
  int cnt_x = 0; 
  
 while(cnt_x<3){
  valor = vector[cnt_x];//almaceno el primer valor, luego el siguiente
  cnt_x++;
    
   if(valor>='0' && valor<='9') {
      ayuda = ayuda*0x10; 
      ayuda = ayuda + valor -'0'; 
   }
   else if (valor>='A' && valor<='F'){
      ayuda=ayuda*0x10;
      ayuda = ayuda + 10 + valor -'A';
   }
   else if (valor>='a' && valor<='f'){
      ayuda=ayuda*0x10;
      ayuda = ayuda + 10 + valor -'a';
   }
   else if (valor =='t'){
      rcvPID = ayuda; 
   }
 }//end while cnt_x
} //end f-procesar

//######################### VARIABLES USADOS EN LA INTERRUPCIÓN
int recivido = 0;
const int len=5;
int buff[len]="0ctx"; 
int control=0;

/*######################### TRATAMIENTO DE INTERRUPCIÓN  */
#int_rda
void serial_isr() {                    // Interrupción recepción serie USART
   recivido = getc();                     //borra la bandera de interrupción al extraer el dato
   buff[control++]=recivido;                  
   if (control==4){control=0; buff[len-1]=0x00;}//aquí: buff[cnt]=0x00 caracter NULL                 //limite de datos recibidos
   if (recivido=='i'){control=0;}// reinicia el conteo
}
/*######################### 
PROCESAMIENTO DE LOS DATOS
RECIBIDOS EN LA INTERRUPCIÓN*/




void main()
{
  #if defined(__PCD__)
   setup_timer2(TMR_INTERNAL,10000); //1ms tick
  #else
   setup_timer_2(T2_DIV_BY_4, 250, 5); //1ms tick
  #endif
   enable_interrupts(INT_TIMER2);
  #if defined(__PCD__)
   enable_interrupts(INTR_GLOBAL);
  #else
   enable_interrupts(GLOBAL);
  #endif
   
   enable_interrupts(INT_RDA);      //Activa la interrupción serial
   J1939Init();  //Initialize J1939 Driver must be called before any other J1939 function is used
   
      
   while(TRUE)
   {
      procesar(&buff);//actualizo el rcvPID
      
      J1939Task();
      delay_ms(50); 
      
   }
}
