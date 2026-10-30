//Firmware para controle da placa de controle de velocidade LAT Ver00 em 09/10/2026
//Adição de proteção contra curtos com alarme sonoro (BEEP)
//Adicionado ModBus com integração do sistema
//Adicionado medição de rpm e fins de curso
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#define ENDPADRAO 1
#define ENDWRITEMAX 10
#define SYNC 6
#define RETPOT A2
#define OCPROT A1
#define BEEP 2
#define TXRX 3
#define LED 12
#define MOTOR 4
#define ROTSENS 8
#define FC1 5
#define FC2 7
#define WP 5
#define VELMAX 512
#define VELMIN 30
#define POTVMAX 1020
#define POTVMIN 93
#define TOPCOUNT 130
#define MINCOUNT 10
#define RETCLEARTEMPO 30000
#define PI 3.1415926535897932384626433832795
#define VRMSMAX 0.70710678118654752440084436210485
#define VMEDIAMAX 0.63661990481259183242126921637183
#define TMEIAONDA 8333 //us
#define LEDON digitalWrite(LED,HIGH)
#define LEDOFF digitalWrite(LED,LOW)
#define MOTORON digitalWrite(MOTOR,HIGH)
#define MOTOROFF digitalWrite(MOTOR,LOW)
#define TX485 digitalWrite(TXRX,HIGH)
#define RX485 digitalWrite(TXRX,LOW)

//Mapa Memoria ModBus
#define ParEndSensor 0
#define ParDelayModbus 1
#define ParEepromCheck 2
#define ParCalcVcarga 3
#define ParTriacSW 4
#define ParManDig 5
#define ParPotDigVal 6
#define ParOcFlagAutClear 7
#define ParOCn 8
#define ParVmax 9
#define ParVmin 10
#define ParPulsoRede 11
#define ParPotVal 12
#define ParSensorFC1 13
#define ParSensorFC2 14
#define ParVelMotor 15
#define ParOcPulse 16
#define ParOcFlag 17
#define ParRetFlag 18





#include <PinChangeInterrupt.h>

//Variáveis do ModBus
volatile unsigned int Mapa[30];
unsigned int CRClow, CRChigh;
unsigned int RxModbus[8];
unsigned int RspModbus[50];
unsigned int Rxi=0;
volatile int RxTime=0;
unsigned int RxPisca=1;
volatile char TxR=0;
volatile char CmdModbusIn=0;

//Variáveis ReadHoldingRegisters
unsigned int EndIni,QtdReg;

//Variáveis WriteSingleRegister
unsigned int ValueRegister;
unsigned int PermiteGravar=1;

//Variáveis de tempo
volatile unsigned long Ms=0;
unsigned long retFlagClear=0;

//Variaveis Controle de Potencia
bool syncON = false;
int nPulse=0;
int ocPulse=0;
bool ocFlag=false;
bool flagRet=false;

//variaveis do sensor de rotacao
bool rotSensToggle=false;
long rotSensStamp=0;
long timeStampMs=0;


void setup() {
  
  TCCR0A = 0; // Clear normal/PWM defaults
  TCCR0B = 0; // Stop timer
  TCNT0  = 0; // Clear counter

  TCCR0A |= 0b10000000;//Clear on Match A 
  TCCR0B |= 0b00000101;//Clkio/1024 (64us/count - Max = 16320 us)
  //TCNT0 = 0; //Zera contador
  OCR0A = 120;//Comparador A em 65
  OCR0B = 125;//Comparador B em 80
  //TIMSK0 = 0b00000110;//Habilita interrupção nas comparações A e B

  configSerialModbus();

  pinMode(LED, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(SYNC,INPUT);
  pinMode(OCPROT,INPUT);
  pinMode(RETPOT,INPUT);
  pinMode(BEEP,OUTPUT);
  pinMode(TXRX,OUTPUT);
  pinMode(ROTSENS,INPUT);
  pinMode(FC1,INPUT_PULLUP);
  pinMode(FC2,INPUT_PULLUP);

  digitalWrite(MOTOR,LOW);
  digitalWrite(BEEP,LOW);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SYNC), isr_syncHandle, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(OCPROT), isr_ocprotHandle, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(RETPOT), isr_retpotHandle, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ROTSENS), isr_rotSensHandle, FALLING);

  
  checkModBusMap();

  RX485;//Define direção para Receber comando

  beep(1);


}

// the loop function runs over and over again forever
void loop() {


  if(syncON)
  {
    potUpdate();
    syncON=false;
  }

  if(CmdModbusIn==1)
  RecebeCmdModbus();
  
  fcsUpdate();
  
  Atraso(100);


}

void Atraso(unsigned int Ta){
   
Ms=0;
  
while(Ms<Ta){} 
  
  
}

//Ticking 1 ms
ISR(TIMER2_COMPB_vect)
{

RxTime++;//Controla tempo entre caracteres Modbus
Ms++;
timeStampMs++;
retFlagClear++;

if(timeStampMs>6000)
Mapa[ParVelMotor]=0;

if(Mapa[ParRetFlag]&&retFlagClear>RETCLEARTEMPO)
{Mapa[ParRetFlag]=0;flagRet=false;}

  
}

//Interrupção para receber comandos Modbus
ISR(USART_RX_vect)
{

//Limpa array se não for comando inteiro de 8 bytes  
if((Rxi>0)&&(RxTime>50))
Rxi=0;

RxModbus[Rxi]=UDR0;
Rxi++;

if(Rxi==8){
CmdModbusIn=1;
Rxi=0;
}
 
RxTime=0;

}

void configSerialModbus()
{

   //Ticking 1 ms
  TCCR2A=0;
  TCCR2A |= 0b00000011;
  TCCR2B=0;
  TCCR2B |= 0b00001100;//divide clock em 64
  TIMSK2 = 0;
  TIMSK2 |= 0b00000100;
  OCR2A=250;
  OCR2B=125;
  //Fim 

  //Configuração da porta serial 8 bits 9600
  UCSR0B |= 0b10011000; //Habilita Int.,Rx e Tx
  UBRR0L = 103; //BaudRate de 9600
  UBRR0H = 0;   //BaudRate de 9600
  pinMode(0,INPUT);
  pinMode(1,OUTPUT);

}

void checkModBusMap()
{
  //Verifica se existe mapa na EEPROM
  Mapa[ParEepromCheck]=LeParam(ParEepromCheck);
  
  if(Mapa[ParEepromCheck]==0xAAAA)
    {
    
    for(int i=0;i<30;i++)
    Mapa[i]=LeParam(i); 
    
    }
  else 
    {
    GravaMapaDefault();
    
    for(int i=0;i<30;i++)
    Mapa[i]=LeParam(i);
    
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%     Rotina para cálculo do CRC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void CRC16(unsigned int index,unsigned int *aponta){

  unsigned int CRC=0xFFFF, poli=0xA001,LSB;

while(index){

CRC=CRC^*aponta; //XOR entre CRC e byte recebido

for(int i=0;i<8;i++){
LSB=CRC & 0x0001; //Pega valor de LSB
CRC=CRC>>1; //Desloca CRC um bit a direita;
CRC=CRC & 0x7FFF; //Zera MSB
if(LSB==1)
CRC=CRC^poli;}

aponta++; //aponta para o proximo byte
index--; //decrementa indice
}

//divide em dois bytes
CRClow=CRC & 0x00FF; 
CRChigh=CRC & 0xFF00;
CRChigh=CRChigh>>8;
CRChigh=CRChigh & 0x00FF;

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%  Envia dados pela serial %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TxEnvia(unsigned int OutByte){
  
  unsigned char DataTx;
  
  DataTx=(unsigned char)OutByte;
  
TxR=0;

while(TxR==0){
TxR = UCSR0A & 0b00100000;
}

UDR0=DataTx; 

 
}


void RecebeCmdModbus(void){

 
if(RxModbus[0]==Mapa[ParEndSensor]){

//beep(1);
  
CRC16(6,RxModbus);

if((RxModbus[6]==CRClow)&&(RxModbus[7]==CRChigh)){
  
  
if(RxModbus[1]==3)
ReadHoldingRegister();

if(RxModbus[1]==6)
WriteSingleRegister();
  
  
}else Atraso(10); //Para ver o Led piscar

}else Atraso(10); //Para ver o Led piscar

CmdModbusIn=0;


}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ReadHoldingRegister(void){


EndIni=RxModbus[2];
EndIni<<=8;
EndIni+=RxModbus[3]; 

QtdReg=RxModbus[4];
QtdReg<<=8;
QtdReg+=RxModbus[5];  

RspModbus[0]=RxModbus[0];
RspModbus[1]=RxModbus[1];
RspModbus[2]=2*QtdReg;

int i=0,j=0;

for(i=3, j=0;j<QtdReg;i+=2,j++){
  
RspModbus[i]=Mapa[EndIni+j];

RspModbus[i]=RspModbus[i] & 0xFF00;
RspModbus[i]=RspModbus[i]>>8;
RspModbus[i+1]=Mapa[EndIni+j] & 0x00FF; 
  
}

CRC16(3+2*QtdReg,RspModbus);

RspModbus[3+2*QtdReg]=CRClow;
RspModbus[3+2*QtdReg+1]=CRChigh;

TX485;

Atraso(Mapa[ParDelayModbus]);

UCSR0B &= 0b11101111;  //Desativa Rx

for(int i=0;i<3+2*QtdReg+2;i++) 
TxEnvia(RspModbus[i]);

Atraso(5);

UCSR0B |= 0b00010000;  //Ativa Rx

RX485;

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void WriteSingleRegister(void){  

bool gravEsp=false;
  
EndIni=RxModbus[2];
EndIni<<=8;
EndIni+=RxModbus[3];   
 
ValueRegister=RxModbus[4];
ValueRegister<<=8;
ValueRegister+=RxModbus[5]; 


if(EndIni>ENDWRITEMAX)
gravEsp=true;

if(EndIni==ParPotDigVal)
{
  gravEsp=true;
  if(ValueRegister<1024)
  {
  Mapa[ParPotDigVal]=ValueRegister;
  GravaParam(ParPotDigVal, ValueRegister);
  }
}

if(EndIni==ParVmin)
{
  gravEsp=true;
  if(ValueRegister<513)
  {
  Mapa[ParVmin]=ValueRegister;
  GravaParam(ParVmin, ValueRegister);
  }
}

if(EndIni==ParVmax)
{
  gravEsp=true;
  if(ValueRegister<1024)
  {
  Mapa[ParVmax]=ValueRegister;
  GravaParam(ParVmax, ValueRegister);
  }
}



if(EndIni==ParDelayModbus)
{
  gravEsp=true;
  if(ValueRegister<100)
  {
  GravaParam(ParDelayModbus,ValueRegister);
  Mapa[ParDelayModbus]=ValueRegister;
  }
}

if(!gravEsp)
{
  GravaParam(EndIni,ValueRegister);
  Mapa[EndIni]=ValueRegister;
}

TX485;
Atraso(Mapa[ParDelayModbus]);

UCSR0B &= 0b11101111;  //Desativa Rx

for(int i=0;i<8;i++) 
TxEnvia(RxModbus[i]);//Resposta é a mesma do pedido

Atraso(5);

UCSR0B |= 0b00010000;  //Ativa Rx

RX485;
  
}

//%%%%%%%%%%%%%%%%%%%%%%% Grava e Le na EEPROM %%%%%%%%%%%%%%%%%%%%%%
void GravaParam(unsigned int End,unsigned int Par){

unsigned int ParH=0;
unsigned int ParL=0;

ParH = Par & 0xFF00;
ParH >>=8;
ParL = Par & 0x00FF;

EEPROM.write(2*End,ParH);
EEPROM.write(2*End+1,ParL);
  
}


unsigned int LeParam(unsigned int EndParam){
  

unsigned int Lido;

unsigned int ParH=0;
unsigned int ParL=0;

ParH=EEPROM.read(2*EndParam);
ParH<<=8;
ParL=EEPROM.read(2*EndParam+1);
  
Lido=ParH+ParL; 
 
 
 return Lido;  
  
}


void GravaMapaDefault(void){
  
  GravaParam(ParEndSensor,ENDPADRAO);
  GravaParam(ParEepromCheck,0xAAAA);
  GravaParam(ParDelayModbus,10);
  GravaParam(ParCalcVcarga,0);//Vmedia
  GravaParam(ParTriacSW,1);//Ligada
  GravaParam(ParManDig,0);//Manual
  GravaParam(ParPotDigVal,256);//0 a 512  
  GravaParam(ParOcFlagAutClear,1);//1 - Clear Auto 
  GravaParam(ParOCn,5);
  GravaParam(ParVmax,VELMAX);
  GravaParam(ParVmin,VELMIN);
  
  //Zera variaveis de leitura na EEPROM
  for(int j=ENDWRITEMAX+1;j<30;j++)
  GravaParam(j,0);
  
}

//########################################  Rotinas de controle de Potencia  

void isr_retpotHandle()
{
  if(Mapa[ParManDig])
  Mapa[ParPotDigVal]=1023;//Retorna na máxima velocidade
  
  Mapa[ParRetFlag]=1;
  flagRet=true;

  retFlagClear=0;//Inicia contagem para limpar flag em RETCLEARTEMPO
}

void isr_rotSensHandle()
{
  rotSensToggle^=1;

  if(rotSensToggle)
  timeStampMs=0;
  else
  {
    double rpmCalc=0;
    rotSensStamp=timeStampMs;
    rpmCalc=60000/rotSensStamp;
    Mapa[ParVelMotor]=(unsigned int)rpmCalc;
  }
}

void isr_ocprotHandle()
{
  ocPulse++;

  Mapa[ParOcPulse]=(unsigned int)ocPulse;

  if(ocPulse>Mapa[ParOCn])
  {ocFlag=true;Mapa[ParOcFlag]=1;}
  

}

void isr_syncHandle()
{
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SYNC));

  Mapa[ParPulsoRede]=1;
  
  nPulse++;

  if(nPulse==120)
  {
    digitalWrite(LED,HIGH);

    if(ocFlag)
    digitalWrite(BEEP,HIGH);

    syncON=true;
    
  }

  if(nPulse>130)
  {
    nPulse=0;
    digitalWrite(LED,LOW);

    if(ocFlag)
    digitalWrite(BEEP,LOW);
    
    if(Mapa[ParOcFlagAutClear]==1)
    {
      ocPulse=0;
      ocFlag=false;
      Mapa[ParOcFlag]=0;
    }
  }
  
  

 
  TCCR0B = 0;

  TIMSK0 |= 0b00000110; //Habilita interrupções CMPA e CMPB
  TIFR0  |= 0b00000111;
  
  TCNT0 = 0;

  TCCR0B |= 0b00000101;

  
}

ISR(TIMER0_COMPA_vect)
{

  bool potAlto=false;
  
  if(Mapa[ParManDig]&&Mapa[ParPotVal]>20&&!flagRet)
  {
  digitalWrite(MOTOR,0);
  }
  else
  {

    if(!ocFlag&&Mapa[ParTriacSW]==1)
    digitalWrite(MOTOR,1);

  }

    
  

}

ISR(TIMER0_COMPB_vect)
{
  digitalWrite(MOTOR,0);
  Mapa[ParPulsoRede]=0;

  TIMSK0 &= 0b11111001; //desabilita interrupção CMPA e CMPB
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SYNC), isr_syncHandle, FALLING);

}

//###########################################  Rotina de atualização conforme posição do POT #########################
void fcsUpdate()
{
  if(!digitalRead(FC1))
  Mapa[ParSensorFC1]=1;
  else
  Mapa[ParSensorFC1]=0;

  if(!digitalRead(FC2))
  Mapa[ParSensorFC2]=1;
  else
  Mapa[ParSensorFC2]=0;
}

double potUpdate()
{

  double pot=0;
  double potA=0;
  double potD=0;
  double aux=0;
  double alpha=0;
  unsigned int td=0;
  double vmaxcalc=0;
  double vmincalc=0;
  double vout=0;

  byte compVal=0; 


  //Calcula o percentual da posição do potenciômetro
    aux=analogRead(0);
    potA=aux-POTVMIN;
    aux=POTVMAX-POTVMIN;
    potA/=aux;
    aux=potA*100;
    Mapa[ParPotVal]=(unsigned int)aux;

  //Calcula percentual do pot digital
    if(Mapa[ParPotDigVal]>1023)
    Mapa[ParPotDigVal]=1023;

    potD=(double)Mapa[ParPotDigVal];
    potD/=1023;

  //Controle de potência por potenciômetro ou via digital (ModBus)
  if(!Mapa[ParManDig])
  {
    pot=potA;
  }
  else
  {
    pot=potD;
  }
 
  
  //Calcula o relativo para Vmax e Vmin definidos
  vmaxcalc=(double)Mapa[ParVmax];
  vmincalc=(double)Mapa[ParVmin];
  vout=(vmaxcalc-vmincalc)*pot + vmincalc;
  vout/=1023;

  pot=vout;
  
  //Corrige angulo conforme Vrms ou Vmedia
  if(Mapa[ParCalcVcarga]==0)
  alpha=findAlpha(pot, false);//true = Vrms, false = Vmedia
  else
  alpha=findAlpha(pot, true);//true = Vrms, false = Vmedia

  td = disparoCalc(alpha);
  
  //Calcula em funcao do tempo de disparo
  pot=(double)td;
  pot/=TMEIAONDA;
  //Fim da correção
  
  aux=pot*TOPCOUNT;
  
  //compVal=TOPCOUNT- (byte)aux;//Invertido
  compVal=(byte)aux;

  if(compVal==0)
  compVal=2;
  
  OCR0A = compVal;
  OCR0B = compVal+WP;

  return pot;

}
//##################################### Rotinas Calculo RMS e MEDIA
double vrmsCalc(double alpha)
{

  double vrms=0;
  

  double rad = (alpha/180)*PI;

  double aux1=(PI-rad)/(2*PI);
  double aux2=sin(2*rad)/(4*PI);

  vrms=sqrt(aux1+aux2);

  return vrms;

}

double vmediaCalc(double alpha)
{

  double rad=(alpha/180)*PI;

  return (1+cos(rad))/PI;
}


double findAlpha(double rate,bool type)
{

  //type true = Vrms
  //type false = vmedia

  double vIn=0;
  
  if(type)
  vIn=VRMSMAX*rate;
  else
  vIn=VMEDIAMAX*rate;

  double alpha=0;
  double aux=0;

  for(alpha=0;alpha<180;alpha+=1)
  {
   
   if(type==true)
   {
     aux=vrmsCalc(alpha);
      
    if(aux<=vIn)
    break;

   }
   else
   {
     aux=vmediaCalc(alpha);

      if(aux<=vIn)
      break;
   }
   

  }

  return alpha;
}

unsigned int disparoCalc(double alpha)
{
  double aux = 0;
  aux=  (alpha/180)*TMEIAONDA;
  return (unsigned int)aux;
}

void beep(int n)
{
  int i=0;
  long j=0;

  for(i=0;i<n;i++)
  {
    digitalWrite(BEEP,HIGH);
    for(j=0;j<100000;j++){asm volatile("nop");}
    digitalWrite(BEEP,LOW);
    for(j=0;j<200000;j++){asm volatile("nop");}
  }

}




