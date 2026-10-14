//Firmware para controle da placa de controle de velocidade LAT Ver00 em 09/10/2026
#define SYNC 6
#define OCPROT A1
#define LED 12
#define MOTOR 4
#define WP 5
#define VELMAX 1.0
#define VELMIN 0.2
#define TOPCOUNT 130
#define MINCOUNT 10
#define PI 3.1415926535897932384626433832795
#define VRMSMAX 0.70710678118654752440084436210485
#define VMEDIAMAX 0.63661990481259183242126921637183
#define TMEIAONDA 8333 //us
#define LEDON digitalWrite(LED,HIGH)
#define LEDOFF digitalWrite(LED,LOW)
#define MOTORON digitalWrite(MOTOR,HIGH)
#define MOTOROFF digitalWrite(MOTOR,LOW)

#include <PinChangeInterrupt.h>

bool syncON = false;
int nPulse=0;
int ocPulse=0;
bool ocFlag=false;


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

  pinMode(LED, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(SYNC,INPUT);
  pinMode(OCPROT,INPUT);

  digitalWrite(MOTOR,LOW);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SYNC), isr_syncHandle, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(OCPROT), isr_ocprotHandle, FALLING);
}

// the loop function runs over and over again forever
void loop() {


  if(syncON)
  {
    potUpdate();
    syncON=false;
  }


}

void isr_ocprotHandle()
{
  ocPulse++;

  if(ocPulse>5)
  ocFlag=true;
  

}

void isr_syncHandle()
{
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SYNC));
  
  nPulse++;

  if(nPulse==120)
  {
    digitalWrite(LED,HIGH);
    syncON=true;
    ocPulse=0;
    ocFlag=false;
  }

  if(nPulse>130)
  {
    nPulse=0;
    digitalWrite(LED,LOW);
  }
  
  

 
  TCCR0B = 0;

  TIMSK0 |= 0b00000110; //Habilita interrupções CMPA e CMPB
  TIFR0  |= 0b00000111;
  
  TCNT0 = 0;

  TCCR0B |= 0b00000101;

  
}

ISR(TIMER0_COMPA_vect)
{
  
  if(!ocFlag)
  digitalWrite(MOTOR,1);
  

}

ISR(TIMER0_COMPB_vect)
{
  digitalWrite(MOTOR,0);
  TIMSK0 &= 0b11111001; //desabilita interrupção CMPA e CMPB
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SYNC), isr_syncHandle, FALLING);

}

//###########################################  Rotina de atualização conforme posição do POT #########################
double potUpdate()
{

  double pot=0;
  double aux=0;
  double alpha=0;
  unsigned int td=0;

  byte compVal=0; 

  pot=(double)analogRead(0);
  pot/=1023;
  
  //Corrige angulo conforme Vrms ou Vmedia
  alpha=findAlpha(pot, false);//true = Vrms, false = Vmedia
  td = disparoCalc(alpha);

  pot=(double)td;
  pot/=TMEIAONDA;
  //Fim da correção
  
  aux=pot*TOPCOUNT;
  
  compVal=TOPCOUNT- (byte)aux;

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




