HardwareTimer timer(3);

#include <Wire_slave.h>

/////
int address = 16;   //change address here 15,16,17
/////

#define CSr PA2
#define CSl PA3
#define slpr PA4
#define slpl PA5
#define DIRr PA6
#define DIRl PA7
#define PWMr PB0
#define PWMl PB1


int vel1 ,vel2 , om1 ,om2 = 0, hb =0;
int pwmr = 0, pwml = 0;

void receiveEvent(int howMany)
{
  if ( Wire.available())
  {
    vel1 = Wire.read();
    vel2 = Wire.read();
    om1 = Wire.read();
    om2 = Wire.read();
    hb = Wire.read();
    
    vel = vel1 + (vel2<<8);
    omega = om1 + (om2<<8);
    
    if(vel>4095){
      vel = vel-4095;
    }else{
      vel = -vel;
    }
    
    if(omega>4095){
      omega = omega-4095;
    }else{
      omega = -omega;
    }

  }
}

void setup()
{
timer.setPeriod(50);

  pinMode(PWMl, PWM);
  pinMode(PWMr, PWM);
  pinMode(slpl, OUTPUT);
  pinMode(slpr, OUTPUT);
  pinMode(DIRr, OUTPUT);
  pinMode(DIRl, OUTPUT);
  pinMode(CSr, INPUT);
  pinMode(CSl, INPUT);

  digitalWrite(slpl, HIGH);
  digitalWrite(slpr, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin(address);

  Wire.onReceive(receiveEvent);

 pwmWrite(PWMl, 0);
  pwmWrite(PWMr,0);
  digitalWrite(slpl, LOW);
  digitalWrite(slpr,LOW);
  digitalWrite(DIRr, LOW);
  digitalWrite(DIRl, LOW);

}

void loop()
{

  pwmr = int(constrain((vel + omega), -4095, 4095));
  pwml = int(constrain((vel - omega), -4095, 4095));

  if (hb==0)
  {
	  if (pwmr > 0)
	  {
	    digitalWrite(slpr, HIGH);
	    digitalWrite(DIRr, HIGH);
	    pwmWrite(PWMr, abs(pwmr));
	  }
	  else if (pwmr < 0)
	  {
	    digitalWrite(slpr, HIGH);
	    digitalWrite(DIRr, LOW);
	    pwmWrite(PWMr, abs(pwmr));
	  }
	  else
	  {
	    digitalWrite(slpr, LOW);
	    pwmWrite(PWMr, 0);
	  }

	  if (pwml > 0)
	  {
	    digitalWrite(slpl, HIGH);
	    digitalWrite(DIRl, LOW);
	    pwmWrite(PWMl, abs(pwml));
	  }
	  else if (pwml < 0)
	  {
	    digitalWrite(slpl, HIGH);
	    digitalWrite(DIRl, HIGH);
	    pwmWrite(PWMl, abs(pwml));
	  }
	  else
	  {
	    digitalWrite(slpl, LOW);
	    pwmWrite(PWMl, 0);
	  }
  }
  else
  {
  	digitalWrite(slpr, LOW);
	pwmWrite(PWMr, 0);
	digitalWrite(slpl, LOW);
	pwmWrite(PWMl, 0);
  }

}