/*
*********************************************************
* Author   : Marlon Passos
* E-mail   : marlonjbpassos@gmail.com
* Github   : https://github.com/MarlonPassos
* Simulator:
*********************************************************
* Microcontroller: ATmega328P
* Modelo: UNO R3
* Version: 0.1
* File: rb_control.ino
*
* Description:
*   Carro robótico, utilizando as bibliotecas 'Thread' e 'ThreadController'.
*   Utilizando o código do Braço Robótico IRA versão 0.2 como base.
*
*--------------------------------------------------------------------------------------
* ira_v02:
* https://github.com/MarlonPassos/ArduinoBracoRobotico/blob/master/ira_v02/ira_v02.ino
*
* Biblioteca SoftwareSerial:
* https://github.com/PaulStoffregen/SoftwareSerial/tree/master/examples
*
*  	Interface serial baseada em software, permite implementar um comunicação serial
* 	assíncrona no arduino, para que se comunique com outros dispositivos seriais. Permite
* 	criar uma conexão serial em qualquer um dos pinos digitais de entrad/saí­da.
*
* Biblioteca 'Thread'/'ThreadController':
* https://github.com/ivanseidel/ArduinoThread
*
* Notas Utilizadas no buzzer (pitches.h):
* https://gist.github.com/mikeputnam/2820675
*
*/

#include <SoftwareSerial.h>
#include "Thread.h"
#include "ThreadController.h"
#include "pitches.h"
//----------------------------Pinagem motor-------------------------------------
//Pinagem motor A para controle de rotação horário/anti-horário
#define IN1_A 4
#define IN2_A 5
//Pinagem motor B para controle de rotaÃ§Ã£o horÃ¡rio/anti-horÃ¡rio
#define IN3_B 7
#define IN4_B 8
//Define os pinos e velocidade dos motores, rotação máxima de 8 bits (255)
#define MOTOR_A_S 6
#define MOTOR_B_S 9
//----------------------------Conexão bluetooth---------------------------------
#define SERIAL_IN 2         //Entrada serial via software
#define SERIAL_OUT 3        //SaÃ­da serial via software
//----------------------------Define os pinos dos leds luzes dianteira----------
#define LED_D0 A0
#define LED_D1 A1
#define LED_D2 A2
//----------------------------Define os pinos das luzes trazeiras---------------
#define LED_T3 A3
#define LED_T4 A4
//----------------------------Sensor ultrassom----------------------------------
#define TRIG 10
#define ECHO 11
//----------------------------Define pino buzzina-------------------------------
#define BUZZER A5
//--------------------------Time_thread e delays--------------------------------
#define DELAY_RC 10            //Tempo de espera do motor
#define TIME_TH  10            //Tempo execução

byte speed_motor = 100;        //Controle a velocidade do motor PWM
byte states_led = 1;
//---------------------------Objetos Thread-------------------------------------
Thread th_rc_forward;          //Frente
Thread th_rc_back;             //Traz
Thread th_rc_right;            //Direita
Thread th_rc_left;             //Esquerda
Thread th_rc_stop;             //Para o motor
Thread th_rc_speaker;          //Liga Buzzer
Thread th_rc_no_speaker;       //Desliga Buzzer
Thread th_on_front_led;        //Liga o led dianteiro
Thread th_off_front_led;       //Desliga o led dianteiro
Thread th_on_back_led;         //Liga o led traseiro
Thread th_off_back_led;        //Desliga o led traseiro
Thread th_flashes_led;
Thread th_rc_explorer;         //Explorador
Thread th_speaker_explorador;

ThreadController rc_control;
//
//----------------------------RX/TX bluetooth-----------------------------------
SoftwareSerial bth(SERIAL_IN, SERIAL_OUT);
//----------------------------Protótipo das funções-----------------------------
void run_cmd(char data);      //Executa os comandos
void rc_forward();            //gira sentido horario
void rc_back();	              //gira sentido anti-horario
void rc_right();              //motores A horario/B anti-horario
void rc_left();               //motores A anti-horario/B horario
void rc_stop();
void speed_motor_current();   //Seta valor speed_motor quando alterado
void rc_speaker();            //Liga o buzzer
void rc_nospeaker();          //Desliga o buzzer
void on_front_led();
void off_front_led();
void on_back_led();
void off_back_led();
void flashes_led();
//--------------------------função explorador-----------------------------------
void rc_explorer();           //Explorador
float calcula_dist();
void trigPulse();
void speaker_explorer();
float dist_cm;
float dist_right;
float dist_left;
//Notas
int melody[]={NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
int noteDurations[]={4, 8, 8, 4, 4, 4, 4, 4};
//---------------------------Setup----------------------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.flush();
  bth.begin(9600);
  bth.flush();
  //Definindo os pinos como entrada e saÃ­sa bluetooth
  pinMode(SERIAL_IN, INPUT);
  pinMode(SERIAL_OUT, OUTPUT);
  //Definindo os pinos dos motores A B
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN3_B, OUTPUT);
  pinMode(IN3_B, OUTPUT);
  //Leds
  pinMode(LED_D0, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_T3, OUTPUT);
  pinMode(LED_T4, OUTPUT);
  //Buzzer (speaker)
  pinMode(BUZZER, OUTPUT);
  //--------explorador
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  digitalWrite(TRIG, LOW);
  //------------------------------------Atribuindo as funçõe Auxiliares aos objetos threas
  th_rc_forward.setInterval(TIME_TH);       //Objeto thread gira o motor em sentido horário
  th_rc_forward.onRun(rc_forward);
  th_rc_forward.enabled = false;

  th_rc_back.setInterval(TIME_TH);          //gira sentido anti-horario
  th_rc_back.onRun(rc_back);
  th_rc_back.enabled = false;

  th_rc_right.setInterval(TIME_TH);         //motores A horario/B anti-horario
  th_rc_right.onRun(rc_right);
  th_rc_right.enabled = false;

  th_rc_left.setInterval(TIME_TH);          //motores A anti-horario/B horario
  th_rc_left.onRun(rc_left);
  th_rc_left.enabled = false;

  th_rc_stop.setInterval(TIME_TH);          //para os motores
  th_rc_stop.onRun(rc_stop);
  th_rc_stop.enabled = false;

  th_rc_speaker.setInterval(TIME_TH);       //Buzzer
  th_rc_speaker.onRun(rc_speaker);
  th_rc_speaker.enabled = false;
  //th_rc_no_speaker.setInterval(TIME_TH);    //Buzzer
  th_rc_no_speaker.onRun(rc_no_speaker);
  th_rc_no_speaker.enabled = false;

  th_on_front_led.onRun(on_front_led);
  th_on_front_led.enabled = false;

  th_off_front_led.onRun(off_front_led);
  th_off_front_led.enabled = false;

  th_on_back_led.onRun(on_back_led);
  th_on_back_led.enabled = false;

  th_off_back_led.onRun(off_back_led);
  th_off_back_led.enabled = false;

  th_flashes_led.setInterval(TIME_TH+100);
  th_flashes_led.onRun(flashes_led);
  th_flashes_led.enabled = false;

  th_rc_explorer.setInterval(TIME_TH);
  th_rc_explorer.onRun(rc_explorer);
  th_rc_explorer.enabled = false;

  th_speaker_explorador.onRun(speaker_explorer);
  th_speaker_explorador.enabled = false;

  //Adiciona os objetos thread a função de controle thread
  rc_control.add(&th_rc_forward);
  rc_control.add(&th_rc_back);
  rc_control.add(&th_rc_right);
  rc_control.add(&th_rc_left);
  rc_control.add(&th_rc_stop);
  rc_control.add(&th_rc_speaker);
  rc_control.add(&th_rc_no_speaker);
  rc_control.add(&th_on_front_led);
  rc_control.add(&th_off_front_led);
  rc_control.add(&th_on_back_led);
  rc_control.add(&th_off_back_led);
  rc_control.add(&th_flashes_led);
  rc_control.add(&th_rc_explorer);
  rc_control.add(&th_speaker_explorador);
}
//----------------------------loop principal------------------------------------
void loop()
{
  if (bth.available())
  {
    byte inByte = bth.read();
    Serial.write((char) inByte);
    run_cmd((char)inByte);
  }
  rc_control.run();
  bth.flush();                            //Limpa buffer conexão bluetooth
}
//rc vai para tras
void run_cmd(char data)
{
  switch (data) {
    case 'B': th_rc_back.enabled = true; break;
    case 'F': th_rc_forward.enabled = true; break;
    case 'L':
    case 'G':
    case 'H':
              th_rc_right.enabled = true; break;
    case 'R':
    case 'I':
    case 'J':
              th_rc_left.enabled = true; break;
    //case 'S': th_rc_stop.enabled = true; break;
    case 'V': th_rc_speaker.enabled = true; break;
    case 'v': th_rc_no_speaker.enabled = true; break;
    case 'W': th_on_front_led.enabled = true; break;
    case 'w': th_off_front_led.enabled = true;break;
    case 'X':
              th_rc_explorer.enabled = true;
              th_flashes_led.enabled = true;
              th_speaker_explorador.enabled = true;
              break;
    case 'x':
              th_rc_explorer.enabled = false;
              th_flashes_led.enabled = false;
              break;
    //----------Velocidade dos mostores pwm------
    case '0': speed_motor = 100; break;
    case '1': speed_motor = 115; break;
    case '2': speed_motor = 130; break;
    case '3': speed_motor = 145; break;
    case '4': speed_motor = 160; break;
    case '5': speed_motor = 175; break;
    case '6': speed_motor = 190; break;
    case '7': speed_motor = 205; break;
    case '8': speed_motor = 220; break;
    case '9': speed_motor = 220; break;
    case 'q': speed_motor = 254; break;
    default : th_rc_stop.enabled = true;
  }
}
//Liga os leds dianteiro
void on_front_led()
{
  digitalWrite(LED_D0, HIGH);
  digitalWrite(LED_D1, HIGH);
  digitalWrite(LED_D2, HIGH);
  th_on_front_led.enabled = false;
}
//Desliga os leds dianteiro
void off_front_led()
{
  digitalWrite(LED_D0, LOW);
  digitalWrite(LED_D1, LOW);
  digitalWrite(LED_D2, LOW);
  th_off_front_led.enabled = false;
}
//Acende as luzes traseitras
void on_back_led()
{
  digitalWrite(LED_T3, HIGH);
  digitalWrite(LED_T4, HIGH);
  th_on_back_led.enabled = false;
}
//Apaga as luzes traseiras
void off_back_led()
{
  digitalWrite(LED_T3, LOW);
  digitalWrite(LED_T4, LOW);
  th_off_back_led.enabled = false;
}
//Pisca leds
void flashes_led()
{
  states_led = states_led == 1? 0: 1;
  if(states_led)
    on_back_led();     //Modificar para 'th_on_back_led.enabled = true'
  else
    off_back_led();    //Modificar para 'th_off_back_led.enabled = true'
}
//Buzzer ligado
void rc_speaker()
{
  tone(BUZZER, 1500);
  th_rc_speaker.enabled = false;
}
//Buzzer desligado
void rc_no_speaker()
{
  noTone(BUZZER);
  th_rc_no_speaker.enabled = false;
}
//Gira o motor no sentido horario
void rc_forward()
{
  //velocidade dos motores A/B
  speed_motor_current();
  //Gira os motores A/B no sentido anti-horÃ¡rio
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, HIGH);
  digitalWrite(IN3_B, LOW);
  digitalWrite(IN4_B, HIGH);
  th_rc_forward.enabled = false;
}
//Gira o motor no sentido anti-horario
void rc_back()
{
  //Velocidade dos motores A/B
  speed_motor_current();
  //Gira os motores A/B no sentido horÃ¡rio
  digitalWrite(IN1_A, HIGH);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN3_B, HIGH);
  digitalWrite(IN4_B, LOW);
  th_rc_back.enabled = false;
  //Acende os leds traseiros
  th_on_back_led.enabled = true;
}
//vira para esquerda
void rc_left()
{
  //velocidade dos motores A/B
  speed_motor_current();
  //Gira os motores A horÃ¡rio/B no sentido anti-horÃ¡rio
  digitalWrite(IN1_A, HIGH);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN3_B, LOW);
  digitalWrite(IN4_B, HIGH);
  //Acende o led esquerda
  digitalWrite(LED_T3, HIGH);
  th_rc_left.enabled = false;
  th_off_back_led.enabled = true;
}
//vira para direita
void rc_right()
{
  //velocidade dos motores A/B
  speed_motor_current();
  //Gira os motores A anti-horÃ¡rio/B no sentido horÃ¡rio
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, HIGH);
  digitalWrite(IN3_B, HIGH);
  digitalWrite(IN4_B, LOW);
  //Acende led direito
  digitalWrite(LED_T4, HIGH);
  th_rc_right.enabled = false;
  th_off_back_led.enabled = true;
}
//Para os motores A/B
void rc_stop()
{
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN3_B, LOW);
  digitalWrite(IN4_B, LOW);
  th_rc_stop.enabled = false;
  //Desliga led trazeiro
  th_off_back_led.enabled = true;
}
//Normaliza a velocidade dos motores A/B conforme PWM aplicado
void speed_motor_current()
{
  analogWrite(MOTOR_A_S,speed_motor);
  analogWrite(MOTOR_B_S,speed_motor);
}
//----------------------funções robo explorador---------------------------------
//buzzer_explorador...
void speaker_explorer()
{
  if (1){
    //Itera sobre as notas
    for (int thisNote=0; thisNote <8; thisNote++){
      //Calcula a duração das notas
      int noteDuration = 1000 / noteDurations [thisNote];
      tone(BUZZER, melody [thisNote], noteDuration);
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      //Desliga o buzzer
      noTone(BUZZER);
    }
    th_speaker_explorador.enabled = false;
  }
}
float calcula_dist()
{
  float pulse;
  trigPulse();
  pulse = pulseIn(ECHO, HIGH);
  return (pulse/58.82);
}
void trigPulse()
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
}
void rc_explorer()
{
  speed_motor = 80;
  rc_forward();                   //Movimenta-se para frente
  dist_cm = calcula_dist();       //Calcula a distância
  if(dist_cm < 25)                //Se menor que 20 cm
  {
    rc_stop();                    //Para os motores
    delay(10);                    //Espera 10 milissegundos
    rc_back();                    //Movimenta-se para tras
    delay(600);                   //Espera 1 segundo
    rc_stop();                    //Para os motores
    delay(10);                    //Espera 10 milissegundos
    rc_right();                   //Vira para esquerda
    delay(800);                   //Espera 200 milissegundos
  }
  delay(100);                     //Espera 100 milisegundos
}
