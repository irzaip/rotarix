/*

     >> Ini file akhir yang di pakai di FLIPIRQ
   
     >> Making Music with Arduino: https://nerdmusician.teachable.com/p/making-music-with-arduino  
     >> Curso Fazendo Música com Arduino em Português: https://www.musiconerd.com/curso-completo

     Por por Gustavo Silveira, 2020.
     - Esta Sketch lê as portas digitais e analógicas do Arduino e envia notas midi e MIDI Control Change

     Quer aprender a fazer seus próprios códigos e entender os códigos de outras pessoas?
     Confira nosso curso completo do Fazendo Música com Arduino: http://musiconerd.com/curso-completo
  
     http://www.musiconerd.com
     http://www.youtube.com/musiconerd
     http://facebook.com/musiconerdmusiconerd
     http://instagram.com/musiconerd/
     http://www.gustavosilveira.net
     gustavosilveira@musiconerd.com

*/


/////////////////////////////////////////////
// Escolhendo seu placa
// Defina seu placa, escolha:
// "ATMEGA328" se estiver usando o ATmega328 - Uno, Mega, Nano ...
// "ATMEGA32U4" se estiver usando com o ATmega32U4 - Micro, Pro Micro, Leonardo ...
// "TEENSY" se estiver usando uma placa Teensy
// "DEBUG" se você quer apenas debugar o código no monitor serial
// você não precisa comentar ou descomentar qualquer biblioteca MIDI abaixo depois de definir sua placa

#define ATMEGA32U4 1 //* coloque aqui o uC que você está usando, como nas linhas acima seguidas de "1", como "ATMEGA328 1", "DEBUG 1", etc.

/////////////////////////////////////////////
// BIBLIOTECAS
// -- Define a biblioteca MIDI -- //

//Mux control pins
int s0 = 6;
int s1 = 7;
int s2 = 8;
int s3 = 9;

//Mux in "SIG" pin
int SIG_pin = 2;


// se estiver usando com ATmega328 - Uno, Mega, Nano ...
#ifdef ATMEGA328
#include <MIDI.h>
MIDI_CREATE_DEFAULT_INSTANCE();

// se estiver usando com ATmega32U4 - Micro, Pro Micro, Leonardo ...
#elif ATMEGA32U4
#include "MIDIUSB.h"

#endif
// ---- //

/////////////////////////////////////////////
// BOTOES
const int N_BUTTONS = 1; //*  número total de botões
const int BUTTON_ARDUINO_PIN[N_BUTTONS] = {10}; //* pinos de cada botão conectado diretamente ao Arduino

//#define pin13 1 // descomente se você estiver usando o pino 13 (o pino com led), ou comente a linha se não
byte pin13index = 12; //* coloque o índice do pin 13 do array buttonPin[] se você estiver usando, se não, comente

int buttonCState[N_BUTTONS] = {};        // armazena o valor atual do botão
int buttonPState[N_BUTTONS] = {};        // armazena o valor anterior do botão

// debounce
unsigned long lastDebounceTime[N_BUTTONS] = {0};  // a última vez que o pino de saída foi alternado
unsigned long debounceDelay = 20;    //* o tempo de debounce; aumentar se a saída estiver mandando muitas notas de uma vez so

/////////////////////////////////////////////
// POTENCIOMETROS
const int N_POTS = 16; //* número total de pots (slide e rotativo)
const int POT_ARDUINO_PIN[N_POTS] = {A5, A4, A3, A2, A1, A0}; //* pinos de cada pot conectado diretamente ao Arduino

int potCState[N_POTS] = {0}; // estado atual da porta analogica
int potPState[N_POTS] = {0}; // estado previo da porta analogica
int potVar = 0; // variacao entre o valor do estado previo e o atual da porta analogica

int midiCState[N_POTS] = {0}; // Estado atual do valor midi
int midiPState[N_POTS] = {0}; // Estado anterior do valor midi

const int TIMEOUT = 300; //* quantidade de tempo em que o potenciometro sera lido apos ultrapassar o varThreshold
const int varThreshold = 10; //* threshold para a variacao no sinal do potenciometro
boolean potMoving = true; // se o potenciometro esta se movendo
unsigned long PTime[N_POTS] = {0}; // tempo armazenado anteriormente
unsigned long timer[N_POTS] = {0}; // armazena o tempo que passou desde que o timer foi zerado

/////////////////////////////////////////////
// midi
byte midiCh = 2; //* Canal MIDI a ser usado
byte note = 36; //* nota mais baixa a ser usada
byte cc = 11; //* O mais baixo MIDI CC a ser usado

/////////////////////////////////////////////
// SETUP
void setup() {

  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  
  // Baud Rate
  // use se estiver usando with ATmega328 (uno, mega, nano...)
  // 31250 para MIDI class compliant | 115200 para Hairless MIDI
  Serial.begin(31250); //*

#ifdef DEBUG
Serial.println("Debug mode");
Serial.println();
#endif

  // Buttons
  // Inicializa botões com pull up resistor
  for (int i = 0; i < N_BUTTONS; i++) {
    pinMode(BUTTON_ARDUINO_PIN[i], INPUT_PULLUP);
  }

#ifdef pin13 // inicializa o pino 13 como uma entrada
pinMode(BUTTON_ARDUINO_PIN[pin13index], INPUT);
#endif


}

/////////////////////////////////////////////
// LOOP
void loop() {

  buttons();
  potentiometers();

}

/////////////////////////////////////////////
// BOTOES
void buttons() {

  for (int i = 0; i < N_BUTTONS; i++) {

    buttonCState[i] = digitalRead(BUTTON_ARDUINO_PIN[i]);   // lê os pinos do arduino

#ifdef pin13
if (i == pin13index) {
buttonCState[i] = !buttonCState[i]; // inverte o pino 13 porque tem um resistor pull down em vez de um pull up
}
#endif

    if ((millis() - lastDebounceTime[i]) > debounceDelay) {

      if (buttonPState[i] != buttonCState[i]) {
        lastDebounceTime[i] = millis();

        if (buttonCState[i] == HIGH) {

          // Envia a nota MIDI de acordo com a placa escolhida
#ifdef ATMEGA328
// ATmega328 (uno, mega, nano...)
 MIDI.sendNoteOn(note + i, 127, midiCh); // note, velocity, channel
//MIDI.sendNoteOff(note + i, 0, midiCh);

#elif ATMEGA32U4
// ATmega32U4 (micro, pro micro, leonardo...)
noteOn(midiCh, note + i, 127);  // channel, note, velocity
// noteOff(midiCh,note + i, 0);
MidiUSB.flush();

#elif TEENSY
// Teensy
usbMIDI.sendNoteOn(note + i, 127, midiCh); // note, velocity, channel
// usbMIDI.sendNoteOff(note + i, 0, midiCh);

#elif DEBUG
Serial.print(i);
Serial.println(": button on");
#endif

        }
        else {

          // Envia a nota MIDI OFF de acordo com a placa escolhida
#ifdef ATMEGA328
// ATmega328 (uno, mega, nano...)
// MIDI.sendNoteOn(note + i, 0, midiCh); // note, velocity, channel
MIDI.sendNoteOff(note + i, 0, midiCh);

#elif ATMEGA32U4
// ATmega32U4 (micro, pro micro, leonardo...)
// noteOn(midiCh, note + i, 0);  // channel, note, velocity
noteOff(midiCh,note + i, 0);
MidiUSB.flush();

#elif TEENSY
// Teensy
// usbMIDI.sendNoteOn(note + i, 0, midiCh); // note, velocity, channel
usbMIDI.sendNoteOff(note + i, 0, midiCh);

#elif DEBUG
Serial.print(i);
Serial.println(": button off");
#endif

        }
        buttonPState[i] = buttonCState[i];
      }
    }
  }
}


int readMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  //loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  int val = analogRead(SIG_pin);

  //return the value
  return val;
}
/////////////////////////////////////////////
// POTENTIOMETERS
void potentiometers() {

  /* para que seja feita apenas a leitura das portas analogicas quando elas sao utilizadas, sem perder resolucao,
    ´e  preciso estabelecer um "threshold" (varThreshold),  um valor minimo que as portas tenham que ser movimentadas
    para que se comece a leitura. Apos isso cria-se uma especie de "portao", um portao que se abre e permite
    que as porta analogicas sejam lidas sem interrupcao por determinado tempo (TIMEOUT). Quando o timer ´e menor que TIMEOUT
    significa que o potenciometro foi mexido ha muito pouco tempo, o que significa que ele provavelmente ainda esta se movendo,
    logo deve-se manter o "portao" aberto; caso o timer seja maior que TIMEOUT siginifica que ja faz um tempo que ele nao ´e movimentado,
    logo o portao deve ser fechado. Para que essa logica aconteca deve-se zerar o timer (linhas 99 e 100) a cada vez que a porta analogica
    variar mais que o varThreshold estabelecido.
  */


  //Debug somente
  //    for (int i = 0; i < nPots; i++) {
  //      Serial.print(potCState[i]); Serial.print(" ");
  //    }
  //    Serial.println();

  for (int i = 0; i < N_POTS; i++) { // Faz o loop de todos os potenciômetros

    // potCState[i] = analogRead(POT_ARDUINO_PIN[i]);
   potCState[i] = readMux(i);
   
    midiCState[i] = map(potCState[i], 0, 1023, 0, 127); // Mapeia a leitura do potCState para um valor utilizável em midi

    potVar = abs(potCState[i] - potPState[i]); // Calcula o valor absoluto entre a diferença entre o estado atual e o anterior do pot

    if (potVar > varThreshold) { // Abre o portão se a variação do potenciômetro for maior que o limite (varThreshold)
      PTime[i] = millis(); // Armazena o tempo anterior
    }

    timer[i] = millis() - PTime[i]; // Reseta o timer 11000 - 11000 = 0ms

    if (timer[i] < TIMEOUT) { // Se o timer for menor que o tempo máximo permitido, significa que o potenciômetro ainda está se movendo
      potMoving = true;
    }
    else {
      potMoving = false;
    }

    if (potMoving == true) { // Se o potenciômetro ainda estiver em movimento, envie control change
      if (midiPState[i] != midiCState[i]) {

        // Envia o MIDI CC de acordo com a placa escolhida
#ifdef ATMEGA328
// ATmega328 (uno, mega, nano...)
MIDI.sendControlChange(cc + i, midiCState[i], midiCh); // cc number, cc value, midi channel

#elif ATMEGA32U4
// ATmega32U4 (micro, pro micro, leonardo...)
controlChange(midiCh, cc + i, midiCState[i]); //  (channel, CC number,  CC value)
MidiUSB.flush();

#elif TEENSY
// Teensy
usbMIDI.sendControlChange(cc + i, midiCState[i], midiCh); // cc number, cc value, midi channel

#elif DEBUG
Serial.print("Pot: ");
Serial.print(i);
Serial.print(" ");
Serial.println(midiCState[i]);
//Serial.print("  ");
#endif

        potPState[i] = potCState[i]; // Armazena a leitura atual do potenciômetro para comparar com a próxima
        midiPState[i] = midiCState[i];
      }
    }
  }
}

/////////////////////////////////////////////
// se estiver usando com ATmega32U4 (micro, pro micro, leonardo ...)
#ifdef ATMEGA32U4

// Arduino (pro)micro midi functions MIDIUSB Library
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

#endif
