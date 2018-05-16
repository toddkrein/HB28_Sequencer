//Simple Block Step Sequencer Demo
//For use on JamBox (HackerBox #0028)
//  audio output on external I2S PCM5102 module
//  each "row" plays a one of eight wavetables
//  buttons set/clear use of wavetable for each time step
//  mutiple waves can be combined in each step
//  fifth knob sets cycle speed
//  slow speed to more easily set buttons
//  raise speed to hear melody
//

#include <SPI.h>
#include "LedMatrix.h"
#include "driver/i2s.h"
#include "freertos/queue.h"

//LED Matrix Pins
#define NUMBER_OF_DEVICES 4 
#define CS_PIN 15
#define CLK_PIN 14
#define MISO_PIN 2 //Not Used
#define MOSI_PIN 12

#define fs  16000  //sample rate in Hz
#define pi2 6.288319
#define samplelen 100

#define sineEntries 64

LedMatrix ledMatrix = LedMatrix(NUMBER_OF_DEVICES, CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
byte gridstate[32];
int curcol=0;

static const i2s_port_t i2s_num = (i2s_port_t)0; // i2s port number

static const i2s_config_t i2s_config = {
     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
     .sample_rate = fs,
     .bits_per_sample = (i2s_bits_per_sample_t) 16,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  
     .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
     .intr_alloc_flags = 0,
     .dma_buf_count = 8,
     .dma_buf_len = 64 
};

static const i2s_pin_config_t pin_config = {
    .bck_io_num = 26,
    .ws_io_num = 25,
    .data_out_num = 22,
    .data_in_num = I2S_PIN_NO_CHANGE
};


int sineTable[sineEntries];
int waveFreq[] = {264, 297, 329, 352, 396, 440, 494, 555};

void setup() {
  Serial.begin(115200);
  Serial.println("In Setup");

  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);   //install and start i2s driver
  i2s_set_pin(i2s_num, &pin_config);
  i2s_set_sample_rates(i2s_num, fs); //set sample rates

  calcSines();

  ledMatrix.init();
  pinMode(4, INPUT_PULLDOWN);
  pinMode(5, INPUT_PULLDOWN);
  pinMode(16, INPUT_PULLDOWN);
  pinMode(17, INPUT_PULLDOWN);
  pinMode(18, INPUT_PULLDOWN);
  pinMode(19, INPUT_PULLDOWN);
  pinMode(21, INPUT_PULLDOWN);
  pinMode(23, INPUT_PULLDOWN);

  analogReadResolution(10);
  pinMode(32, INPUT);
  pinMode(33, INPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(36, INPUT);

  for (int i=0; i<32; i++)
    gridstate[i]=0;

  introscroll();
}

void loop() 
{
  
  byte btns;
  int pots[5];
  ledMatrix.clear();
  readinputs(&btns, pots);

  gridstate[curcol] ^= btns; 

  for (int i=0; i<32; i++)
  {
    if (i==curcol)
    {
      setcolmask(i,(gridstate[i] ^ 255));
    }
    else
    {
      setcolmask(i,gridstate[i]);
    }
  }

  //read range is 0-1023
  //map to:
  //slowest delay is 800ms/column, fastest delay is 60ms/column
  int tt = ((((1023-pots[4])*(800-60))/1023)+60);
  playwavevect(gridstate[curcol], tt, curcol);
  ledMatrix.commit();
  curcol++;
  if (curcol>=32) curcol=0;
}


// preload the sine table entries
void calcSines(void) {
  int i;

  for (i=0; i<sineEntries; i++) {
    sineTable[i] = (int) (sin((3.14159/2.0) * (float)i/(float)sineEntries) * 32000);
//    Serial.println(sineTable[i]);
  }
}

// return the sine table entry for the given step, where step is 0 -> 4*sineTable -1
//
int getSine(int step) {
  int value;

  int quad;

  step = step%(4*sineEntries);

  quad = step/sineEntries;

  if ((quad == 0) || (quad == 2))       // Going up
    value = sineTable[step % sineEntries];
  else
    value = sineTable[sineEntries - (step%sineEntries) -1];    // coming back down

  if ((quad == 2) || (quad == 3))     // bottom half of sine
    value = -value;

//  Serial.println(step);
//  Serial.println(value);
  return value;
}

void introscroll()
{
  ledMatrix.setText("Jam");
  for (int i=0; i<28; i++)
  {
    ledMatrix.clear();
    ledMatrix.scrollTextLeft();
    ledMatrix.drawText();
    ledMatrix.commit();
    delay(100);
  }
  delay(1500);
}

void readinputs(byte *buttons, int *potentiometers)
{
    *buttons = 0;
    if (digitalRead(4) == HIGH)  *buttons+=1;
    if (digitalRead(5) == HIGH)  *buttons+=2;
    if (digitalRead(16) == HIGH) *buttons+=4;
    if (digitalRead(17) == HIGH) *buttons+=8;
    if (digitalRead(18) == HIGH) *buttons+=16;
    if (digitalRead(19) == HIGH) *buttons+=32;
    if (digitalRead(21) == HIGH) *buttons+=64;
    if (digitalRead(23) == HIGH) *buttons+=128;
    
    potentiometers[0] = analogRead(32); 
    potentiometers[1] = analogRead(33); 
    potentiometers[2] = analogRead(34);
    potentiometers[3] = analogRead(35);
    potentiometers[4] = analogRead(36); 
}

//sets an LED column according to bitmask
void setcolmask(int col, char bits)
{
    for (int i=0; i<8; i++)
    {
       if (bits & (1<<i))
         ledMatrix.setPixel(col,i);
    }
}

//sets an LED column from the bottom up with value 0-8
void setcolbotval(int col, char val)
{
    for (int i=0; i<8; i++)
    {
       int cmp = i+1;
       if (val>=cmp)
         ledMatrix.setPixel(col,(7-i));
    }
}



//play combined waves in bitvector for apx t (ms) 
void playwavevect(char vect, int t, int column)
{
  int combinedwave[samplelen];      // the samples for this frame
  int weight=0;
  long sample;                      // sample at the current instant
  long j;                           // sample offset from start of slot
  long entry;                       // which sine table entry to use
  int samplesPerMS;
  int frameCount;                   // how many frames do we need to generate
  int currentFrame;

  for (int i=0; i<8; i++)           // count how many vectors are being added
  {
    if ( vect & (1<<i) )
      weight++;
  }

  // determine the number of frames (samplelen samples) that we need to play to fill 't' ms
  samplesPerMS = fs/1000;           // samples/second * seconds/ms
  frameCount = (samplesPerMS * t) / samplelen + 1;   // samples/ms * ms / samples/frame (+1 to run at fastest time)

  // go generate and play each frame
  for (currentFrame = 0; currentFrame < frameCount; currentFrame++) {
    for (int i=0; i<samplelen; i++)   // generate all the samples in this frame
    {
      if (weight == 0){
        combinedwave[i] = 0;
        continue;
      }
      sample = 0;
      j = 0;
//      j = column * t * samplesPerMS;                                   // try to avoid the click at the start of a column
      j += ((long)currentFrame * (long)samplelen) + (long)i;           // sample offset from start of slot, so we don't glitch at the start of each frame
      if (vect & 1) {
//       x = j/fs;            // sample count / sample count/sec = seconds
//       y = x * waveFreq[0]; // seconds * cycles/second = cycles
//       z = y * 4 * sineEntries; // cycles * entries/cycle = entry
         entry = (j * (long)waveFreq[0] * (long)4 * (long)sineEntries) / (long)fs;    
         sample += (long)getSine(entry);
      }
      if (vect & 2) {
         entry = (j * (long)waveFreq[1] * (long)4 * (long)sineEntries) / (long)fs;       
         sample += (long)getSine(entry);
      }
      if (vect & 4) {
         entry = (j * (long)waveFreq[2] * (long)4 * (long)sineEntries) / (long)fs;       
         sample += (long)getSine(entry);
      }
      if (vect & 8) {
         entry = (j * (long)waveFreq[3] * (long)4 * (long)sineEntries) / (long)fs;       
         sample += (long)getSine(entry);
      }
      if (vect & 16){
         entry = (j * (long)waveFreq[4] * (long)4 * (long)sineEntries) / (long)fs;       
         sample += (long)getSine(entry);
      }
      if (vect & 32){
         entry = (j * (long)waveFreq[5] * (long)4 * (long)sineEntries) / (long)fs;       
         sample += (long)getSine(entry);
      }
      if (vect & 64){
         entry = (j * (long)waveFreq[6] * (long)4 * (long)sineEntries) / (long)fs;       
         sample += (long)getSine(entry);
      }
      if (vect & 128){
         entry = (j * (long)waveFreq[7] * (long)4 * (long)sineEntries) / (long)fs;       
         sample += (long)getSine(entry);
      }
  
      combinedwave[i] = (int)(sample / (long)weight);
      combinedwave[i] |= (combinedwave[i] << 16);       // give the left channel some love.
    }
//    i2s_write_bytes((i2s_port_t)i2s_num, (char *) combinedwave, samplelen, 100);    // ?? what is 100?
    i2s_write_bytes((i2s_port_t)i2s_num, (char *) combinedwave, samplelen * sizeof(int), portMAX_DELAY);    // ?? what is 100?
  }
}




