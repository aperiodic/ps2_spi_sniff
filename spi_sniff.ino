#include <SPI.h>

static int buff_size = 512;
volatile byte buff[512];
volatile int pos;
volatile boolean buffer_full;
int flushes;

void setup(void) {
  Serial.begin(115200);

  pinMode(MISO, OUTPUT);

  pos = 0;
  buffer_full = false;
  flushes = 0;

  // SPI Setup
  SPCR |= 1 << CPOL; // clock is idle high
  SPCR |= 1 << CPHA; // sample on trailing (i.e. rising) edge
  SPCR |= 1 << DORD; // data is transmitted in LSB order
  SPCR |= 1 << SPIE; // enable SPI interrupts
  SPCR |= 1 << SPE; // enable SPI
}

// SPI Interrupt Handler
ISR (SPI_STC_vect) {
  if (pos < buff_size) {
    buff[pos++] = SPDR;
  } else {
    buffer_full = true;
  }
}

void loop(void) {
  if (buffer_full) {
    SPCR &= ~(1 << SPIE); // disable SPI interrupts
    for (int i = 0; i < buff_size; i++) {
      Serial.println(buff[i], HEX);
    }
    Serial.print("buffer flush ");
    Serial.println(flushes++, DEC);
    pos = 0;
    buffer_full = false;
    SPCR |= 1 << SPIE; // enable SPI interrupts
  }
}
