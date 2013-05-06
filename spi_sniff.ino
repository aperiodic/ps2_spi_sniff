#include <SPI.h>
#include <Wire.h>

#define DEBUG true


// Identity Arbitration
// ====================
//
// The arduino with id alpha reads the MOSI side of the SPI traffic and
// coordinates synchronization over I2C. Id selection is done with a switch
// between ground and a pull-up resistor, fed into the ID_SELECT pin. Low means
// alpha, high means beta.  Behavior in the presence of multiple nodes with the
// same id is undefined.

enum id {
  alpha, beta
};
enum id our_id;
#define ID_SELECT 8



// Communication Pin Definitions
// =============================
//
// These are only for reference.
#define SPI_CS 10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_CLOCK 13

#define I2C_SDA A4
#define I2C_SDL A5
#define I2C_SLAVE_ADDRESS 42



// PS2 Dual Shock Protocol State Machine
// =====================================
//
// All this PS2-related stuff is mostly cribbed from the previously referenced
// [protocol notes] [scanlime-notes] by Scanlime. You should go read the first
// few paragraphs of that, up through "General packet format", and skim the
// rest.
//
// Every PS2 message begins with a fixed-length header, which includes the
// number of data packets that will follow (note that this determination is even
// a pure function of the message type and controller mode, so it can be
// obtained from the [controller state machine][csm-internal]). After that
// number of data packets are transmitted and acked, the PS2 terminates the
// message by driving the controller's CS line high.

enum ptcl_states {
  IDLE,
  HEADER_0,
  HEADER_1,
  HEADER_2,
  DATA,
  PENDING_TERMINATION
};

volatile enum ptcl_states ptcl_state;



// PS2 Dual Shock Controller State Machine
// =======================================



// Application-Level Protocol Details
//   or Junk I Made Up
//   or Here Be Some Bugs
//   ================================
//
// The problem here is that the Bus Pirate I have doesn't allow you to set the
// clock phase when sniffing SPI traffic (though it does when it's the master),
// so I'm going to string together two Arduinos that both read one side of the
// traffic, then communicate over an I2C bus to synchronize.
//
// As aspiring competent systems designers, we must ask ourselves what should
// happen should I2C communication fail. We will attempt synchronization every
// byte, but we'll keep a backlog of the last few bytes if we need them. Any
// further bytes are logged to UART (and the condition to an LED) if in debug
// mode (otherwise they are ignored).

volatile byte IC2_SLACK 3
volatile byte* ptcl_buff[IC2_SLACK];

#ifdef DEBUG
  volatile boolean had_i2c_backlog = false;
#endif

// After synchronization, the bytes are stored in a log buffer big enough to
// contain all the bytes in the longest possible PS2 to controller message,
// which is forty between the two sides according to [M. Elizabeth Scott's
// protocol documentation] [scanlime-notes]. In debug mode, after each message
// terminates, it is logged to UART.
// [scanlime-notes]: https://gist.github.com/scanlime/5042071

#define MAX_MSG_SIZE 40
volatile byte* msg_log[MAX_MSG_SIZE];
volatile int log_pos;
volatile boolean log_full;
int msgs_captured;


void setup(void) {
  Serial.begin(115200);

  pinMode(MISO, OUTPUT);
  pinMode(MASTER_SELECT, OUTPUT);

  pos = 0;
  buffer_full = false;
  flushes = 0;

  delay(1);
  are_master = (digitalRead(MASTER_SELECT) == HIGH);

  // I2C Setup
  if (are_master) {
    Wire.begin();
  } else {
    Wire.begin(I2C_SLAVE_ADDRESS);
  }

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
    if (are_master) {
      master_buff[pos++] = SPDR;
    } else {
      slave_buff[pos++] = SPDR;
    }
  } else {
    buffer_full = true;
  }
}

// I2C Master Received Interrupt
void got_slave_bytes(void) {
  byte count = Wire.available();
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
