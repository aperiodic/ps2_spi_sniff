#include <SPI.h>
#include <Wire.h>

#define DEBUG true

#ifdef DEBUG
  #define DEADLOCKED_LED 3
#endif


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
// [scanlime-notes]: https://gist.github.com/scanlime/5042071
//
// Every PS2 message begins with a fixed-length header, which includes the
// number of data packets that will follow (note that this determination is even
// a pure function of the message type and controller mode, so it can be
// obtained from the [controller state machine][csm-internal]). After that
// number of data packets are transmitted and acked, the PS2 terminates the
// message by driving the controller's CS line high.

enum ptcl_states {
  idle,
  header_0,
  header_1,
  header_2,
  data,
  pending_termination
};

volatile enum ptcl_states ptcl_state;



// PS2 Dual Shock Controller State Machine
// =======================================

enum ctlr_states {
  digital = 0x4,
  analog = 0x7, // a.k.a. dual-shock mode
  escape = 0xf
}

volatile enum ctlr_states ctlr_state;



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
// We have two logs for storing the bytes, one for each side of the SPI traffic.
// The bytes that we read go in our_log, and the bytes we get from the other
// node over I2C go in their_log. When the logs fill up, they will be dumped to
// UART after the next synchronization. If we get another SPI byte before we can
// synchronize, then we just drop all the data in the logs, and if we're in
// debug mode we'll light up an LED to indicate this.

#define LOG_SIZE 128
volatile byte* our_log[LOG_SIZE];
volatile byte* their_log[LOG_SIZE];
volatile byte bytes_unsynced;
volatile byte our_log_head, our_log_tail, their_log_head, their_log_tail;

// When a byte comes in over SPI, we check to see if there's room in our
// ring buffer log for it. If so, we throw it in there and increment the count
// of unsynchronized bytes. If there's not room, we just drop it on the floor
// and if in debug mode we light up the SLOWSYNC_LED

#ifdef DEBUG
  #define SLOWSYNC_LED 2
#endif

// SPI Interrupt Handler
ISR (SPI_STC_vect) {
  if (our_log_pos == LOG_SIZE) {
    our_log_pos = 0;
#ifdef DEBUG
    digitalWrite(SLOWSYNC_LED, HIGH);
#endif
  }
  our_log[our_log_pos++] = SPDR;
  bytes_unsynced++;
  if (bytes_unsynced > LOG_SIZE) {
    bytes_unsynced = LOG_SIZE;
  }
}

// The synchronization process is pretty straightforward. The node with id alpha
// acts as the I2C master, and first requests all the unsynchronized bytes from
// the beta node, then transmits all its unsynchronized bytes back.

void alpha_synchronize(void) {
  // bail if we're not the alpha node
  if (our_id != alpha) return;

  byte bytes_synced = 0;
  Wire.requestFrom(I2C_SLAVE_ADDRESS, bytes_unsynced);

  // potential for deadlock here, if the two nodes receive different amounts of
  // bytes over SPI, which shouldn't happen unless one of them gets disconnected
  // from the SPI bus
#ifdef DEBUG
  digitalWrite(DEADLOCKED_LED, HIGH);
#endif
  while (Wire.available() < bytes_unsynced) {}

  while (Wire.available() > 0) {
    // if there's room in the log chuck it in
    if (their_log_head != (their_log_tail + 1) % LOG_SIZE) {
      their_log[their_log_tail++] = Wire.read();
      their_log_tail %= LOG_SIZE;
    } else {
      // if the ring buffer is full we just drop the data on the floor
      Wire.read();
#ifdef DEBUG
      digitalWrite(SLOWDUMP_LED, HIGH);
#endif
    }
    bytes_synced++;
  }

#ifdef DEBUG
  digitalWrite(DEADLOCKED_LED, LOW);
#endif

  // send our bytes
  byte uhead = unsynced_head(bytes_unsynced, our_log_tail, LOG_SIZE);
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  for (byte i = 0; i < bytes_unsynced; i++) {
    Wire.write(our_log[(uhead + i) % LOG_SIZE]);
  }
  Wire.endTransmission();
#ifdef DEBUG
  if (bytes_unsynced != bytes_synced) {
    digitalWrite(UNSYNCHRONIZED_LED, HIGH);
  }
#endif
  bytes_unsynced -= bytes_synced;
}

// The beta node acts as the I2C slave. First it must handle the master's
// request for all of its unsynchronized bytes.

void beta_synchronize_tx(void) {
  byte uhead = unsynced_head(bytes_unsynced, our_log_tail, LOG_SIZE);
  for (byte i = 0; i < bytes_unsynced, i++) {
    Wire.write(our_log[(uhead + i) % LOG_SIZE]);
  }
}

// Then it must receive all of the master's unsynchronized bytes.

void beta_synchronize_rx(int bytes_synced) {
  while (Wire.available() > 0) {
    if (their_log_head != (their_log_tail + 1) % LOG_SIZE) {
      their_log[their_log_tail++] = Wire.read();
      their_log_tail %= LOG_SIZE;
    } else {
      Wire.read();
#ifdef DEBUG
      digitalWrite(SLOWDUMP_LED, HIGH);
#endif
    }
  }

#ifdef DEBUG
  if (bytes_unsynced != bytes_synced) {
    digitalWrite(UNSYNCHRONIZED_LED, HIGH);
  }
#endif
  bytes_unsynced -= bytes_synced;
}

byte unsynced_head(byte unsynced, byte log_pos, byte log_size) {
  // log_pos is where next byte will end up, hence the -1. The '+ 2*log_size
  // % log_size' ensures that 0 <= start < log_size.
  return ((log_pos - unsynced - 1) + 2*log_size) % log_size;
}



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
