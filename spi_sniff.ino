#include <SPI.h>
#include <Wire.h>

#define DEBUG true

#ifdef DEBUG
  #define DEADLOCKED_LED 2
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

#ifdef DEBUG
  #define ALPHA_LED 3
#endif



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
volatile int our_log_head, our_log_tail, their_log_head, their_log_tail;

// When a byte comes in over SPI, we check to see if there's room in our
// ring buffer log for it. If so, we throw it in there and increment the count
// of unsynchronized bytes. If there's not room, we just drop it on the floor
// and if in debug mode we light up the SLOWSYNC_LED.

volatile int bytes_unsynched, bytes_unreported;
#ifdef DEBUG
  #define SLOWSYNC_LED 4
#endif

// SPI Interrupt Handler
ISR (SPI_STC_vect) {
  if (our_log_head == (our_log_tail + 1) % LOG_SIZE) {
#ifdef DEBUG
    digitalWrite(SLOWSYNC_LED, HIGH);
#endif
    return;
  }

  our_log[our_log_tail++] = SPDR;
  our_log_tail %= LOG_SIZE;
  bytes_unsynched++;
}

// The synchronization process is pretty straightforward. The node with id alpha
// acts as the I2C master, and first requests all the unsynchronized bytes from
// the beta node, then transmits all its unsynchronized bytes back.

// returns number of bytes synchronized
int alpha_synchronize(void) {
  // bail if we're not the alpha node
  if (our_id != alpha) return 0;

  // bail if we're already synched up
  if (bytes_unsynched == 0) return 0;

  int bytes_synched = 0;
  Wire.requestFrom(I2C_SLAVE_ADDRESS, bytes_unsynched);

  // potential for deadlock here, if the two nodes receive different amounts of
  // bytes over SPI, which shouldn't happen unless one of them gets disconnected
  // from the SPI bus
#ifdef DEBUG
  digitalWrite(DEADLOCKED_LED, HIGH);
#endif
  while (Wire.available() < bytes_unsynched) {}

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
    bytes_synched++;
  }

#ifdef DEBUG
  digitalWrite(DEADLOCKED_LED, LOW);
#endif

  // send our bytes
  int uhead = unsynched_head(bytes_unsynched, our_log_tail, LOG_SIZE);
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  for (int i = 0; i < bytes_unsynched; i++) {
    Wire.write(our_log[(uhead + i) % LOG_SIZE]);
  }
  Wire.endTransmission();
#ifdef DEBUG
  if (bytes_unsynched != bytes_synched) {
    digitalWrite(UNSYNCHRONIZED_LED, HIGH);
  }
#endif

  bytes_unsynched -= bytes_synched;
  bytes_unreported += bytes_synched;

  return bytes_synched;
}

// The beta node acts as the I2C slave. First it must handle the master's
// request for all of its unsynchronized bytes.

void beta_synchronize_tx(void) {
  int uhead = unsynched_head(bytes_unsynched, our_log_tail, LOG_SIZE);
  for (int i = 0; i < bytes_unsynched, i++) {
    Wire.write(our_log[(uhead + i) % LOG_SIZE]);
  }
}

// Then it must receive all of the master's unsynchronized bytes.

void beta_synchronize_rx(int bytes_synched) {
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
  if (bytes_unsynched != bytes_synched) {
    digitalWrite(UNSYNCHRONIZED_LED, HIGH);
  }
#endif

  bytes_unsynched -= bytes_synched;
  bytes_unreported += bytes_synched;
}

int unsynched_head(int unsynched, int log_tail, int log_size) {
  // log_tail is where next byte will end up, hence the -1. The '+ 2*log_size'
  // before the modulus ensures that 0 <= start < log_size.
  return ((log_tail - unsynched - 1) + 2*log_size) % log_size;
}

int unreported_head(int unreported, int unsynched, int log_tail, int log_size) {
  int uhead = unsynched_head(unsynched, log_tail, log_size);
  return ((uhead - unreported) + log_size) % log_size;
}



// Setup
// =====
//
// Alright, now we've got to string everything together. In setup, we read our
// id from the ID_SELECT pin, and then initialize the I2C bus and attach
// interrupts accordingly. We also do the usual variable initialization and UART
// (Serial), SPI, and pin direction configuration.

void setup(void) {
  pos = 0;
  buffer_full = false;
  flushes = 0;

  Serial.begin(115200);

  pinMode(MISO, INPUT); // both nodes are SPI slaves
  pinMode(ID_SELECT, INPUT);

  our_id = (digitalRead(ID_SELECT) == HIGH) ? alpha : beta;
#ifdef DEBUG
  if (our_id == alpha) {
    digitalWrite(ALPHA_LED, HIGH);
  }
#endif

  // Synchronization Setup
  if (our_id == alpha) {
    // alpha is the I2C master
    Wire.begin();
  } else {
    // beta is the I2C slave
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onRequest(beta_synchronize_tx);
    Wire.onReceive(beta_synchronize_rx);
  }

  // SPI Setup
  SPCR |= 1 << CPOL; // clock is idle high
  SPCR |= 1 << CPHA; // sample on trailing (i.e. rising) edge
  SPCR |= 1 << DORD; // data is transmitted in LSB order
  SPCR |= 1 << SPIE; // enable SPI interrupts
  SPCR |= 1 << SPE; // enable SPI
}

void loop(void) {
  // first, synchronize
  if (our_id == alpha) {
#ifdef DEBUG
    digitalWrite(SYNCHRONIZATION_LED, HIGH);
#endif
    int synched = alpha_synchronize();
#ifdef DEBUG
    digitalWrite(SYNCHRONIZATION_LED, LOW);
#endif
  }

  if (bytes_unreported > 0) {
    int start = unreported_head(bytes_unreported,
                                bytes_unsynched,
                                our_log_tail,
                                LOG_SIZE);
    int bytes_reported = 0;

    for (int i = 0; i < bytes_unreported; i++) {
      if (our_id == alpha) {
        Serial.print(our_log[(start + i) % LOG_SIZE], HEX);
        Serial.print("/");
        Serial.print(their_log[(start + i) % LOG_SIZE], HEX);
      } else {
        Serial.print(their_log[(start + i) % LOG_SIZE], HEX);
        Serial.print("/");
        Serial.print(our_log[(start + i) % LOG_SIZE], HEX);
      }
      bytes_reported += 1;
    }
  }
}
