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
#define SPI_KS 10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_KLOCK 13

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


// Event System
// ============
//
// We'll be using an event-oriented programming paradigm where handlers are
// registered for I/O events such as bytes coming in over SPI or I2C. The
// handlers will be run in the main application thread and never in interrupt
// handlers. The upside of this is that all of the application's handlers will
// be atomic with respect to each other, which greatly simplifies the code. Only
// the event system itself has to be concurrent.
//
// The event system will maintain two ring buffers for each I/O channel that it
// supports, one for bytes coming in and another for those going out. For each
// such buffer, we must also maintain counts of the number of successful read
// and write operations on the buffer. The counts will allow us to calculate the
// amount of room left in the buffer, and will be transformed into array indices
// corresponding to the buffer's current head and tail when actually reading
// from and writing to the array. In order to keep the transformation from
// counts to indices as efficient as possible, the sizes of the buffers will be
// restricted to powers of two.
//
// Since we will be reading from and writing to these buffers inside of
// interrupt handlers, we must take care to implement these operations such that
// the buffers can be concurrently written to and read from. Fortunately, each buffer
// has only one producer and one consumer, which is either the application's
// handlers or the event system's interrupt handlers, depending on the direction
// of the buffer. This condition, referred to as the "1P1C guarantee", means
// that each producer and consumer effectively has a mutex lock on either the
// write count or the read count of the buffer, respectively, since there are no
// other producers or consumers that might mutate them, and there is no need for
// the producer to mutate the read count nor for the consumer to mutate the
// write count (they need only observe them). This condition allows us to
// enqueue onto and dequeue from the buffers without using locks or an atomic
// compare-and-swap operation.

bool buff_write(byte item, byte* buff, int capacity,
                unsigned int* reads, unsigned int* writes) {
  // First we must check that there is room left in the buffer, that is, that
  // the buffer is not full. The number of items in the buffer can be
  // calculated using the difference between the numbers of reads and writes on
  // the buffer. We can't always just take the difference, though, since it's
  // possible that the count of writes has overflown the unsigned int type and
  // wrapped around, making the difference (writes - reads) negative. If this is
  // the case, then we subtract the difference difference from the maximum
  // possible unsigned integer value in order to find the count of items in the
  // buffer (a proof of this procedure can be obtained by drawing the following
  // diagram:
  //
  //                  diff         items
  //         _________|_____    ___|__
  //        /               \ /       \
  //       |                 |         |
  //       writes            reads     writes'
  //       |                 |         | (where writes' % UINT_MAX = writes)
  //       |                 |         |
  //  |----|-----------------'----|----|----------------------|
  //  |    |                      |    |                      |
  //  |    '------------;---------|----'                      |
  //  |                 |         |                           |
  //  0                 |         UINT_MAX                    2 * UINT_MAX
  //                    |
  //                    UINT_MAX
  //
  //  and then observing that since diff + items = UINT_MAX, it follows that
  //  UINT_MAX - diff = items).

  unsigned int items;
  int diff = *writes - *reads;
  if (diff < 0) {
    items = UINT_MAX - diff;
  } else {
    items = diff;
  }

  // If the number of items is equal to the capacity of the buffer, then it's
  // full and we return false to indicate that the write failed.

  if (items == capacity) return false;

  // If we haven't bailed, we know that the buffer wasn't full sometime since we
  // started executing this function. Due to the 1P1C guarantee, we know that
  // nobody else can be attempting to write more things in to the buffer,
  // meaning the buffer can't be any more full than when this function started,
  // can't get any more full for the rest of the function, and we effectively have
  // a mutex lock on the *writes argument, since the consumer never modifies it.

  // Therefore, we can now calculate the current tail from the number of writes,
  // insert the item there, and then increment the number of writes. Note the
  // use of the assumption that capacity is a power of two here, in order to be
  // able to use the fast bitwise modulo trick.

  unsigned int tail = *writes & (capacity - 1);
  buff[tail] = item;
  *writes = *writes + 1;

  // Return true to indicate that the write succeeded.
  return true;
}

bool buff_read(byte* item, byte* buff, int capacity,
               unsigned int* reads, unsigned int* writes) {
  // For reading from the buffer, we just need to ensure that the buffer is
  // nonempty. This is a bit easier to check than the nonfull condition needed
  // for writing, since the buffer is only empty if there have been exactly the
  // same number of read and write operations on it.

  if (*writes == *reads) return false;

  // By the same argument we used in buff_write above, we know that if we're at
  // this point then the queue was nonempty, won't get any emptier, and we can
  // mutate the read count without conflict.

  unsigned int head = *reads & (capacity - 1);
  *item = buff[head];
  *reads = *reads + 1;

  return true;
}

// As for the specific buffers, we'll have two for SPI and two for I2C. The SPI
// buffers will be a bit larger (32 bytes, enough to hold an entire side of the
// longest PS2 <-> Controller message).

#define SPI_BUFFER_K 5
#define SPI_BUFFER_SIZE (1 << SPI_BUFFER_K)
volatile byte* spi_in_buff[SPI_BUFFER_SIZE];
volatile unsigned int spi_in_buff_reads, spi_in_buff_writes;
volatile byte* spi_out_buff[SPI_BUFFER_SIZE];
volatile unsigned int spi_out_buff_reads, spi_out_buff_writes;

#define I2C_BUFFER_K 3
#define I2C_BUFFER_SIZE (1 << I2C_BUFFER_K)
volatile byte* i2c_in_buff[I2C_BUFFER_SIZE];
volatile unsigned int i2c_in_buff_reads, i2c_in_buff_writes;
volatile byte* i2c_out_buff[I2C_BUFFER_SIZE];
volatile unsigned int i2c_out_buff_reads, i2c_out_buff_writes;

// Then we'll define specific read and write functions for each buffer using
// the general buff_read and buff_write functions we've already defined.

bool spi_in_read(byte* item) {
  return buff_read(item, spi_in_buff, SPI_BUFFER_SIZE,
                   &spi_in_buff_reads, &spi_in_buff_writes);
}

bool spi_in_write(byte item) {
  return buff_write(item, spi_in_buff, SPI_BUFFER_SIZE,
                    &spi_in_buff_reads, &spi_in_buff_writes);
}



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
