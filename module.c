#include <linux/module.h>     // Needed by all modules
#include <linux/kernel.h>     // Needed for KERN_INFO
#include <linux/init.h>       // Needed for the macros
#include <linux/fs.h>         // Needed for the file structure & register_chrdev()
#include <linux/tty.h>        // 
#include <linux/tty_driver.h> // Needed for struct tty_driver

#include <linux/gpio.h>       // needed for gpio_X() calls

#include <linux/interrupt.h>  // Needed for request_interrupt()
#include <linux/workqueue.h>

#include <linux/delay.h>      // Needed for udelay

#include <linux/tty_flip.h> 
#include <linux/serial.h>

#include <asm/io.h>           // Needed for ioremap & iounmap

#include <asm/io.h>
#include <asm/uaccess.h>

#include "module.h"
#include "queue.h"     // needed for queue_xxx functions

// MajorDriverNumber == 0 is using dynamically number
static const int RaspicommMajorDriverNumber = 0;

// struct that holds the gpio configuration
typedef struct {
  int gpio;             // set to the gpio that should be requested
  int gpio_alternative; // set to the alternative that the gpio should be configured
  int gpio_requested;   // set if the gpio was successfully requested, otherwise 0
} gpioconfig;

typedef enum {
  STOPBITS_ONE = 0,
  STOPBITS_TWO = 1
} Stopbits;

typedef enum {
  DATABITS_7 = 1,
  DATABITS_8 = 0
} Databits;

typedef enum {
  PARITY_OFF = 0,
  PARITY_ON =  1
} Parity;

typedef enum {
  MAX3140_UART_R     = 1 << 15, 
  MAX3140_UART_T     = 1 << 14,
  MAX3140_UART_FEN   = 1 << 13,
  MAX3140_UART_SHDNo = 1 << 12,
  MAX3140_UART_TM    = 1 << 11,
  MAX3140_UART_RM    = 1 << 10,
  MAX3140_UART_PM    = 1 << 9,
  MAX3140_UART_RAM   = 1 << 8,
  MAX3140_UART_IR    = 1 << 7,
  MAX3140_UART_ST    = 1 << 6,
  MAX3140_UART_PE    = 1 << 5,  // Parity Enable
  MAX3140_UART_L     = 1 << 4,
  MAX3140_UART_B3    = 1 << 3,
  MAX3140_UART_B2    = 1 << 2,
  MAX3140_UART_B1    = 1 << 1,
  MAX3140_UART_B0    = 1 << 0,

  MAX3140_wd_Pt = 1 << 8  

} MAX3140_UartFlags;

#define MAX3140_WRITE_CONFIG ( MAX3140_UART_R | MAX3140_UART_T )
#define MAX3140_READ_CONFIG ( MAX3140_UART_T )
#define MAX3140_READ_DATA ( 0 )
#define MAX3140_WRITE_DATA ( MAX3140_UART_R )


// ****************************************************************************
// **** START raspicomm private functions ****
// ****************************************************************************
// forward declarations of private functions
static int __init raspicomm_init(void);
static void __exit raspicomm_exit(void);

static int raspicomm_get_free_write_buffer_length(void);

static int           raspicomm_max3140_get_swbacksleep   (speed_t speed);
static unsigned char raspicomm_max3140_get_baudrate_index(speed_t speed);
static unsigned int  raspicomm_max3140_get_uart_config   (speed_t speed, 
                                                          Databits databits, 
                                                          Stopbits stopbits, 
                                                          Parity parity);
static void          raspicomm_max3140_configure         (speed_t speed, 
                                                          Databits databits, 
                                                          Stopbits stopbits, 
                                                          Parity parity);
static void          raspicomm_max3140_apply_config(void);
// static int           raspicomm_max3140_get_parity_flag(char value);

static int           raspicomm_spi0_send(unsigned int mosi);
static void          raspicomm_rs485_received(struct tty_struct* tty, char c);

static void          raspicomm_irq_work_queue_handler(void* args);
irqreturn_t          raspicomm_irq_handler(int irq, void* dev_id);

static void                   raspicomm_spi0_init(void);
volatile static unsigned int* raspicomm_spi0_init_mem(void);
static int                    raspicomm_spi0_init_gpio(void);
static void                   raspicomm_spi0_init_gpio_alt(int gpio, int alt);
static void                   raspicomm_spi0_deinit_gpio(void);
static int                    raspicomm_spi0_init_irq(void);
static void                   raspicomm_spi0_deinit_irq(void);
static void                   raspicomm_spi0_init_port(void);
static void                   raspicomm_spi0_deinit_mem(volatile unsigned int* spi0);

// ****************************************************************************
// *** END raspicomm private functions ****
// ****************************************************************************

// ****************************************************************************
// **** START raspicommDriver functions ****
// ****************************************************************************
static int  raspicommDriver_open (struct tty_struct *, struct file *);
static void raspicommDriver_close(struct tty_struct *, struct file *);
static int  raspicommDriver_write(struct tty_struct *, 
                                  const unsigned char *, 
                                  int);
static int  raspicommDriver_write_room(struct tty_struct *);
static void raspicommDriver_flush_buffer(struct tty_struct *);
static int  raspicommDriver_chars_in_buffer(struct tty_struct *);
static void raspicommDriver_set_termios(struct tty_struct *, struct ktermios *);
static void raspicommDriver_stop(struct tty_struct *);
static void raspicommDriver_start(struct tty_struct *);
static void raspicommDriver_hangup(struct tty_struct *);
static int  raspicommDriver_tiocmget(struct tty_struct *tty);
static int  raspicommDriver_tiocmset(struct tty_struct *tty,
                                     unsigned int set, 
                                     unsigned int clear);
static int  raspicommDriver_ioctl(struct tty_struct* tty,
                                  unsigned int cmd,
                                  unsigned int long arg);
// static void raspicommDriver_poll(unsigned long);
// static void raspicommDriver_set_tty_param(struct tty_struct *, struct ktermios *);
// static void raspicommDriver_shutdown(struct tty_port *);
// static int raspicommDriver_carrier_raised(struct tty_port *);
// static void raspicommDriver_dtr_rts(struct tty_port *, int);
// ****************************************************************************
// **** END raspicommDriver functions ****
// ****************************************************************************

// ****************************************************************************
// **** START raspicomm private fields ****
// ****************************************************************************
static const struct tty_operations raspicomm_ops = {
  .open = raspicommDriver_open,
  .close = raspicommDriver_close,
  .write = raspicommDriver_write,
  .write_room = raspicommDriver_write_room,
  .flush_buffer =raspicommDriver_flush_buffer,
  .chars_in_buffer = raspicommDriver_chars_in_buffer,
  .ioctl = raspicommDriver_ioctl,
  .set_termios = raspicommDriver_set_termios,
  .stop= raspicommDriver_stop,
  .start = raspicommDriver_start,
  .hangup = raspicommDriver_hangup,
  .tiocmget = raspicommDriver_tiocmget,
  .tiocmset = raspicommDriver_tiocmset
};

#define IRQ_DEV_NAME "raspicomm"

// the driver instance
static struct tty_driver* raspicommDriver;

// the number of open() calls
static int OpenCount = 0;

// ParityIsEven == true ? even : odd
static int ParityIsEven = 1;
static int ParityEnabled = 0;

// currently opened tty device
static struct tty_struct* OpenTTY;

// transmit queue
static struct queue TxQueue;

// mutex to secure the spi data transfer
static DEFINE_MUTEX( SpiLock );

// work queue for bottom half of irq handler
static DECLARE_WORK( IrqWork, raspicomm_irq_work_queue_handler );

// variable used in the delay to simulate the baudrate
static int SwBacksleep;

// config setting of the spi0
static int SpiConfig;

// Spi0 memory interface pointer
volatile static unsigned int* Spi0;

// The requested gpio (set by raspicomm_spi0_init_gpio and freed by raspicomm_spi0_deinit_gpio)
static int Gpio;

// The interrupt that signals when data is available
static int Gpio17_Irq;

// the configured gpios
static gpioconfig GpioConfigs[] =  { {7}, {8}, {9}, {10}, {11} };

// ****************************************************************************
// **** END raspicomm private fields
// ****************************************************************************


// ****************************************************************************
// **** START module specific functions ****
// ****************************************************************************
// module entry point function - gets called from insmod
module_init(raspicomm_init);

// module exit point function - gets called from rmmod
module_exit(raspicomm_exit);
// ****************************************************************************
// **** END module specific functions ****
// ****************************************************************************

// ****************************************************************************
// **** START raspicomm private function implementations
// ****************************************************************************
// initialization function that gets called when the module is loaded
static int __init raspicomm_init(void)
{
  Gpio = Gpio17_Irq = -EINVAL;;
  SpiConfig = 0;

  // log the start of the initialization
  #if DEBUG
    LOG("raspicomm: kernel module initialization");
  #endif

  raspicommDriver = alloc_tty_driver(1);

  // return if allocation fails
  if (!raspicommDriver)
    return -ENOMEM;

  // init the driver
  raspicommDriver->driver_name = "raspicomm rs485";
  raspicommDriver->name = "ttyRPC";
  raspicommDriver->major = RaspicommMajorDriverNumber;
  raspicommDriver->minor_start = 0;
  raspicommDriver->num = 1;
  raspicommDriver->type = TTY_DRIVER_TYPE_SERIAL;
  raspicommDriver->subtype = SERIAL_TYPE_NORMAL;
  raspicommDriver->init_termios = tty_std_termios;
  raspicommDriver->init_termios.c_ispeed = 9600;
  raspicommDriver->init_termios.c_ospeed = 9600;
  raspicommDriver->init_termios.c_cflag = B9600 | CREAD | CS8 | CLOCAL;

  // initialize function callbacks of tty_driver, necessary before tty_register_driver()
  tty_set_operations(raspicommDriver, &raspicomm_ops);
  
  // try to register the tty driver
  if (tty_register_driver(raspicommDriver))
  {
    LOG("raspicomm: tty_register_driver failed");
    put_tty_driver(raspicommDriver);
    return -1; // return if registration fails
  }
  
  // initialize the spi0
  raspicomm_spi0_init();

	return 0;
}

// cleanup function that gets called when the module is unloaded
static void __exit raspicomm_exit(void)
{
  // unregister the driver
  if (tty_unregister_driver(raspicommDriver))
    LOG("raspicomm: tty_unregister_driver failed");

  put_tty_driver(raspicommDriver);

  // free mapped memory
  raspicomm_spi0_deinit_mem(Spi0);

  // free the irq
  raspicomm_spi0_deinit_irq();

  // free gpio
  raspicomm_spi0_deinit_gpio();

  // log the unloading of the rs-485 module
  LOG("raspicomm: kernel module exit");
}

// Helper function that calculates the size of the free buffer remaining
static int raspicomm_get_free_write_buffer_length(void)
{
  int room;

  mutex_lock( &SpiLock );
  room = queue_get_room(&TxQueue);
  mutex_unlock( &SpiLock );

  return room;
}

// helper function
static unsigned char raspicomm_max3140_get_baudrate_index(speed_t speed)
{
  switch (speed)
  {
    case 600: return 0x7;
    case 1200: return 0xE;
    case 2400: return 0xD;
    case 4800: return 0xC;
    case 9600: return 0xB;
    case 19200: return 0xA;
    case 38400: return 0x9;
    case 57600: return 0x2;
    case 115200: return 0x1;
    case 230400: return 0x0;
    default: return raspicomm_max3140_get_baudrate_index(9600);
  }
}

// helper function that creates the config for spi
static unsigned int raspicomm_max3140_get_uart_config(speed_t speed, Databits databits, Stopbits stopbits, Parity parity)
{
  unsigned int value = 0;

  value |= (MAX3140_UART_R | MAX3140_UART_T);

  value |= MAX3140_UART_RM;

  value |= raspicomm_max3140_get_baudrate_index(speed);

  value |= stopbits << 6;

  value |= parity << 5;

  value |= databits << 4;

  return value;
}

static int raspicomm_max3140_get_swbacksleep(speed_t speed)
{
  return 10000000 / speed;
}

static void raspicomm_max3140_configure(speed_t speed, Databits databits, Stopbits stopbits, Parity parity)
{
  int swBacksleep = raspicomm_max3140_get_swbacksleep(speed), 
      config = raspicomm_max3140_get_uart_config(speed, databits, stopbits, parity);

  #if DEBUG
    printk ( KERN_INFO "raspicomm: raspicomm_max3140_configure() called speed=%i, databits=%i, stopbits=%i, parity=%i => config: %X, swBacksleep: %i", speed, databits, stopbits, parity, config, swBacksleep);
  #endif

  SpiConfig = config;
  SwBacksleep = swBacksleep;
}

// initializes the spi0 for supplied configuration
static void raspicomm_max3140_apply_config(void)
{ 
  mutex_lock ( &SpiLock );
  raspicomm_spi0_send( SpiConfig );
  raspicomm_spi0_send( 0x8600 ); // enable receive by disabling RTS (TE set so that no data is sent)
  mutex_unlock ( &SpiLock );
}

// static int raspicomm_max3140_get_parity_flag(char c)
// {
//   // even parity: is 1 if number of ones is odd -> making number of bits of value and parity = even
//   // odd parity: is 1 if the number of ones is even -> making number of bits of value and parity = odd

//   int parityEven = ParityIsEven;
//   int parityEnabled = ParityEnabled;
//   int count = 0, i;
//   int ret;

//   if (parityEnabled == 0)
//     return 0;

//   // count the number of ones  
//   for (i = 0; i < 8; i++)
//     if (c & (1 << i))
//       count++;

//   if (parityEven)
//     ret = (count % 2) ? MAX3140_wd_Pt : 0;
//   else
//     ret = (count % 2) ? 0 : MAX3140_wd_Pt;  

//   #if DEBUG
//     printk( KERN_INFO "raspicomm: raspicomm_max3140_get_parity_flag(c=%c) parityEven=%i, count=%i, ret=%i", c, parityEven, count, ret );
//   #endif

//   return ret;
// }

#define BCM2708_PERI_BASE 0x20000000

#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)

// ****************************************************************************
// **** START SPI defines ****
// ****************************************************************************
#define SPI0_BASE (BCM2708_PERI_BASE + 0x204000)
#define SPI0_CNTLSTAT *(Spi0 + 0)
#define SPI0_FIFO *(Spi0 + 1)
#define SPI0_CLKSPEED *(Spi0 + 2)

// SPI transfer done. WRT to CLR!
#define SPI0_CS_DONE     0x00010000 

 // Activate: be high before starting
#define SPI0_CS_ACTIVATE 0x00000080
#define SPI0_CS_CLRFIFOS 0x00000030
#define SPI0_CS_CHIPSEL0 0x00000000
#define SPI0_CS_CHIPSEL1 0x00000001
#define SPI0_CS_CHIPSEL2 0x00000002
#define SPI0_CS_CHIPSELN 0x00000003

#define SPI0_CS_CLRALL (SPI0_CS_CLRFIFOS | SPI0_CS_DONE)
// ****************************************************************************
// **** END SPI defines ****
// ****************************************************************************

static int raspicomm_spi0_send(unsigned int mosi)
{
  // TODO direct pointer access should not be used -> use kernel functions iowriteX() instead see http://www.makelinux.net/ldd3/chp-9-sect-4
  unsigned char v1,v2;
  int status;

  #if DEBUG
    printk ( KERN_INFO "raspicomm: raspicomm_spi0_send(%X): %X spi0+1 %X spi0+2 %X", mosi, SPI0_CNTLSTAT, SPI0_FIFO, SPI0_CLKSPEED );
  #endif

  // Set up for single ended, MS comes out first
  v1 = mosi >> 8;
  v2 = mosi & 0x00FF;

  // Enable SPI interface: Use CS 0 and set activate bit
  SPI0_CNTLSTAT = SPI0_CS_CHIPSEL0 | SPI0_CS_ACTIVATE;

  // Delay to make sure chip select is high for a short while
  // udelay(100);

  // Write the command into the FIFO
  SPI0_FIFO = v1;
  SPI0_FIFO = v2;

  // TODO: the following code can lockup the kernel in an interrupt! We need to make sure it stops after a timeout
  // wait for SPI to be ready
  // This will take about 16 micro seconds
  do {
     status = SPI0_CNTLSTAT;
  } while ( (status & SPI0_CS_DONE) == 0 );
  SPI0_CNTLSTAT = SPI0_CS_DONE; // clear the done bit

  // Data from the ADC chip should now be in the receiver
  // read the received data
  v1 = SPI0_FIFO;
  v2 = SPI0_FIFO;

  // #if DEBUG
  //   printk( KERN_INFO "raspicomm: raspicomm_spi0_send(%X) returned: %X", mosi, ( (v1<<8) | (v2) ) );
  // #endif
 
  return ( (v1<<8) | (v2) );
}

// one time initialization for the spi0 
static void raspicomm_spi0_init(void)
{
  // map the spi0 memory
  Spi0 = raspicomm_spi0_init_mem();  

  // initialize the spi0
  raspicomm_spi0_init_port();

  // init the gpios
  raspicomm_spi0_init_gpio();

  // register the irq for the spi0
  raspicomm_spi0_init_irq();

  raspicomm_max3140_configure(9600, DATABITS_8, STOPBITS_ONE, PARITY_OFF);
  raspicomm_max3140_apply_config();

}

// map the physical memory that we need for spi0 access
volatile static unsigned int* raspicomm_spi0_init_mem(void)
{
  // in user space we would do mmap() call, in kernel space we do ioremap

  // call ioremap to map the physical address to something we can use
  unsigned int* p = ioremap(SPI0_BASE, 12);

  #if DEBUG
    printk( KERN_INFO "raspicomm: ioremap(%X) returned %X", SPI0_BASE, (int)p);

    printk ( KERN_INFO "raspicomm: spi0: %X spi0+1 %X spi0+2 %X", *p, *(p+1), *(p+2) );
  #endif  

  return p;
}

// the bottom half of the irq handler, is allowed to get some sleep
void raspicomm_irq_work_queue_handler(void *arg)
{
  int rxdata, txdata;

  // lock on the transmit queue
  mutex_lock( &SpiLock );

  // #if DEBUG
  //   printk( KERN_INFO "raspicomm: raspicomm_irq_handler (%i)", irq);
  // #endif

  // issue a read command to discover the cause of the interrupt
  rxdata = raspicomm_spi0_send(0);

  // DEBUG: log data
  // #if DEBUG
  //   printk( KERN_INFO "raspicomm: raspicomm_irq_handler rxdata=%X", rxdata);
  // #endif

  // read the data 
  while(rxdata & MAX3140_UART_R) // while we are receiving
  {
    // handle the received data
    raspicomm_rs485_received( OpenTTY, rxdata & 0x00FF );

    // get the next rxdata
    rxdata = raspicomm_spi0_send(0);
  }

  // write the data
  if ((rxdata & MAX3140_UART_T) && (SpiConfig & MAX3140_UART_TM))
  {


    if (!queue_is_empty(&TxQueue))
    {
      txdata = queue_dequeue(&TxQueue);

      // parity flag
      // raspicomm_spi0_send( MAX3140_UART_TM | txdata | raspicomm_max3140_get_parity_flag((char)txdata) );
      
      // raspicomm_spi0_send( 0x8000| txdata );
      raspicomm_spi0_send( MAX3140_UART_R | txdata );
    }
    else
    {
      // mask transmit buffer empty interrupt
      SpiConfig = SpiConfig & ~0x0800; // clear the TM bit
      SpiConfig = SpiConfig | 0xc000; // set bits 15 and 14
      raspicomm_spi0_send(SpiConfig);

      // usleep(SwBacksleep);
      udelay(SwBacksleep); // there is no usleep function in the kernel      

      raspicomm_spi0_send(0x8600); // enable receive by disabling RTS (TE set so that no data is sent)
    }
  }

  // unlock the transmit queue
  mutex_unlock( &SpiLock );
}

// irq handler, that gets fired when the gpio 17 falling edge occurs
irqreturn_t raspicomm_irq_handler(int irq, void* dev_id)
{
  // schedule the bottom half of the irq
  schedule_work( &IrqWork );

  return IRQ_HANDLED;
}

// sets the specified gpio to the alternative function from the argument
static void raspicomm_spi0_init_gpio_alt(int gpio, int alt)
{
  volatile unsigned int* p;
  int address;
  
  #if DEBUG
    printk( KERN_INFO "raspicomm: raspicomm_spi0_init_gpio_alt(gpio=%i, alt=%i) called", gpio, alt);
  #endif

  // calc the memory address for manipulating the gpio
  address = GPIO_BASE + (4 * (gpio / 10) );

  // map the gpio into kernel memory
  p = ioremap(address, 4);

  // if the mapping was successful
  if (p != NULL) {
    #if DEBUG
      printk ( KERN_INFO "raspicomm: ioremap returned %X", (int)p );
    #endif

    // set the gpio to the alternative mapping
    (*p) |= (((alt) <= 3 ? (alt) + 4 : (alt) == 4 ? 3 : 2) << (((gpio)%10)*3));

    // free the gpio mapping again
    iounmap(p);
  }
}

// init the gpios as specified in 
static int raspicomm_spi0_init_gpio()
{
  int i, length = sizeof(GpioConfigs) / sizeof(gpioconfig), ret = SUCCESS;

  #if DEBUG
    printk ( KERN_INFO "raspicomm: raspicomm_spi0_init_gpio() called with %i gpios", length );
  #endif

  for (i = 0; i < length; i++)
  {
    if ( gpio_request_one( GpioConfigs[i].gpio, GPIOF_IN, "SPI" ) == 0 )
    {
      GpioConfigs[i].gpio_requested = GpioConfigs[i].gpio; // mark the gpio as successfully requested
      #if DEBUG
        printk( KERN_INFO "raspicomm: gpio_request_one(%i) succeeded", GpioConfigs[i].gpio);
      #endif

      // set the alternative function according      
      raspicomm_spi0_init_gpio_alt( GpioConfigs[i].gpio, GpioConfigs[i].gpio_alternative );
    }
    else {
      printk( KERN_ERR "raspicomm: gpio_request_one(%i) failed", GpioConfigs[i].gpio );
      ret--;
    }
  }

  #if DEBUG
  if ( ret == SUCCESS )
    printk ( KERN_INFO "raspicomm: raspicomm_spi0_init_gpio() returned successfully" );
  #endif

  return ret;
}

static void raspicomm_spi0_deinit_gpio()
{
  int i, length = sizeof(GpioConfigs) / sizeof(gpioconfig);
 
  #if DEBUG
    printk( KERN_INFO "raspicomm: raspicomm_spi0_deinit_gpio() called" );
  #endif

  // frees all gpios that we successfully requested  
  for (i = 0; i < length; i++)
  {
    if ( GpioConfigs[i].gpio_requested ) 
    {
      #if DEBUG
        printk( KERN_INFO "raspicomm: Freeing gpio %i", GpioConfigs[i].gpio_requested );
      #endif
      gpio_free( GpioConfigs[i].gpio_requested );
      GpioConfigs[i].gpio_requested = 0;

    }
  }
}

static int raspicomm_spi0_init_irq(void)
{  
  // the interrupt number and gpio number
  int irq, io = 17;

  // request the gpio and configure it as an input
  int err = gpio_request_one(io, GPIOF_IN, "SPI0");

  if (err) {
    printk( KERN_ERR "raspicomm: gpio_request_one(%i) failed with error=%i", io, err );
    return -1;
  } 

  // store the requested gpio so that it can be freed later on
  Gpio = io;

  // map it to an irq
  irq = gpio_to_irq(io);

  if (irq < 0) {
    printk( KERN_ERR "raspicomm: gpio_to_irq failed with error=%i", irq );
    return -1;
  }

  // store the irq so that it can be freed later on
  Gpio17_Irq = irq;

  // request the interrupt
  err = request_irq(irq,                                // the irq we want to receive
                    raspicomm_irq_handler,              // our irq handler function
                    IRQF_SHARED | IRQF_TRIGGER_FALLING, // irq is shared and triggered on the falling edge                                         
                    IRQ_DEV_NAME,                       // device name that is displayed in /proc/interrupts
                    (void*)(raspicomm_irq_handler)      // a unique id, needed to free the irq
                   );

  if (err) {
    printk( KERN_ERR "raspicomm: request_irq(%i) failed with error=%i", irq, err);
    return -1;
  }

  #if DEBUG
    printk( KERN_INFO "raspicomm: raspicomm_spi0_init_irq completed successfully");
  #endif

  return SUCCESS;
}

static void raspicomm_spi0_deinit_irq()
{
  int gpio = Gpio;
  int irq = Gpio17_Irq;

  // if we've got a valid irq, free it
  if (irq > 0)  {

    // disable the irq first
    LOG( "raspicomm: Disabling irq ");
    disable_irq(irq);

    // free the irq
    LOG( "raspicomm: Freeing irq" );
    free_irq(irq, (void*)(raspicomm_irq_handler));
    Gpio17_Irq = 0;
  }

  // if we've got a valid gpio, free it
  if (gpio > 0) {
    LOG ( "raspicomm: Freeing gpio" );
    gpio_free(gpio);
    Gpio = 0;
  }

}

// initializes the spi0 using the memory region Spi0
static void raspicomm_spi0_init_port()
{
  // 1 MHz spi clock
  SPI0_CLKSPEED = 250;

  // clear FIFOs and all status bits
  SPI0_CNTLSTAT = SPI0_CS_CLRALL;

  SPI0_CNTLSTAT = SPI0_CS_DONE; // make sure done bit is cleared
}

// frees the memory are used to access the spi0
static void raspicomm_spi0_deinit_mem(volatile unsigned int* spi0)
{
  // after using the device call iounmap to return the address space to the kernel
  if (spi0 != NULL)
    iounmap(spi0);
}

// this function pushes a received character to the opened tty device, called by the interrupt function
static void raspicomm_rs485_received(struct tty_struct* tty, char c)
{
  #if DEBUG
    printk( KERN_INFO "raspicomm: raspicomm_rs485_received(c=%c)", c);
  #endif

  if (tty != NULL)
  {
    // check if we need to flip the buffer before inserting a char
    // if (tty->flip.count >= TTY_FLIPBUF_SIZE)
    //   tty_flip_buffer_push(tty);

    // send the character to the tty
    tty_insert_flip_char(tty, c, TTY_NORMAL);

    // tell it to flip the buffer
    tty_flip_buffer_push(tty);
  }

}

// ****************************************************************************
// **** END raspicomm private function implementations
// ****************************************************************************


// ****************************************************************************
// **** START tty driver interface function implementations
// ****************************************************************************
// called by the kernel when open() is called for the device
static int raspicommDriver_open(struct tty_struct* tty, struct file* file)
{
  LOG("raspicomm: raspicommDriver_open() called");

  if (OpenCount++)
  {
    #if DEBUG
      printk( KERN_INFO "raspicomm: raspicommDriver_open() was not successful as OpenCount = %i", OpenCount);
    #endif

    return -ENODEV;
  }
  else
  {
    #if DEBUG
      printk( KERN_INFO "raspicomm: raspicommDriver_open() was successful");
    #endif

    OpenTTY = tty;

    // TODO Do we need to reset the connection?
    // reset the connection
    // raspicomm_max3140_apply_config();

    return SUCCESS;
  }

}

// called by the kernel when close() is called for the device
static void raspicommDriver_close(struct tty_struct* tty, struct file* file)
{
  LOG("raspicomm: raspicommDriver_close called");

  if (--OpenCount)
  {
    #if DEBUG
      printk ( KERN_INFO "raspicomm: device was not closed, as an open count is %i", OpenCount);
    #endif
  }
  else    
  {
    OpenTTY = NULL;
    #if DEBUG
      printk ( KERN_INFO "raspicomm: device was closed");
    #endif
  }
}

// called by the kernel after write() is called from userspace and write_room() returns > 0
static int raspicommDriver_write(struct tty_struct* tty, 
                                 const unsigned char* buf,
                                 int count)
{
  int bytes_written = 0;
  int receive;
  // int txdata;

  #if DEBUG
    printk(KERN_INFO "raspicomm: raspicommDriver_write(count=%i)\n", count);
  #endif

  mutex_lock(&SpiLock);

  // insert them into the transmit queue
  for (bytes_written = 0; bytes_written < count; bytes_written++)
  {
    if (queue_enqueue(&TxQueue, buf[bytes_written]) < 0)
      break;
  }

  // SpiConfig = SpiConfig | 0xC800; // set bit 15, 14, TM bit
  SpiConfig = SpiConfig | MAX3140_UART_T | MAX3140_UART_R | MAX3140_UART_TM;
  receive = raspicomm_spi0_send( SpiConfig );

  if (receive & MAX3140_UART_T) // transmit buffer is ready to accept data
  {
    // txdata = queue_dequeue(&TxQueue);
    raspicomm_spi0_send( 0x8000 | queue_dequeue(&TxQueue) );
    // raspicomm_spi0_send( 0x8000 |  txdata | raspicomm_max3140_get_parity_flag((char)txdata));
  }
    
  mutex_unlock(&SpiLock);

  return bytes_written;
}

// called by kernel to evaluate how many bytes can be written
static int raspicommDriver_write_room(struct tty_struct * tty)
{
  int length = raspicomm_get_free_write_buffer_length();

  #if DEBUG
    printk(KERN_INFO "raspicomm: raspicommDriver_write_room() returns %i", length);
  #endif

  return length;
}

static void raspicommDriver_flush_buffer(struct tty_struct * tty)
{
  LOG("raspicomm: raspicommDriver_flush_buffer called");
}

static int raspicommDriver_chars_in_buffer(struct tty_struct * tty)
{
  LOG("raspicomm: raspicommDriver_chars_in_buffer called");
  return 0;
}

// called by the kernel when cfsetattr() is called from userspace
static void raspicommDriver_set_termios(struct tty_struct* tty, struct ktermios* kt)
{
  int cflag;
  speed_t baudrate; Databits databits; Parity parity; Stopbits stopbits;

  LOG("raspicomm: raspicommDriver_set_termios() called");

  // get the baudrate
  baudrate = tty_get_baud_rate(tty);

  // get the cflag
  cflag = tty->termios->c_cflag;

  // get the databits
  switch ( cflag & CSIZE )
  {
    case CS7:
      databits = DATABITS_7;
      break;

    default:
    case CS8:
      databits = DATABITS_8;
      break;
  }

  // get the stopbits
  stopbits = ( cflag & CSTOPB ) ? STOPBITS_TWO : STOPBITS_ONE;

  // get the parity
  if ( cflag & PARENB ) // is parity used
  {
    ParityIsEven = !( cflag & PARODD ); // is it even or odd? store it for sending
    parity = PARITY_ON;
    ParityEnabled = 1;
  }
  else {
    parity = PARITY_OFF;
    ParityEnabled = 0;
  }

  // #if DEBUG
  //   printk ( KERN_INFO "raspicomm: Parity=%i, ParityIsEven = %i", parity, ParityIsEven);
  // #endif
  
  // update the configuration
  raspicomm_max3140_configure(baudrate, databits, stopbits, parity);

  raspicomm_max3140_apply_config();
}

static void raspicommDriver_stop(struct tty_struct * tty)
{
  LOG("raspicomm: raspicommDriver_stop called");
}

static void raspicommDriver_start(struct tty_struct * tty)
{
  LOG("raspicomm: raspicommDriver_start called");
}

static void raspicommDriver_hangup(struct tty_struct * tty)
{
  LOG("raspicomm: raspicommDriver_hangup called");
}

static int raspicommDriver_tiocmget(struct tty_struct *tty)
{
  LOG("raspicomm: raspicommDriver_tiocmget called");
  return 0;
}

static int raspicommDriver_tiocmset(struct tty_struct *tty,
                              unsigned int set, 
                              unsigned int clear)
{
  LOG("raspicomm: raspicommDriver_tiocmset called");
  return 0;
}

// called by the kernel to get/set data
static int raspicommDriver_ioctl(struct tty_struct* tty,
                           unsigned int cmd,
                           unsigned int long arg)
{
  int ret;

  // LOG("raspicomm: raspicommDriver_ioctl called");

#if DEBUG
  printk(KERN_INFO "raspicomm: raspicommDriver_ioctl() called with cmd=%i, arg=%li", cmd, arg);
#endif

  switch (cmd)
  {
    case TIOCMSET:
      ret = 0;
      break;

    case TIOCMGET:
      ret = 0;
      break;

    default:
      ret = -ENOIOCTLCMD;
      break;
  }

  return ret;
}

// static void raspicommDriver_poll(unsigned long ignored)
// {
// }

// static void raspicommDriver_set_tty_param(struct tty_struct * tty, struct ktermios * kt)
// {
// }

// static void raspicommDriver_shutdown(struct tty_port * port)
// {
// }

// static int raspicommDriver_carrier_raised(struct tty_port * port)
// {
//   return 0;
// }

// static void raspicommDriver_dtr_rts(struct tty_port * port, int onoff)
// {
// }

// ****************************************************************************
// **** END raspicommDriver interface functions ****
// ****************************************************************************