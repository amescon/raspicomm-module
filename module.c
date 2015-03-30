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
#include <linux/version.h>    /* needed for KERNEL_VERSION() macro */
#include <asm/io.h>           // Needed for ioremap & iounmap
#include <asm/uaccess.h>
#include "platform.h"
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
  MAX3140_WRITE_DATA_R = 1 << 15,
  MAX3140_WRITE_DATA_TE = 1 << 10,
  MAX3140_WRITE_DATA_RTS = 1 << 9
} MAX3140_WRITE_DATA_t;

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

static void          raspicomm_irq_work_queue_handler(struct work_struct *work);
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
static void raspicommDriver_throttle(struct tty_struct * tty);
static void raspicommDriver_unthrottle(struct tty_struct * tty);


// ****************************************************************************
// **** END raspicommDriver functions ****
// ****************************************************************************

// ****************************************************************************
// **** START raspicomm private fields ****
// ****************************************************************************
static const struct tty_operations raspicomm_ops = {
  .open            = raspicommDriver_open,
  .close           = raspicommDriver_close,
  .write           = raspicommDriver_write,
  .write_room      = raspicommDriver_write_room,
  .flush_buffer    =raspicommDriver_flush_buffer,
  .chars_in_buffer = raspicommDriver_chars_in_buffer,
  .ioctl           = raspicommDriver_ioctl,
  .set_termios     = raspicommDriver_set_termios,
  .stop            = raspicommDriver_stop,
  .start           = raspicommDriver_start,
  .hangup          = raspicommDriver_hangup,
  .tiocmget        = raspicommDriver_tiocmget,
  .tiocmset        = raspicommDriver_tiocmset,
  .throttle        = raspicommDriver_throttle,
  .unthrottle      = raspicommDriver_unthrottle
};

#define IRQ_DEV_NAME "raspicomm"
#define PORT_COUNT 1

// the driver instance
static struct tty_driver* raspicommDriver;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
static struct tty_port Port;
#endif

// the number of open() calls
static int OpenCount = 0;

// ParityIsEven == true ? even : odd
static int ParityIsEven = 1;
static int ParityEnabled = 0;

// currently opened tty device
static struct tty_struct* OpenTTY = NULL;

// transmit queue
static queue_t TxQueue;

// work queue for bottom half of irq handler
static DECLARE_DELAYED_WORK(IrqDelWork, raspicomm_irq_work_queue_handler);

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
static int __init raspicomm_init()
{
  Gpio = Gpio17_Irq = -EINVAL;;
  SpiConfig = 0;

  // log the start of the initialization
  LOG("kernel module initialization");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)

  /* initialize the port */
  tty_port_init(&Port);
  Port.low_latency = 1;

  /* allocate the driver */
  raspicommDriver = tty_alloc_driver(PORT_COUNT, TTY_DRIVER_REAL_RAW);

  /* return if allocation fails */
  if (IS_ERR(raspicommDriver))
    return -ENOMEM;
#else
  /* allocate the driver */
  raspicommDriver = alloc_tty_driver(PORT_COUNT);

  /* return if allocation fails */
  if (!raspicommDriver)
    return -ENOMEM;
#endif

  // init the driver
  raspicommDriver->owner                 = THIS_MODULE;
  raspicommDriver->driver_name           = "raspicomm rs485";
  raspicommDriver->name                  = "ttyRPC";
  raspicommDriver->major                 = RaspicommMajorDriverNumber;
  raspicommDriver->minor_start           = 0;
  //raspicommDriver->flags                 = TTY_DRIVER_REAL_RAW;
  raspicommDriver->type                  = TTY_DRIVER_TYPE_SERIAL;
  raspicommDriver->subtype               = SERIAL_TYPE_NORMAL;
  raspicommDriver->init_termios          = tty_std_termios;
  raspicommDriver->init_termios.c_ispeed = 9600;
  raspicommDriver->init_termios.c_ospeed = 9600;
  raspicommDriver->init_termios.c_cflag  = B9600 | CREAD | CS8 | CLOCAL;

  // initialize function callbacks of tty_driver, necessary before tty_register_driver()
  tty_set_operations(raspicommDriver, &raspicomm_ops);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
  /* link the port with the driver */
  tty_port_link_device(&Port, raspicommDriver, 0);
#endif
  
  // try to register the tty driver
  if (tty_register_driver(raspicommDriver))
  {
    LOG("tty_register_driver failed");
    put_tty_driver(raspicommDriver);
    return -1; // return if registration fails
  }

  // initialize the spi0
  raspicomm_spi0_init();

  LOG ("raspicomm_init() completed");

  /* successfully initialized the module */
  return 0; 
}

// cleanup function that gets called when the module is unloaded
static void __exit raspicomm_exit()
{
  LOG ("raspicomm_exit() called");

  // unregister the driver
  if (tty_unregister_driver(raspicommDriver))
    LOG("tty_unregister_driver failed");

  put_tty_driver(raspicommDriver);

  // free mapped memory
  raspicomm_spi0_deinit_mem(Spi0);

  // free the irq
  raspicomm_spi0_deinit_irq();

  // free gpio
  raspicomm_spi0_deinit_gpio();

  // log the unloading of the rs-485 module
  LOG("kernel module exit");
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

  value |= MAX3140_WRITE_CONFIG;

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

  LOG( "raspicomm_max3140_configure() called speed=%i, databits=%i, stopbits=%i, parity=%i => config: %X, swBacksleep: %i", speed, databits, stopbits, parity, config, swBacksleep);

  SpiConfig = config;
  SwBacksleep = swBacksleep;
}

// initializes the spi0 for supplied configuration
static void raspicomm_max3140_apply_config()
{ 
  raspicomm_spi0_send( SpiConfig );

  /* write data (R set, T not set) and enable receive by disabling RTS (TE set so that no data is sent) */
  raspicomm_spi0_send( MAX3140_WRITE_DATA_R | MAX3140_WRITE_DATA_TE | MAX3140_WRITE_DATA_RTS);
}

// Uncommented by javicient

static int raspicomm_max3140_get_parity_flag(char c)
{
  // even parity: is 1 if number of ones is odd -> making number of bits of value and parity = even
  // odd parity: is 1 if the number of ones is even -> making number of bits of value and parity = odd

  int parityEven = ParityIsEven;
  int parityEnabled = ParityEnabled;
  int count = 0, i;
  int ret;

  if (parityEnabled == 0)
    return 0;

 // count the number of ones  
   for (i = 0; i < 8; i++)
     if (c & (1 << i))
       count++;

   if (parityEven)
     ret = (count % 2) ? MAX3140_wd_Pt : 0;
   else
     ret = (count % 2) ? 0 : MAX3140_wd_Pt;

   LOG ( "raspicomm_max3140_get_parity_flag(c=%c) parityEven=%i, count=%i, ret=%i", c, parityEven, count, ret );

   return ret;
 }



static int raspicomm_spi0_send(unsigned int mosi)
{
  // TODO direct pointer access should not be used -> use kernel functions iowriteX() instead see http://www.makelinux.net/ldd3/chp-9-sect-4
  unsigned char v1,v2;
  int status;

  //LOG ("raspicomm_spi0_send(%X): %X spi0+1 %X spi0+2 %X", mosi, SPI0_CNTLSTAT, SPI0_FIFO, SPI0_CLKSPEED );

  // Set up for single ended, MS comes out first
  v1 = mosi >> 8;
  v2 = mosi & 0x00FF;

  // Enable SPI interface: Use CS 0 and set activate bit
  SPI0_CNTLSTAT = SPI0_CS_CHIPSEL0 | SPI0_CS_ACTIVATE;

  // Write the command into the FIFO
  SPI0_FIFO = v1;
  SPI0_FIFO = v2;

  do {
     status = SPI0_CNTLSTAT;
  } while ( ((status & SPI0_CS_DONE) == 0) &&
            ((status & SPI0_TA) == SPI0_TA) );
  SPI0_CNTLSTAT = SPI0_CS_DONE; // clear the done bit

  if (((status & SPI0_CS_DONE) == 0) && ((status & SPI0_TA) == 0))
    LOG_INFO("spi transfer was not done, but transfer was not active anymore!");

  // Data from the ADC chip should now be in the receiver
  // read the received data
  v1 = SPI0_FIFO;
  v2 = SPI0_FIFO;

  LOG( "raspicomm_spi0_send(%X) recv: %X", mosi, ( (v1<<8) | (v2) ) );
  //if (use_backsleep)
  //  udelay(SwBacksleep);

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

  LOG( "ioremap(%X) returned %X", SPI0_BASE, (int)p);
  LOG( "spi0: %X spi0+1 %X spi0+2 %X", *p, *(p+1), *(p+2) );

  return p;
}

// the bottom half of the irq handler, is allowed to get some sleep
static void raspicomm_irq_work_queue_handler(struct work_struct *work)
{
  /* enable receive by disabling RTS (TE set so that no data is sent)*/
  raspicomm_spi0_send(MAX3140_WRITE_DATA_R | MAX3140_WRITE_DATA_RTS | MAX3140_WRITE_DATA_TE);
}

// irq handler, that gets fired when the gpio 17 falling edge occurs
irqreturn_t raspicomm_irq_handler(int irq, void* dev_id)
{
  //// schedule the bottom half of the irq
  //schedule_work( &IrqWork );

  int rxdata, txdata;

  // issue a read command to discover the cause of the interrupt
  rxdata = raspicomm_spi0_send(MAX3140_READ_DATA);

  /* if data is available in the receive register */
  if (rxdata & MAX3140_UART_R)
  {
    // handle the received data
    raspicomm_rs485_received(OpenTTY, rxdata & 0x00FF);
  }
  /* if the transmit buffer is empty */
  else if ((rxdata & MAX3140_UART_T))
  {
    /* get the data to send from the transmit queue */
    if (queue_dequeue(&TxQueue, &txdata))
    {
      ///* enable the transmit buffer empty interrupt */
      //raspicomm_spi0_send((SpiConfig = SpiConfig | MAX3140_WRITE_CONFIG | MAX3140_UART_TM));

      /* send the data */
      raspicomm_spi0_send(MAX3140_WRITE_DATA | txdata | raspicomm_max3140_get_parity_flag((char)txdata));

      /* enable the transmit buffer empty interrupt again */
      //raspicomm_spi0_send((SpiConfig = SpiConfig | MAX3140_WRITE_CONFIG | MAX3140_UART_TM));

      /* enable the transmit buffer empty interrupt again. If already empty is returned, set it once again, needed for baudrate 2400 and lower when sending more bytes at once */
      while (MAX3140_UART_T == raspicomm_spi0_send((SpiConfig = SpiConfig | MAX3140_WRITE_CONFIG | MAX3140_UART_TM)))
        raspicomm_spi0_send((SpiConfig = SpiConfig | MAX3140_WRITE_CONFIG | MAX3140_UART_TM));
    }
    else
    {
      /* set bits R + T (bit 15 + bit 14) and clear TM (bit 11) transmit buffer empty */
      raspicomm_spi0_send((SpiConfig = (SpiConfig | MAX3140_WRITE_CONFIG) & ~MAX3140_UART_TM));

      /* give the max3140 enough time to send the data over usart before disabling RTS, else the transmission is broken */
      if (SwBacksleep < 500) {
        udelay(SwBacksleep);

        /* enable receive by disabling RTS (TE set so that no data is sent)*/
        raspicomm_spi0_send(MAX3140_WRITE_DATA_R | MAX3140_WRITE_DATA_RTS | MAX3140_WRITE_DATA_TE);

      } else {
        /* hand the processing of the RTS line over to a non-interrupt routine,
           because the delay is too long to use udelay() */
        schedule_delayed_work(&IrqDelWork, usecs_to_jiffies(SwBacksleep));
      }
    }
  }

  return IRQ_HANDLED;
}

// sets the specified gpio to the alternative function from the argument
static void raspicomm_spi0_init_gpio_alt(int gpio, int alt)
{
  volatile unsigned int* p;
  int address;

  LOG("raspicomm_spi0_init_gpio_alt(gpio=%i, alt=%i) called", gpio, alt);

  // calc the memory address for manipulating the gpio
  address = GPIO_BASE + (4 * (gpio / 10) );

  // map the gpio into kernel memory
  p = ioremap(address, 4);

  // if the mapping was successful
  if (p != NULL) {

    LOG("ioremap returned %X", (int)p );

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

  LOG ( "raspicomm_spi0_init_gpio() called with %i gpios", length );

  for (i = 0; i < length; i++)
  {
    if ( gpio_request_one( GpioConfigs[i].gpio, GPIOF_IN, "SPI" ) == 0 )
    {
      GpioConfigs[i].gpio_requested = GpioConfigs[i].gpio; // mark the gpio as successfully requested

      LOG ( "gpio_request_one(%i) succeeded", GpioConfigs[i].gpio);

      // set the alternative function according      
      raspicomm_spi0_init_gpio_alt( GpioConfigs[i].gpio, GpioConfigs[i].gpio_alternative );
    }
    else {
      printk( KERN_ERR "raspicomm: gpio_request_one(%i) failed", GpioConfigs[i].gpio );
      ret--;
    }
  }

  return ret;
}

static void raspicomm_spi0_deinit_gpio()
{
  int i, length = sizeof(GpioConfigs) / sizeof(gpioconfig);
 
  LOG( "raspicomm_spi0_deinit_gpio() called" );

  // frees all gpios that we successfully requested  
  for (i = 0; i < length; i++)
  {
    if ( GpioConfigs[i].gpio_requested ) 
    {
      LOG( "Freeing gpio %i", GpioConfigs[i].gpio_requested );

      gpio_free( GpioConfigs[i].gpio_requested );
      GpioConfigs[i].gpio_requested = 0;

    }
  }
}

static int raspicomm_spi0_init_irq()
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
  err = request_irq(irq,                           // the irq we want to receive
                    raspicomm_irq_handler,         // our irq handler function
                    IRQF_TRIGGER_FALLING,          // irq is triggered on the falling edge
                    IRQ_DEV_NAME,                  // device name that is displayed in /proc/interrupts
                    (void*)(raspicomm_irq_handler) // a unique id, needed to free the irq
                   );

  if (err) {
    printk( KERN_ERR "raspicomm: request_irq(%i) failed with error=%i", irq, err);
    return -1;
  }

  LOG ( "raspicomm_spi0_init_irq completed successfully");

  return SUCCESS;
}

static void raspicomm_spi0_deinit_irq()
{
  int gpio = Gpio;
  int irq = Gpio17_Irq;

  // if we've got a valid irq, free it
  if (irq > 0)  {

    // disable the irq first
    LOG( "Disabling irq ");
    disable_irq(irq);

    // free the irq
    LOG( "Freeing irq" );
    free_irq(irq, (void*)(raspicomm_irq_handler));
    Gpio17_Irq = 0;
  }

  // if we've got a valid gpio, free it
  if (gpio > 0) {
    LOG ( "Freeing gpio" );
    gpio_free(gpio);
    Gpio = 0;
  }

}

// initializes the spi0 using the memory region Spi0
static void raspicomm_spi0_init_port()
{
  // 1 MHz spi clock
  //SPI0_CLKSPEED = 250 / 1;
  SPI0_CLKSPEED = 80;

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
  LOG( "raspicomm_rs485_received(c=%c)", c);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
  if (tty != NULL && tty->port != NULL)
  {
    // send the character to the tty
    tty_insert_flip_char(tty->port, c, TTY_NORMAL);

    // tell it to flip the buffer
    tty_flip_buffer_push(tty->port);
  }
#else
  if (tty != NULL)
  {
    // send the character to the tty
    tty_insert_flip_char(tty, c, TTY_NORMAL);

    // tell it to flip the buffer
    tty_flip_buffer_push(tty);
  }
#endif

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
  LOG("raspicommDriver_open() called");

  if (OpenCount++)
  {
    LOG( "raspicommDriver_open() was not successful as OpenCount = %i", OpenCount);

    return -ENODEV;
  }
  else
  {
    LOG( "raspicommDriver_open() was successful");

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
  LOG("raspicommDriver_close called");

  if (--OpenCount)
  {
    LOG( "device was not closed, as an open count is %i", OpenCount);
  }
  else
  {
    OpenTTY = NULL;
    LOG( "device was closed");
  }
}

// called by the kernel after write() is called from userspace and write_room() returns > 0
static int raspicommDriver_write(struct tty_struct* tty, 
                                 const unsigned char* buf,
                                 int count)
{
  int bytes_written = 0;
  unsigned long flags;

  LOG ("raspicommDriver_write(count=%i)\n", count);

  while (bytes_written < count)
  {
    if (queue_enqueue(&TxQueue, buf[bytes_written]))
    {
      bytes_written++;
    }
    else
      cpu_relax();

    disable_irq(Gpio17_Irq);
    local_irq_save(flags);

    raspicomm_irq_handler(0, 0);

    local_irq_restore(flags);
    enable_irq(Gpio17_Irq);

  }

  return bytes_written;
}

// called by kernel to evaluate how many bytes can be written
static int raspicommDriver_write_room(struct tty_struct *tty)
{
  return INT_MAX;
}

static void raspicommDriver_flush_buffer(struct tty_struct * tty)
{
  LOG("raspicommDriver_flush_buffer called");
}

static int raspicommDriver_chars_in_buffer(struct tty_struct * tty)
{
  //LOG("raspicommDriver_chars_in_buffer called");
  return 0;
}

// called by the kernel when cfsetattr() is called from userspace
static void raspicommDriver_set_termios(struct tty_struct* tty, struct ktermios* kt)
{
  int cflag;
  speed_t baudrate; Databits databits; Parity parity; Stopbits stopbits;

  LOG("raspicommDriver_set_termios() called");

  // get the baudrate
  baudrate = tty_get_baud_rate(tty);

  // get the cflag
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
  cflag = tty->termios.c_cflag;
#else
  cflag = tty->termios->c_cflag;
#endif

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
  LOG("raspicommDriver_stop called");
}

static void raspicommDriver_start(struct tty_struct * tty)
{
  LOG("raspicommDriver_start called");
}

static void raspicommDriver_hangup(struct tty_struct * tty)
{
  LOG("raspicommDriver_hangup called");
}

static int raspicommDriver_tiocmget(struct tty_struct *tty)
{
  LOG("raspicommDriver_tiocmget called");
  return 0;
}

static int raspicommDriver_tiocmset(struct tty_struct *tty,
                              unsigned int set, 
                              unsigned int clear)
{
  LOG("raspicommDriver_tiocmset called");
  return 0;
}

// called by the kernel to get/set data
static int raspicommDriver_ioctl(struct tty_struct* tty,
                           unsigned int cmd,
                           unsigned int long arg)
{
  int ret;

  // LOG("raspicomm: raspicommDriver_ioctl called");

  LOG ("raspicommDriver_ioctl() called with cmd=%i, arg=%li", cmd, arg);

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

static void raspicommDriver_throttle(struct tty_struct * tty)
{
  LOG_INFO("throttle");
}

static void raspicommDriver_unthrottle(struct tty_struct * tty)
{
  LOG_INFO("unthrottle");
}


// ****************************************************************************
// **** END raspicommDriver interface functions ****
// ****************************************************************************
