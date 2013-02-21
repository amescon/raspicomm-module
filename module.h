#include <linux/kernel.h>     // Needed for KERN_INFO
#include <linux/module.h>

// ****************************************************************************
// **** START Module Defines ****
// ****************************************************************************
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mdk");
MODULE_DESCRIPTION("Raspicomm kernel module with tty driver support for rs-485");
MODULE_SUPPORTED_DEVICE("ttyRPC");

#define SUCCESS 0 

// ****************************************************************************
// **** END Module Defines ****
// ****************************************************************************

// outputs verbose debug information
#define DEBUG 0

// if DEBUG is defined, log using printk(), else do nothing
#if DEBUG

  #define LOG(x) \
  do { \
      printk(KERN_INFO x); \
  } while(0);

#else

  #define LOG(x)

#endif
