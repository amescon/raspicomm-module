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
//#define DEBUG

// if DEBUG is defined, log using printk(), else do nothing
#ifdef DEBUG
  #define LOG(fmt, args...) do { printk( KERN_DEBUG "rpc: " fmt "\n", ## args); } while(0)
#else
  #define LOG(fmt, args...)
#endif
