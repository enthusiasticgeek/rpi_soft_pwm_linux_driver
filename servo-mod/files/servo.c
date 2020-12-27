/*
MIT License
Copyright (c) 2020 Pratik M Tambe <enthusiasticgeek@gmail.com>
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/slab.h>         // kmalloc()
#include <linux/mutex.h>	/// Required for the mutex functionality
#include <linux/uaccess.h>
#include <linux/delay.h>

//Tested with kernel 4.9.108

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Pratik M Tambe");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A simple Linux driver for servo control through GPIO pins");  ///< The description -- see modinfo
MODULE_VERSION("0.1");              ///< A version number to inform users

#ifndef  DEVICE_NAME
#define  DEVICE_NAME "Servo"    ///< The device will appear at /dev/Servo using this value
#endif

#ifndef  CLASS_NAME
#define  CLASS_NAME  "AutomatorServo"         ///< The device class -- this is a character device driver
#endif

#ifndef  MAX_MESSAGE_SIZE
#define  MAX_MESSAGE_SIZE 4096
#endif


#ifndef PAIR1
#define PAIR1 1  //<------- change to 0 to disable all pairs
#endif

enum Commands {
    None,
    GetInfo,     //Add commands as needed
    TotalCommands
};

#if PAIR1
static const int ServoOperationGpioPin = 27;
#endif

//Depending on the version of raspberry Pi select the following for SOC_PERI_BASE macro
//The same driver may be used on multiple RPI devices on updating this code and recompiling with the appropriate address
#define RPI1_PERI_BASE 0x20000000
#define RPI_ZERO_PERI_BASE 0x20000000
#define RPI_ZERO_W_PERI_BASE 0x20000000
#define RPI2_PERI_BASE 0x3f000000
#define RPI3_PERI_BASE 0x3f000000
#define RPI3B_PERI_BASE 0x3f000000
#define RPI4_PERI_BASE 0xfe000000

static DEFINE_MUTEX(Servo_mutex);  /// A macro that is used to declare a new mutex that is visible in this file
/// results in a semaphore variable Servo_mutex with value 1 (unlocked)
/// DEFINE_MUTEX_LOCKED() results in a variable with value 0 (locked)

static atomic_t message_to_user;
//---------------------------------------------  ----------------------------------------------
#if PAIR1
static int ServoPeriodOn = 1500;
static int ServoPeriodOff = 18500;
static int ServoIterations = 10;
static const int ServoGpioPin = 17;
static uint32_t pwm_activation_index = 0;
static atomic_t op_start;
#endif

// --------------------------------------------------------------------------------------------------
typedef enum {
    GPIO_INPUT = 0b000,									// 0
    GPIO_OUTPUT = 0b001,								// 1
    GPIO_ALTFUNC5 = 0b010,								// 2
    GPIO_ALTFUNC4 = 0b011,								// 3
    GPIO_ALTFUNC0 = 0b100,								// 4
    GPIO_ALTFUNC1 = 0b101,								// 5
    GPIO_ALTFUNC2 = 0b110,								// 6
    GPIO_ALTFUNC3 = 0b111,								// 7
} GPIOMODE;

// ------ START: GPIO DEFINE -----
#define SOC_PERI_BASE       RPI1_PERI_BASE
#define GPIO_BASE               (SOC_PERI_BASE + 0x200000)    // GPIO controller
#define INP_GPIO(g)   *(gpio.addr + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)   *(gpio.addr + ((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio.addr + (((g)/10))) |= (((a)<=3?(a) + 4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET  *(gpio.addr + 7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR  *(gpio.addr + 10) // clears bits which are 1 ignores bits which are 0
#define GPIO_READ(g)  *(gpio.addr + 13) &= (1<<(g))
struct bcm_peripheral {
    unsigned long addr_p;
    int mem_fd;
    void *map;
    volatile unsigned int *addr;
};

struct bcm_peripheral gpio = {GPIO_BASE};
// ------ END: GPIO DEFINE -----

// --------------------------------------------------------------------------------------------------
static char*   message;           ///< Memory for the string that is passed from userspace
static short  size_of_message;              ///< Used to remember the size of the string stored
static int    majorNumber;                  ///< Stores the device number -- determined automatically
static int    numberOpens = 0;              ///< Counts the number of times the device is opened

#ifndef MAX_COMMAND_SIZE
#define MAX_COMMAND_SIZE 32
#endif
//user command
static char *command = NULL;


// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static struct file_operations fops =
{
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};

static int dev_open(struct inode *inodep, struct file *filep) {
    if(!mutex_trylock(&Servo_mutex)) {   /// Try to acquire the mutex (i.e., put the lock on/down)
        /// returns 1 if successful and 0 if there is contention
        printk(KERN_ALERT "Servo: Device in use by another process");
        return -EBUSY;
    }
    numberOpens++;
    printk(KERN_INFO "Servo: Device has been opened %d time(s)\n", numberOpens);
    return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    int error_count = 0;
    memset(message,0,MAX_MESSAGE_SIZE);
    size_of_message=0;
    if(atomic_read(&message_to_user)==None) {
        printk(KERN_INFO "No Command selected\n");
        sprintf(message,"No Command Selected\n");
    } else if(atomic_read(&message_to_user)==GetInfo) {
        printk(KERN_INFO "Message: getinfo sent\n");
        sprintf(message, "Rpi SG90 servo motor controller\nAuthor: Pratik M Tambe <enthusiasticgeek@gmail.com>\nDate: Dec 25, 2020\nControl/Status Parameters to read/write are available in /sys/devices/virtual/AutomatorServo/Servo/ directory\n");   // appending received string with its length
    }
    size_of_message = strlen(message);                 // store the length of the stored message
    // copy_to_user has the format ( * to, *from, size) and returns 0 on success
    error_count = copy_to_user(buffer, message, size_of_message);

    if (error_count==0) {           // if true then have success
        printk(KERN_INFO "Servo: Sent %d characters to the user\n", size_of_message);
        return (size_of_message=0);  // clear the position to the start and return 0
    }
    else {
        printk(KERN_INFO "Servo: Failed to send %d characters to the user\n", error_count);
        return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
    }
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
    short count;
    memset(command, 0, MAX_COMMAND_SIZE);
    count = copy_from_user(command, buffer, len);

    //reset
    atomic_set(&message_to_user,None);

    if(strncmp(buffer,"getinfo",7)==0) {
        printk(KERN_INFO "Command Request: getinfo");
        atomic_set(&message_to_user,GetInfo);
    } else {
        printk(KERN_ALERT "Command Request: command doesn't exists");
    }

    //sprintf(message, "%s(%zu letters)", buffer, len);   // appending received string with its length
    //size_of_message = strlen(message);                 // store the length of the stored message
    printk(KERN_INFO "Servo: Received %zu characters from the user\n", len);
    return len;
}

static int dev_release(struct inode *inodep, struct file *filep) {
    mutex_unlock(&Servo_mutex);          /// Releases the mutex (i.e., the lock goes up)
    printk(KERN_INFO "Servo: Device successfully closed\n");
    return 0;
}
// --------------------------------------------------------------------------------------------------
#if PAIR1
static ssize_t set_op_start_callback(struct device* dev,
        struct device_attribute* attr,
        const char* buf,
        size_t count)
{
    if((ServoPeriodOn==0)||(ServoPeriodOff==0)) {
        printk("Servo On/Off Period > 0\n");
        return -EINVAL;
    }

    long period_value = 0;
    if (kstrtol(buf, 10, &period_value) < 0)
        return -EINVAL;
    if (period_value != 1)	//Safety check
        return - EINVAL;
    atomic_set(&op_start,true);

    //
    //operation LED  set
    GPIO_SET = 1 << ServoOperationGpioPin;

    pwm_activation_index = 0;
    while(pwm_activation_index < ServoIterations) {
         GPIO_SET = 1 << ServoGpioPin;
         udelay(ServoPeriodOn);
         GPIO_CLR = 1 << ServoGpioPin;
         udelay(ServoPeriodOff);
         pwm_activation_index++;
    }

    //operation LED  clear
    GPIO_CLR = 1 << ServoOperationGpioPin;
    //reset flags
    atomic_set(&op_start,false);

    return count;
}


static ssize_t set_period_on_callback(struct device* dev,
        struct device_attribute* attr,
        const char* buf,
        size_t count)
{

    long period_value = 0;
    if (kstrtol(buf, 10, &period_value) < 0)
        return -EINVAL;

    ServoPeriodOn = period_value;
    return count;
}

static ssize_t set_period_off_callback(struct device* dev,
        struct device_attribute* attr,
        const char* buf,
        size_t count)
{

    long period_value = 0;
    if (kstrtol(buf, 10, &period_value) < 0)
        return -EINVAL;

    ServoPeriodOff = period_value;
    return count;
}

static ssize_t set_op_iterations_callback(struct device* dev,
        struct device_attribute* attr,
        const char* buf,
        size_t count)
{

    if((ServoPeriodOn==0)||(ServoPeriodOff==0)) {
        printk("Set Servo On/Off Period > 0 first\n");
        return -EINVAL;
    }
    long period_value = 0;
    if (kstrtol(buf, 10, &period_value) < 0)
        return -EINVAL;

    ServoIterations = period_value;
    return count;
}



//===========================================================================
//The biggest unsigned 64 bit number you can have is 18,446,744,073,709,551,615 which is 20 digits + 1 NUL terminated char.
//TODD(Pratik) replace snprintf with copy_to_user() function
static ssize_t get_period_callback_on(struct device* dev,
        struct device_attribute* attr,
        char* buf)
{
    return snprintf(buf,21,"%d\n", ServoPeriodOn);
}

static ssize_t get_period_callback_off(struct device* dev,
        struct device_attribute* attr,
        char* buf)
{
    return snprintf(buf,21,"%d\n", ServoPeriodOff);
}

//===========================================================================
//

static DEVICE_ATTR(op_start, S_IWUSR | S_IWGRP , NULL, set_op_start_callback);
static DEVICE_ATTR(period_on, S_IRWXU | S_IRWXG, get_period_callback_on, set_period_on_callback);
static DEVICE_ATTR(period_off, S_IRWXU | S_IRWXG, get_period_callback_off, set_period_off_callback);
static DEVICE_ATTR(op_iterations, S_IWUSR | S_IWGRP , NULL, set_op_iterations_callback);
#endif

//===========================================================================

static struct class *s_pDeviceClass;
static struct device *s_pDeviceObject;

static int __init ServoModule_init(void)
{


    int result;
#if PAIR1
    atomic_set(&op_start,false);
#endif
    // -- GPIO initial
    gpio.map     = ioremap(GPIO_BASE, 4096);
    gpio.addr    = (volatile unsigned int *)gpio.map;

#if PAIR1
    INP_GPIO(ServoOperationGpioPin);
    OUT_GPIO(ServoOperationGpioPin);

    GPIO_CLR = 1 << ServoOperationGpioPin;
#endif

#if PAIR1
    INP_GPIO(ServoGpioPin);
    OUT_GPIO(ServoGpioPin);

    GPIO_SET = 1 << ServoGpioPin;
#endif


    /* Allocating memory for the buffers */
    //Allocate buffers

    message = kmalloc(MAX_MESSAGE_SIZE,  GFP_KERNEL);
    command = kmalloc(MAX_COMMAND_SIZE,  GFP_KERNEL);

#if PAIR1
    if (!message || !command) {
#endif
        // Freeing buffers
        if (message) {
            kfree(message); //Note kfree
        }
        if (command) {
            kfree(command); //Note kfree
        }
        result = -ENOMEM;
        BUG_ON(result < 0);
    }

    //Reset the buffers
    memset(message,0, MAX_MESSAGE_SIZE);
    memset(command,0, MAX_COMMAND_SIZE);

    printk(KERN_INFO "Servo: Initializing the Servo LKM\n");

    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (majorNumber<0) {
        printk(KERN_ALERT "Servo failed to register a major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "Servo: registered correctly with major number %d\n", majorNumber);


    s_pDeviceClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(s_pDeviceClass)) {               // Check for error and clean up if there is
        unregister_chrdev(majorNumber,DEVICE_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(s_pDeviceClass);          // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "Servo: device class registered correctly\n");
    //BUG_ON(IS_ERR(s_pDeviceClass));

    s_pDeviceObject = device_create(s_pDeviceClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(s_pDeviceObject)) {              // Clean up if there is an error
        class_destroy(s_pDeviceClass);           // Repeated code but the alternative is goto statements
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to create the device\n");
        return PTR_ERR(s_pDeviceObject);
    }
    printk(KERN_INFO "Servo: device class created correctly\n"); // Made it! device was initialized
    //s_pDeviceObject = device_create(s_pDeviceClass, NULL, 0, NULL, "Servo");
    //BUG_ON(IS_ERR(s_pDeviceObject));

#if PAIR1
    result = device_create_file(s_pDeviceObject, &dev_attr_op_start);
    BUG_ON(result < 0);
    result = device_create_file(s_pDeviceObject, &dev_attr_period_on);
    BUG_ON(result < 0);
    result = device_create_file(s_pDeviceObject, &dev_attr_period_off);
    BUG_ON(result < 0);
    result = device_create_file(s_pDeviceObject, &dev_attr_op_iterations);
    BUG_ON(result < 0);
#endif

    mutex_init(&Servo_mutex);       /// Initialize the mutex lock dynamically at runtime
    return 0;
}

static void __exit ServoModule_exit(void)
{
    mutex_destroy(&Servo_mutex);        /// destroy the dynamically-allocated mutex

#if PAIR1
    atomic_set(&op_start,false);
#endif

    if (message) {
        kfree(message); //Note kfree
    }
    if (command) {
        kfree(command); //Note kfree
    }


    GPIO_CLR = 1 << ServoOperationGpioPin;
    INP_GPIO(ServoOperationGpioPin);

#if PAIR1
    GPIO_CLR = 1 << ServoGpioPin;

    INP_GPIO(ServoGpioPin);
#endif


    if (gpio.addr) {
        /* release the mapping */
        printk("module gpio: release gpio.addr\n");
        iounmap(gpio.addr);
    }
    //free up device files
#if PAIR1
    device_remove_file(s_pDeviceObject, &dev_attr_op_start);
    device_remove_file(s_pDeviceObject, &dev_attr_period_on);
    device_remove_file(s_pDeviceObject, &dev_attr_period_off);
    device_remove_file(s_pDeviceObject, &dev_attr_op_iterations);
#endif
    device_destroy(s_pDeviceClass, MKDEV(majorNumber, 0));
    class_unregister(s_pDeviceClass);                          // unregister the device class
    unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
    class_destroy(s_pDeviceClass);

}

module_init(ServoModule_init);
module_exit(ServoModule_exit);
