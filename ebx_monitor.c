/* --- Includes --- */
#include <linux/init.h>    /* Funktionseigenschaften */
#include <linux/module.h>  /* Modulinfrastruktur */
#include <linux/kernel.h>  /* Kernel-Library */
#include <linux/cdev.h>    /* character devices */
#include <linux/fs.h>      /* character devices */
#include <linux/slab.h>    /* kzalloc */
#include <asm/uaccess.h>   /* copy_from_user */
#include <asm/current.h>   /* current is #define current (get_current())) */
#include <linux/sched.h>   /* struct task_struct */
#include <linux/miscdevice.h> /* struct miscdevice */

#include <linux/device.h>

#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/timekeeping.h>

#include <asm/atomic.h>


/* --- Macros and Const definition --- */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michel Grundmann");
MODULE_DESCRIPTION("EBX Monitor");

#define CHARDEV_NAME "ebx_monitor"

#define USE_TIMER_FOR_FRAME_SIMULATION 0


/* --- Type definition --- */


/* --- Variable definition --- */

static int ebx_monitor_open(struct inode* inodeP, struct file* fileP);
static int ebx_monitor_release(struct inode* inodeP, struct file* fileP);

static struct file_operations cdev_fops = {
  .owner   = THIS_MODULE,
  .open    = ebx_monitor_open,
  .release = ebx_monitor_release,
  .read    = NULL,
  .write   = NULL,
  .llseek  = no_llseek
};

static struct miscdevice ebx_monitor_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = CHARDEV_NAME,
  .fops  = &cdev_fops,
  .mode  = 00444
};

static ktime_t prevFrame_ktime;
static ktime_t newFrame_ktime;
static ktime_t frame_ktime;

static atomic_t totalFrameCount_atomic  = ATOMIC_INIT(0);
static atomic_t totalFrameDelta_atomic  = ATOMIC_INIT(0);
static atomic_t lastFrameDelta_atomic   = ATOMIC_INIT(0);
static atomic_t averageFrameTime_atomic = ATOMIC_INIT(0);

#if USE_TIMER_FOR_FRAME_SIMULATION
#include <linux/timer.h>   /* *_timer_* */
static struct timer_list ourTimer;
#endif


/* --- Function Prototypes --- */
static int ebx_monitor_createDeviceAttributeFiles(struct device* inDeviceP);

#if USE_TIMER_FOR_FRAME_SIMULATION
static void ebx_monitor_timer_expired(unsigned long inData);
static void ebx_monitor_initTimer(void);
#endif


/* --- GLOBAL Funtions --- */
void
__exit ebx_monitor_exit(void)
{
  printk(KERN_INFO CHARDEV_NAME": Cleanup and exit\n");

#if USE_TIMER_FOR_FRAME_SIMULATION
  printk(KERN_INFO CHARDEV_NAME": Delete timer\n");
  del_timer_sync(&ourTimer);
#endif

  misc_deregister(&ebx_monitor_misc);
} /* ebx_monitor_exit */


int
__init ebx_monitor_init(void)
{
  int result = 0;

  printk(KERN_INFO CHARDEV_NAME": Initialize\n");


  if ((result = misc_register(&ebx_monitor_misc))) {
    printk(KERN_WARNING CHARDEV_NAME": error registering!\n");
    goto fail;
  }

  /* --- create device attribute files --- */
  result = ebx_monitor_createDeviceAttributeFiles(ebx_monitor_misc.this_device);

#if USE_TIMER_FOR_FRAME_SIMULATION
  ebx_monitor_initTimer();
#endif

  return result;

 fail:
  return result;
} /* ebx_monitor_init */


void
ebx_monitor_gotnewframe(const struct timeval* inTimeOfNewFrameP)
{
  // TODO: calculation should NOT be done here!!!
  newFrame_ktime = timeval_to_ktime(*inTimeOfNewFrameP);
  if (prevFrame_ktime.tv64 != 0) {
    ktime_t delta = ktime_sub(newFrame_ktime, prevFrame_ktime);

    atomic_set(&lastFrameDelta_atomic, (int)ktime_to_us(delta));
    atomic_add((int)ktime_to_us(delta), &totalFrameDelta_atomic); 
    atomic_inc(&totalFrameCount_atomic);
    atomic_set(&averageFrameTime_atomic, (atomic_read(&totalFrameDelta_atomic) / atomic_read(&totalFrameCount_atomic)));
  }

  prevFrame_ktime = newFrame_ktime;
} /* ebx_monitor_gotnewframe */
EXPORT_SYMBOL_GPL(ebx_monitor_gotnewframe);


void
ebx_monitor_gotframe(void)
{
 frame_ktime = ktime_get();
} /* ebx_monitor_gotframe */
EXPORT_SYMBOL_GPL(ebx_monitor_gotframe);


static int totalFrameCount = 0;
ssize_t
ebx_monitor_totalFrameCount_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  totalFrameCount = atomic_read(&totalFrameCount_atomic);
  snprintf(buf, 256, "Total frame count = %i\n", totalFrameCount);
  return (strlen(buf)+1);
} /* ebx_monitor_totalFrameCount_show */
DEVICE_ATTR(totalFrameCount, 0444, ebx_monitor_totalFrameCount_show, NULL);


static int totalFrameDelta = 0;
ssize_t
ebx_monitor_totalFrameDelta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  totalFrameDelta = atomic_read(&totalFrameDelta_atomic);
  snprintf(buf, 256, "Total frame delta = %ius\n", totalFrameDelta);
  return (strlen(buf)+1);
} /* ebx_monitor_totalFrameDelta_show */
DEVICE_ATTR(totalFrameDelta, 0444, ebx_monitor_totalFrameDelta_show, NULL);


static int lastFrameDelta = 0;
ssize_t
ebx_monitor_lastFrameDelta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  lastFrameDelta = atomic_read(&lastFrameDelta_atomic);
  snprintf(buf, 256, "Total frame delta = %ius\n", lastFrameDelta);
  return (strlen(buf)+1);
} /* ebx_monitor_lastFrameDelta_show */
DEVICE_ATTR(lastFrameDelta, 0444, ebx_monitor_lastFrameDelta_show, NULL);


static int averageFrameTime = 0;
ssize_t
ebx_monitor_averageFrameTime_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  averageFrameTime = atomic_read(&averageFrameTime_atomic);
  snprintf(buf, 256, "Average frame time = %ius\n", averageFrameTime);
  return (strlen(buf)+1);
} /* ebx_monitor_averageTime_show */
DEVICE_ATTR(averageFrameTime, 0444, ebx_monitor_averageFrameTime_show, NULL);



/* --- STATIC Funtions --- */
static int
ebx_monitor_open(struct inode* inodeP, struct file* fileP)
{
  (void)nonseekable_open(inodeP, fileP);

  return 0;
} /* ebx_monitor_open */


static int
ebx_monitor_release(struct inode* inodeP, struct file* fileP)
{
  (void)inodeP;
  (void)fileP;

  return 0;
} /* ebx_monitor_release */


static int
ebx_monitor_createDeviceAttributeFiles(struct device* inDeviceP)
{
  int result = 0;

  result = device_create_file(inDeviceP, &dev_attr_totalFrameCount);
  if (result != 0) {
    goto fail;
  }

  result = device_create_file(inDeviceP, &dev_attr_totalFrameDelta);
  if (result != 0) {
    goto fail;
  }

  result = device_create_file(inDeviceP, &dev_attr_lastFrameDelta);
  if (result != 0) {
    goto fail;
  }

  result = device_create_file(inDeviceP, &dev_attr_averageFrameTime);
  if (result != 0) {
    goto fail;
  }

  printk(KERN_INFO "chardev: result of device_create_file: %d\n", result);
  return result;

 fail:
  printk(KERN_WARNING CHARDEV_NAME": error in device_create_file\n");
  return result;
} /* ebx_monitor_createDeviceAttributeFiles */


#if USE_TIMER_FOR_FRAME_SIMULATION
static void
ebx_monitor_timer_expired(unsigned long inData)
{
  struct timeval newFrameTime;
  do_gettimeofday(&newFrameTime);

  ebx_monitor_gotnewframe(&newFrameTime);

  ourTimer.expires = get_jiffies_64() + (HZ/30); /* about 33ms = 30fps */
  add_timer(&ourTimer);
} /* ebx_monitor_timer_expired */

static void
ebx_monitor_initTimer(void)
{
  printk(KERN_INFO CHARDEV_NAME": Configure timer\n");
  init_timer(&ourTimer);
  ourTimer.function = ebx_monitor_timer_expired;
  ourTimer.data     = 27;
  ourTimer.expires  = get_jiffies_64() + (HZ/30); /* about 33ms = 30fps */
  printk(KERN_INFO CHARDEV_NAME": Start timer\n");
  add_timer(&ourTimer);
} /* ebx_monitor_initTimer */
#endif /* USE_TIMER_FOR_FRAME_SIMULATION */


module_init(ebx_monitor_init);
module_exit(ebx_monitor_exit);
