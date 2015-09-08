/*******************************************************************************************
 *  MODULE: ebx_monitor.c
 *
 *  DESCRIPTION:
 *    This module implements a  mechanism to collect time stamps (measure points)
 *    in the Linux kernel using ktime. The <stuct MeasurePointS> defines one measure
 *    point which includes an <id> (the start ktime value), the <timestamp> and
 *    a user defined tag value, e.g. to identify specific measure points in the code.
 *    The kernel module initialisation allocates an array of mesaure points depending
 *    on the module parameter <nbrMeasurementPoints> and the fifo to allow a user space
 *    application to read finished measurement results.
 *    To reduce the negative impact of the measurement the only protection to access
 *    the array of measure points is the atomic measure point array index.
 *    A user space application can read the device file in blocking or non-blocking mode,
 *    e.g. cat cat /dev/ebx_monitor.
 *
 *    If the kernel module is loaded without the parameter <nbrMeasurementPoints> (or the
 *    value set to 0), the easy measure mode is activated. In this mode no measure point
 *    array and no fifo is allocated, instead the device attributes are updated, e.g.
 *    cat /sys/devices/virtual/misc/ebx_monitor/averageFrameTime.
 *    
 *  DEVICE:
 *    ls -l /dev/ebx_monitor
 *
 *  DEVICE ATTRIBUTES:
 *    ls -l /sys/devices/virtual/misc/ebx_monitor
 *
 *
 *  AUTHOR: Michel Grundmann
 *
 *  HISTORY:
 *    Date      Author  Description
 *    20150908  mg      Added a "tag" to the measure point to identify the individual measure points
 *
 */


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

#include <linux/string.h> /* strlen */
#include <linux/device.h>

#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/timekeeping.h>

#include <asm/atomic.h>

#include <linux/kfifo.h>   /* use kernel fifo */


/* --- Macros and Const definition --- */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michel Grundmann");
MODULE_DESCRIPTION("EBX Monitor");

#define CHARDEV_NAME "ebx_monitor"

#define USE_TIMER_FOR_FRAME_SIMULATION 0


/* --- Type definition --- */

struct MeasurePointS {
  ktime_t id;
  ktime_t timeStamp;
  unsigned char tag;
};
static struct MeasurePointS* measureP = NULL;
static atomic_t measureIdx_atomic     = ATOMIC_INIT(-1);
static int measurementIdxMax          = 0;
static int nbrOfCharactersPerMeasurePointMax = 50; /* <19-char>us, <19-char>us, tag\n  = 50 */

DECLARE_KFIFO_PTR(ourCharDevFifo, char);


/* --- Variable definition --- */

static int ebx_monitor_open(struct inode* inodeP, struct file* fileP);
static int ebx_monitor_release(struct inode* inodeP, struct file* fileP);
static ssize_t ebx_monitor_read(struct file *fileP, char __user *buf, size_t count, loff_t *f_pos);

static struct file_operations cdev_fops = {
  .owner   = THIS_MODULE,
  .open    = ebx_monitor_open,
  .release = ebx_monitor_release,
  .read    = ebx_monitor_read,
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

static atomic_t totalFrameCount_atomic  = ATOMIC_INIT(0);
static atomic_t totalFrameDelta_atomic  = ATOMIC_INIT(0);
static atomic_t lastFrameDelta_atomic   = ATOMIC_INIT(0);
static atomic_t averageFrameTime_atomic = ATOMIC_INIT(0);
static atomic_t readOneShot_atomic      = ATOMIC_INIT(0);

#if USE_TIMER_FOR_FRAME_SIMULATION
#include <linux/timer.h>   /* *_timer_* */
static struct timer_list ourTimer;
#endif

/* Device data structure */
static struct mutex      ebx_monitor_lock;  /* mutex for access exclusion */
static wait_queue_head_t ebx_monitor_readq; /* read queue for blocking */
static ssize_t           ebx_monitor_count; /* number of bytes to be read */

/* DEBUG START */
static bool message_read; /* just for testing using cat */
/* DEBUG END */
static int nbrMeasurementPoints = 0;
module_param(nbrMeasurementPoints, uint, S_IRUGO);
MODULE_PARM_DESC(nbrMeasurementPoints, "The number of measurement points in the kernel (default=0)");

static int fifoMax = 1024; /* TODO: calculate fifoMax in init, depends on number of "measure points" */


/* --- Function Prototypes --- */
static int ebx_monitor_createDeviceAttributeFiles(struct device* inDeviceP);
static void ebx_monitor_measurementsFinished(void);
static void ebx_monitor_gotnewframe_real(const struct timeval* inTimeOfNewFrameP);
static void ebx_monitor_gotnewframe_dummy(const struct timeval* inTimeOfNewFrameP);
static void ebx_monitor_gotnewframe_easy(const struct timeval* inTimeOfNewFrameP);
static void (*ebx_monitor_gotnewframe_funP) (const struct timeval* inTimeOfNewFrameP);

static void ebx_monitor_gotframe_real(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag);
static void ebx_monitor_gotframe_dummy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag);
static void ebx_monitor_gotframe_easy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag);
static void (*ebx_monitor_gotframe_funP) (const struct timeval* inTimeOfNewFrameP, const unsigned char inTag);

static void ebx_monitor_writeDataToFifo(void);

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
  if (measureP != NULL) {
    kfree(measureP);
  }

  if (nbrMeasurementPoints > 0) {
    /* free memory used for the kernel fifo */
    printk(KERN_INFO CHARDEV_NAME": Free kfifo memory\n");
    kfifo_free(&ourCharDevFifo);
  }
} /* ebx_monitor_exit */


int
__init ebx_monitor_init(void)
{
  int result = 0;
  unsigned int measurePointSize = sizeof(struct MeasurePointS);

  printk(KERN_INFO CHARDEV_NAME": Initialise\n");

  /* Initialise the measurement function pointers */
  if (nbrMeasurementPoints == 0) {
    ebx_monitor_gotnewframe_funP = ebx_monitor_gotnewframe_easy; 
    ebx_monitor_gotframe_funP    = ebx_monitor_gotframe_easy;
  } else {
    ebx_monitor_gotnewframe_funP = ebx_monitor_gotnewframe_real; 
    ebx_monitor_gotframe_funP    = ebx_monitor_gotframe_real;
  }  

  if (nbrMeasurementPoints > 0) {
    measureP = (struct MeasurePointS*)(kcalloc(nbrMeasurementPoints, sizeof(struct MeasurePointS), GFP_KERNEL));
    if (measureP == NULL) {
      result = -ENOMEM;
      goto fail1;
    }
    measurementIdxMax = nbrMeasurementPoints - 1; /* -1 because the index starts with zero */
  }
  printk(KERN_INFO CHARDEV_NAME": sizeof((struct MeasurePointS) = %d, measurementIdxMax = %d\n",
	 measurePointSize,
	 measurementIdxMax);


  /* Initialise wait queue and mutex */
  init_waitqueue_head(&ebx_monitor_readq);
  mutex_init(&ebx_monitor_lock);

  /* Register misc device */
  if ((result = misc_register(&ebx_monitor_misc))) {
    printk(KERN_WARNING CHARDEV_NAME": error registering!\n");
    goto fail2;
  }

  /* --- allocate memory for the kernel fifo --- */
  /* allocate memory only if some measurement points are configured */
  if (nbrMeasurementPoints > 0) {
    printk(KERN_INFO CHARDEV_NAME": Allocate kfifo memory\n");
    result = kfifo_alloc(&ourCharDevFifo, nbrMeasurementPoints * nbrOfCharactersPerMeasurePointMax, GFP_KERNEL);
    if (result != 0) {
      printk(KERN_ERR CHARDEV_NAME": Error kfifo_alloc failed, result = %d\n", result);
      goto fail3;
    }
    /* read the fifo size, as the size will be rounded-up to a power of 2 */
    fifoMax = kfifo_size(&ourCharDevFifo);
    printk(KERN_ERR CHARDEV_NAME": kfifo_size = %d\n", fifoMax);
  }

  /* --- create device attribute files --- */
  result = ebx_monitor_createDeviceAttributeFiles(ebx_monitor_misc.this_device);
  if (result != 0) {
    goto fail4;
  }

#if USE_TIMER_FOR_FRAME_SIMULATION
  ebx_monitor_initTimer();
#endif

  /* DEBUG START */
  ebx_monitor_count = 0;
  /* DEBUG END */

  return result;


 fail4:
  if (nbrMeasurementPoints > 0) {
    kfifo_free(&ourCharDevFifo);
  }
 fail3:
  misc_deregister(&ebx_monitor_misc);
 fail2:
  if (measureP != NULL) {
    kfree(measureP);
  }
 fail1:
  return result;
} /* ebx_monitor_init */


void
ebx_monitor_gotnewframe(const struct timeval* inTimeOfNewFrameP)
{
  ebx_monitor_gotnewframe_funP(inTimeOfNewFrameP);
} /* ebx_monitor_gotnewframe */
EXPORT_SYMBOL_GPL(ebx_monitor_gotnewframe);


void
ebx_monitor_gotframe(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag)
{
  ebx_monitor_gotframe_funP(inTimeOfNewFrameP, inTag);
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


ssize_t
ebx_monitor_readOneShot_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  int readOneShot = atomic_read(&readOneShot_atomic);

  if (readOneShot == 0) {
    snprintf(buf, 256, "Read one shot is deactivated\n");
  } else {
    snprintf(buf, 256, "Read one shot is activated\n");
  }

  return (strlen(buf)+1);
} /* ebx_monitor_readOneShot_show */


static ssize_t
ebx_monitor_readOneShot_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
  unsigned long tmp = 0;
  int ret = kstrtoul(buf, (unsigned int)0, &tmp);
  
  if (ret != 0) {
    /* something is wrong */
    tmp = 0;
  }

  atomic_set(&readOneShot_atomic, (int)tmp);

  /* DEBUG OUTPUT */
  if (tmp == 0) {
    printk(KERN_INFO CHARDEV_NAME": One shot is deactivated\n");
  } else {
    printk(KERN_INFO CHARDEV_NAME": One shot is activated\n");
  }

  return strnlen(buf, count);
}
DEVICE_ATTR(readOneShot, 0444, ebx_monitor_readOneShot_show, ebx_monitor_readOneShot_store);



/* --- STATIC Funtions --- */
static int
ebx_monitor_open(struct inode* inodeP, struct file* fileP)
{
  (void)nonseekable_open(inodeP, fileP);

  /* DEBUG START */
  message_read = false; /* just for testing using cat */
  /* DEBUG END */

  return 0;
} /* ebx_monitor_open */


static int
ebx_monitor_release(struct inode* inodeP, struct file* fileP)
{
  (void)inodeP;
  (void)fileP;

  return 0;
} /* ebx_monitor_release */


static ssize_t
ebx_monitor_read(struct file *fileP, char __user *buf, size_t count, loff_t *f_pos)
{
  int ret = 0;
  unsigned int copied = 0;

  /* TEST: just activate to simulate partial read access: count = 10; */

  /* DEBUG OUTPUT */
  printk(KERN_INFO CHARDEV_NAME": read by \"%s\" (pid %i)\n",
	 current->comm,
	 current->pid);

  /* Get lock or signal restart if interrupted */
  if (mutex_lock_interruptible(&ebx_monitor_lock)) return -ERESTARTSYS;

#if 0 /* Partial read is supported !!! */
  /* Partial reads are unsupported */
  if (ebx_monitor_count > count) {
    ret = -EINVAL;
    goto err;
  }
#endif

  /* The default from 'cat' is to issue multiple reads readOneShot_atomic avoids that */
  if (atomic_read(&readOneShot_atomic) && message_read) {
    mutex_unlock(&ebx_monitor_lock);
    return 0; /* just for testing using cat */
  }

  /* Return EOF if no data avalable to read */
  if ((ebx_monitor_count == 0) && (nbrMeasurementPoints == 0)) {
    /* no data available to read */
    printk(KERN_INFO CHARDEV_NAME": ... no data available (running in easy mode, use sys-fs to retrieve data)\n");
    goto err;
  }

  /* DEBUG OUTPUT */
  if (fileP->f_flags & O_NONBLOCK) {
    printk(KERN_INFO CHARDEV_NAME": read access NON blocking\n");
  } else {
    printk(KERN_INFO CHARDEV_NAME": read access blocking\n");
  }

  /* Return -EAGAIN if user requested O_NONBLOCK and no data available */
  if ((ebx_monitor_count == 0) && (fileP->f_flags & O_NONBLOCK)) {
    ret = -EAGAIN;
    goto err;
  }

  if (!(fileP->f_flags & O_NONBLOCK)) {
    printk(KERN_INFO CHARDEV_NAME": process blocking read access\n");
    /* Blocking read in progress */
    /* The loop is necessary, because wait_event tests are not synchronized */
    while (ebx_monitor_count == 0) {
      if (nbrMeasurementPoints > 0) {
	printk(KERN_INFO CHARDEV_NAME": ... no data available, set functions to real\n");
	/* TODO: is special protection needed to set the function pointers? ....spin lock? */
	ebx_monitor_gotnewframe_funP = ebx_monitor_gotnewframe_real; 
	ebx_monitor_gotframe_funP    = ebx_monitor_gotframe_real;
      }

      /* Release lock */
      mutex_unlock(&ebx_monitor_lock);

      /* Wait on read queue, signal restart if interrupted */
      if (wait_event_interruptible(ebx_monitor_readq, (ebx_monitor_count > 0))) {
	return -ERESTARTSYS;
      }

      /* Get lock again */
      if (mutex_lock_interruptible(&ebx_monitor_lock)) {
	return -ERESTARTSYS;
      }
    } /* while */
  } /* if (!(fileP->f_flags & O_NONBLOCK)) */

  /* Copy data to user */
  ret = kfifo_to_user(&ourCharDevFifo, buf, count, &copied);
  printk(KERN_INFO CHARDEV_NAME": read(): kfifo_to_user copied %d characters, kfifo_len is now %u\n",
	 copied,
	 kfifo_len(&ourCharDevFifo));

  /* DEBUG: just to test nonblocking access using cat */
  message_read = true; /* just for testing using cat */
  atomic_set(&measureIdx_atomic, -1);

  /* Save state */
  ret = copied;
  *f_pos += ret;

  /* Keep on track with delivered data */
  ebx_monitor_count = ebx_monitor_count - copied;

  /* Unlock and signal writers */
  mutex_unlock(&ebx_monitor_lock);

  //TODO...trigger possible next statistic...  wake_up_interruptible(&ebx_monitor_writeq);
  return ret ? ret : copied; /* return ret; */

 err:
  mutex_unlock(&ebx_monitor_lock);
  return ret;
} /* ebx_monitor_read */


static void
ebx_monitor_measurementsFinished(void)
{
  printk(KERN_INFO CHARDEV_NAME": ... data available, set functions to dummy\n");
  /* TODO: is special protection needed to set the function pointers? ....spin lock? */
  ebx_monitor_gotnewframe_funP = ebx_monitor_gotnewframe_dummy; 
  ebx_monitor_gotframe_funP    = ebx_monitor_gotframe_dummy;
  ebx_monitor_writeDataToFifo();
  wake_up_interruptible(&ebx_monitor_readq);
} /* ebx_monitor_measurementsFinished */


static void
ebx_monitor_writeDataToFifo(void)
{
  int idx           = 0;
  ssize_t strLength = 0;
  char* bufP        = kmalloc(100, GFP_KERNEL);

  for (idx = 0; idx <= measurementIdxMax; idx++) {
    snprintf(bufP, 100, "%lldus, %lldus, %u\n",
	     ktime_to_us(measureP[idx].id),
	     ktime_to_us(measureP[idx].timeStamp),
	     measureP[idx].tag);
    printk(KERN_INFO CHARDEV_NAME": %s\n", bufP);
    strLength = strlen(bufP) + 1;
    if (strLength != kfifo_in(&ourCharDevFifo, bufP, strLength)) {
      printk(KERN_INFO CHARDEV_NAME": ERROR kfifo_in failed\n");
    }
  }
  ebx_monitor_count = kfifo_len(&ourCharDevFifo);
  printk(KERN_INFO CHARDEV_NAME": kfifo_len = %u\n", ebx_monitor_count);
} /* ebx_monitor_writeDataToFifo */


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

  result = device_create_file(inDeviceP, &dev_attr_readOneShot);
  if (result != 0) {
    goto fail;
  }

  printk(KERN_INFO "chardev: result of device_create_file: %d\n", result);
  return result;

 fail:
  printk(KERN_WARNING CHARDEV_NAME": error in device_create_file\n");
  return result;
} /* ebx_monitor_createDeviceAttributeFiles */


static void
ebx_monitor_gotnewframe_dummy(const struct timeval* inTimeOfNewFrameP)
{
  /* printk(KERN_INFO CHARDEV_NAME": gotnewframe_dummy() called\n"); */
} /* ebx_monitor_gotnewframe_dummy */

static void
ebx_monitor_gotnewframe_easy(const struct timeval* inTimeOfNewFrameP)
{
  /* calculation has a negative influence on performance !!! */
  ktime_t newFrame_ktime = timeval_to_ktime(*inTimeOfNewFrameP);

  if (prevFrame_ktime.tv64 != 0) {
    ktime_t delta = ktime_sub(newFrame_ktime, prevFrame_ktime);

    atomic_set(&lastFrameDelta_atomic, (int)ktime_to_us(delta));
    atomic_add((int)ktime_to_us(delta), &totalFrameDelta_atomic); 
    atomic_inc(&totalFrameCount_atomic);
    atomic_set(&averageFrameTime_atomic, (atomic_read(&totalFrameDelta_atomic) / atomic_read(&totalFrameCount_atomic)));
  }

  prevFrame_ktime = newFrame_ktime;
} /* ebx_monitor_gotnewframe_easy */


static void
ebx_monitor_gotnewframe_real(const struct timeval* inTimeOfNewFrameP)
{
  int idx = -1;

  ktime_t newFrame_ktime = timeval_to_ktime(*inTimeOfNewFrameP);

  idx = atomic_add_return(1, &measureIdx_atomic);
  if ((idx >= 0) && (idx <= measurementIdxMax)) {
    printk(KERN_INFO CHARDEV_NAME": gotnewframe_real(), idx = %i, measurementIdxMax = %d\n", idx, measurementIdxMax);
    measureP[idx].id        = newFrame_ktime;
    measureP[idx].timeStamp = newFrame_ktime;
    measureP[idx].tag       = 0; /* the tag for a new frame is always zero */
  } else {
    ebx_monitor_measurementsFinished();
  }

  prevFrame_ktime = newFrame_ktime;
} /* ebx_monitor_gotnewframe_real */


static void
ebx_monitor_gotframe_dummy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag)
{
  /* printk(KERN_INFO CHARDEV_NAME": gotframe_dummy() called\n"); */
} /* ebx_monitor_gotframe_dummy */


void
ebx_monitor_gotframe_easy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag)
{
  /* printk(KERN_INFO CHARDEV_NAME": gotframe_easy() called\n"); */
} /* ebx_monitor_gotframe_easy */


void
ebx_monitor_gotframe_real(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag)
{
  int idx = -1;

  ktime_t newFrame_ktime = timeval_to_ktime(*inTimeOfNewFrameP);
  ktime_t frame_ktime    = ktime_get();

  idx = atomic_add_return(1, &measureIdx_atomic);
  if ((idx >= 0) && (idx <= measurementIdxMax)) {
    printk(KERN_INFO CHARDEV_NAME": gotframe_real(), idx = %i, measurementIdxMax = %d\n", idx, measurementIdxMax);
    measureP[idx].id        = newFrame_ktime;
    measureP[idx].timeStamp = frame_ktime;
    measureP[idx].tag       = inTag;
  } else {
    ebx_monitor_measurementsFinished();
  }
} /* ebx_monitor_gotframe_real */


#if USE_TIMER_FOR_FRAME_SIMULATION
static void
ebx_monitor_timer_expired(unsigned long inData)
{
  static unsigned char ourTag = 0;
  struct timeval newFrameTime;
  do_gettimeofday(&newFrameTime);

  ebx_monitor_gotnewframe_funP(&newFrameTime);

  ourTimer.expires = get_jiffies_64() + (HZ/30); /* about 33ms = 30fps */
  add_timer(&ourTimer);

  ebx_monitor_gotframe_funP(&newFrameTime, ++ourTag);
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
