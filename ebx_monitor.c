/*******************************************************************************************
 *  MODULE: ebx_monitor.c
 *
 *  DESCRIPTION:
 *    This module implements a mechanism to collect time stamps (measure points)
 *    in the Linux kernel using ktime. The <stuct MeasurePointS> defines one measure
 *    point which includes an <id> (the start ktime value), the <timestamp> and
 *    a user defined tag value, e.g. to identify specific measure points in the code.
 *    The kernel module initialisation allocates an array of mesaure points depending
 *    on the module parameter <nbrOfMeasurementPoints> and the fifo to allow a user space
 *    application to read finished measurement results.
 *    To reduce the negative impact of the measurement all the memory needed to perform
 *    a measurement and provide the data to the user space application is allocated during
 *    module initialisation, and the only protection to access the array of measure points
 *    is the atomic measure point array index (measureIdx_atomic).
 *    A user space application can read the device file in blocking or non-blocking mode,
 *    e.g. cat cat /dev/ebx_monitor.
 *    A measurement can explicitly be (re)started by writing the command "start" to the
 *    device, e.g. echo start > /dev/ebx_monitor (the string just has to begin with "start").
 *
 *    If the kernel module is loaded without the parameter <nbrOfMeasurementPoints> (or the
 *    value set to 0), the easy measure mode is activated. In this mode no measure point
 *    array and no fifo is allocated, instead the device attributes are updated, e.g.
 *    cat /sys/devices/virtual/misc/ebx_monitor/averageFrameTime.
 *
 *    The interface to the monitor, and therefore to perform measures, are the following
 *    exported functions:
 *      - ebx_monitor_gotnewframe, to be called on the first measure point
 *      - ebx_monitor_gotframe, to be called on following measure points
 *
 *    The active measurement functionality depends on the following function pointers:
 *      - ebx_monitor_gotnewframe_funP
 *      - ebx_monitor_gotframe_funP
 *
 *    As soon as a mesurement has been finished the function pointers are set to
 *    ebx_monitor_gotnewframe_dummy() and ebx_monitor_gotframe_dummy() and no further
 *    mesurement is done until the data has been read (data are protected in this state).
 *
 *    As soon as the data has been read from the user space application (blocking /
 *    non-blocking read) the function pointers will be set to the appropriate
 *    measurement functions and the next measurement starts.
 *
 *  MODULE PARAMETERS:
 *    - nbrOfMeasurementPoints: The number of measurement points in the kernel (default=0, maximum=1000) (uint)
 *      Defines the number of mesure points, therefore the size of the measueremnt point array
 *      and the size of the fifo for the user space interface.
 *    - deferredFifo: Deferred fifo availability (default=N) (bool)
 *      After a successful measurement the data have to be put into the fifo to be read by the
 *      user space application. By default this is done directly after the last measurement, as
 *      part of the function ebx_monitor_gotnewframe() or ebx_monitor_gotframe().
 *      When <deferredFifo> is set to 'Y' (or '1') this task is delegated to work-queue, to
 *      reduce the negative impact on the running video stream.
 *
 *  MODULE COMMANDS:
 *    Commands can be written to the module by a simple write access, e.g. echo start > /dev/ebx_monitor.
 *    Supported commands:
 *      - start: (re)start a measurement, data ready to be retrieved will be lost.
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
 *    Date      Author  Version  Description
 *    20150908  mg               Added a "tag" to the measure point to identify the individual measure points
 *    20150909  mg               Workqueue to write measure point data from the array to kfifo
 *                               The behavior of writing the data to the kfifo directly after the finished
 *                               measure, or deferred (in the workqueue function), can be configured with the
 *                               module parameter <deferredFifo>.
 *    20150914  mg               Description updated.
 *                               Number of measure points limited to 1000.
 *                               Module version added.
 *                               Added information about memory usage of the monitor.
 *    20150918  mg      1.2      Mode function pointer protected by spinlock.
 *                               Only create device attribute files needed for the measurement mode.
 *                               Remove device attribute files on exit.
 *    20150918  mg      1.3      Performance improvement: Debug output reduced, see precompiler switch EBX_DEBUG_OUTPUT.
 *                               Non blocking read access corrected.
 *    20150926  mg      1.4      Write access to start new measurement implemented, e.g. echo start > /dev/ebx_monitor
 *    20150926  mg      1.5      Corrected device file access attributes.
 *                               Issue with "start" command solved.
 *    20150926  mg      1.6      The new frame has now a real timestamp as well and uses no longer the "id" as timestamp.
 *                               This decision was made because we have the tag which identifies the new frame (tag=0).
 *    20150926  mg      1.7      Naming convention correction.
 *    20150926  mg      1.8      STATUS and DEBUG output removed (precompiler-switch).
 *                               Description updated.
 *
 *
 */


#define EBX_MONITOR_VERSION "1.8"


/* --- Includes --- */
#include <linux/init.h>       /* Funktionseigenschaften */
#include <linux/module.h>     /* Modulinfrastruktur */
#include <linux/kernel.h>     /* Kernel-Library */
#include <linux/cdev.h>       /* character devices */
#include <linux/fs.h>         /* character devices */
#include <linux/slab.h>       /* kzalloc */
#include <asm/uaccess.h>      /* copy_from_user */
#include <asm/current.h>      /* current is #define current (get_current())) */
#include <linux/sched.h>      /* struct task_struct */
#include <linux/miscdevice.h> /* struct miscdevice */

#include <linux/string.h>     /* strlen */
#include <linux/device.h>

#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/timekeeping.h>

#include <asm/atomic.h>

#include <linux/kfifo.h>      /* use kernel fifo */

#include <linux/spinlock.h>


/* --- Macros and Const definition --- */
MODULE_VERSION(EBX_MONITOR_VERSION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michel Grundmann");
MODULE_DESCRIPTION("EBX Monitor to measure execution time in the kernel");

/* The name of the driver */
#define CHARDEV_NAME "ebx_monitor"

/* EBX_STATUS_OUTPUT:
   Set to 1 to get vital debug status information */
#define EBX_STATUS_OUTPUT 0
/* EBX_DEBUG_OUTPUT:
   Set to 1 to get additional, detailed debug information*/
#define EBX_DEBUG_OUTPUT 0
/* USE_TIMER_FOR_FRAME_SIMULATION:
   Used to simulate frames using a timer. To test without a camera. */
#define USE_TIMER_FOR_FRAME_SIMULATION 0


/* --- Type definition --- */
/* Defines a measure point */
struct MeasurePointS {
  ktime_t id;
  ktime_t timeStamp;
  unsigned char tag;
};

/* Defines the possible measure modes. */
typedef enum {
  MonitorModeDummy, /* measure finished, data are protected */
  MonitorModeEasy,  /* no measure points, check devive attribute files */
  MonitorModeReal,  /* configured measure points, the real measure mode */
  MonitorModeUndef
} MonitorModeEnum;


/* --- Variable definition --- */
static int ebx_monitor_open(struct inode* inodeP, struct file* fileP);
static int ebx_monitor_release(struct inode* inodeP, struct file* fileP);
static ssize_t ebx_monitor_read(struct file *fileP, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t ebx_monitor_write(struct file * fileP, const char __user *buf, size_t count, loff_t *f_pos);

static struct file_operations cdev_fops = {
  .owner   = THIS_MODULE,
  .open    = ebx_monitor_open,
  .release = ebx_monitor_release,
  .read    = ebx_monitor_read,
  .write   = ebx_monitor_write,
  .llseek  = no_llseek
};

static struct miscdevice ebx_monitor_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = CHARDEV_NAME,
  .fops  = &cdev_fops,
  .mode  = 00666 // rw access for "ugo"
};


/* Declare the kfifo, used fo the user space API. */
DECLARE_KFIFO_PTR(ourCharDevFifo, char);


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

/* The measure points array */
static struct MeasurePointS* measureP = NULL;
/* The atomic index to the array of measure points. */
static atomic_t measureIdx_atomic     = ATOMIC_INIT(-1);
/* The maximum index of configured measure points (see module parameter). */
static int measurementIdxMax          = 0;
/* The maximum number of characters per measure point */
static int nbrOfCharactersPerMeasurePointMax = 50; /* <19-char>us, <19-char>us, tag\n  = 50 */


/* Device data structure */
static struct mutex      ebx_monitor_lock;  /* mutex for access exclusion */
static wait_queue_head_t ebx_monitor_readq; /* read queue for blocking */
static ssize_t           ebx_monitor_count; /* number of bytes to be read */

static bool message_read; /* just for testing using cat */

static int MaxNbrOfMeasurementPoints = 1000;
static int nbrOfMeasurementPoints = 0;
module_param(nbrOfMeasurementPoints, uint, S_IRUGO);
MODULE_PARM_DESC(nbrOfMeasurementPoints, " The number of measure points in the kernel (default=0, maximum=1000)");

static bool deferredFifo = false;
module_param(deferredFifo, bool, S_IRUGO);
MODULE_PARM_DESC(deferredFifo, " Deferred fifo availability (default=N)");

static spinlock_t myMonitorLock;


/* --- Function Prototypes --- */
static int ebx_monitor_createDeviceAttributeFiles(struct device* inDeviceP);
static void ebx_monitor_removeDeviceAttributeFiles(struct device* inDeviceP);
static void ebx_monitor_measurementsFinished(void);

/* Measurement functions are set depending on the measure mode */
static void ebx_monitor_gotnewframe_real(const struct timeval* inTimeOfNewFrameP);
static void ebx_monitor_gotnewframe_dummy(const struct timeval* inTimeOfNewFrameP);
static void ebx_monitor_gotnewframe_easy(const struct timeval* inTimeOfNewFrameP);
static void (*ebx_monitor_gotnewframe_funP) (const struct timeval* inTimeOfNewFrameP) = ebx_monitor_gotnewframe_dummy;

static void ebx_monitor_gotframe_real(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag);
static void ebx_monitor_gotframe_dummy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag);
static void ebx_monitor_gotframe_easy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag);
static void (*ebx_monitor_gotframe_funP) (const struct timeval* inTimeOfNewFrameP, const unsigned char inTag) = ebx_monitor_gotframe_dummy;

/* Set the measurement functions depending on the mode */
static void ebx_monitor_setMonitorMode(MonitorModeEnum inMode);

/* After a finished measure this function writes the data to the fifo.
   The function is either called directly or deferred (workqueue), depending on configuration (see module parameter). */
static void ebx_monitor_writeDataToFifo(void);

#if USE_TIMER_FOR_FRAME_SIMULATION
static void ebx_monitor_timer_expired(unsigned long inData);
static void ebx_monitor_initTimer(void);
#endif


/* The ebx monitor workqueue function in case of deferredFifo is configured (see module parameter). */
static void ebx_monitor_workqueue(struct work_struct* inDataP);
static DECLARE_WORK(ebx_monitor_work, ebx_monitor_workqueue);


/* --- GLOBAL Funtions --- */
void
__exit ebx_monitor_exit(void)
{
  printk(KERN_INFO CHARDEV_NAME": Cleanup and exit\n");

#if USE_TIMER_FOR_FRAME_SIMULATION
  printk(KERN_INFO CHARDEV_NAME": Delete timer\n");
  del_timer_sync(&ourTimer);
#endif

  flush_scheduled_work();

  /* --- remove device attribute files --- */
  ebx_monitor_removeDeviceAttributeFiles(ebx_monitor_misc.this_device);

  misc_deregister(&ebx_monitor_misc);
  if (measureP != NULL) {
    kfree(measureP);
  }

  if (nbrOfMeasurementPoints > 0) {
    /* free memory used for the kernel fifo */
    printk(KERN_INFO CHARDEV_NAME": Free kfifo memory\n");
    kfifo_free(&ourCharDevFifo);
  }
} /* ebx_monitor_exit */


int
__init ebx_monitor_init(void)
{
  int result  = 0;
  int fifoMax = 0;
#if EBX_DEBUG_OUTPUT
  unsigned int measurePointSize = sizeof(struct MeasurePointS);
#endif
  MonitorModeEnum monitorMode = MonitorModeUndef;
  ebx_monitor_count = 0;

  printk(KERN_INFO CHARDEV_NAME": Initialise (Version "EBX_MONITOR_VERSION")\n");

  /* Initialise the measurement mode */
  if (nbrOfMeasurementPoints == 0) {
    printk(KERN_INFO CHARDEV_NAME": Monitor running in easy mode\n");
    monitorMode = MonitorModeEasy;

  } else {
    printk(KERN_INFO CHARDEV_NAME": Monitor running in real mode: %u measure points\n", nbrOfMeasurementPoints);
    monitorMode = MonitorModeReal;

    /* Limit the number of measure points if necessary */
    if (nbrOfMeasurementPoints > MaxNbrOfMeasurementPoints) {
      nbrOfMeasurementPoints = MaxNbrOfMeasurementPoints;
      printk(KERN_INFO CHARDEV_NAME": Measure points limited to %u\n", nbrOfMeasurementPoints);
    }

    measureP = (struct MeasurePointS*)(kcalloc(nbrOfMeasurementPoints, sizeof(struct MeasurePointS), GFP_KERNEL));
    if (measureP == NULL) {
      result = -ENOMEM;
      goto fail1;
    }
    measurementIdxMax = nbrOfMeasurementPoints - 1; /* -1 because the index starts with zero */

#if EBX_DEBUG_OUTPUT
    printk(KERN_INFO CHARDEV_NAME": sizeof((struct MeasurePointS) = %d, measurementIdxMax = %d\n",
	   measurePointSize,
	   measurementIdxMax);
    printk(KERN_INFO CHARDEV_NAME": Memory used for measure points = %d Bytes\n",
	   measurePointSize * nbrOfMeasurementPoints);
#endif
  } /* if (nbrOfMeasurementPoints == 0) */


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
  if (nbrOfMeasurementPoints > 0) {
#if EBX_DEBUG_OUTPUT
    printk(KERN_INFO CHARDEV_NAME": Allocate kfifo memory\n");
#endif
    result = kfifo_alloc(&ourCharDevFifo, nbrOfMeasurementPoints * nbrOfCharactersPerMeasurePointMax, GFP_KERNEL);
    if (result != 0) {
      printk(KERN_ERR CHARDEV_NAME": Error kfifo_alloc failed, result = %d\n", result);
      goto fail3;
    }
    /* read the fifo size, as the size will be rounded-up to a power of 2 */
    fifoMax = kfifo_size(&ourCharDevFifo);
#if EBX_DEBUG_OUTPUT
    printk(KERN_ERR CHARDEV_NAME": Memory used for fifo = %d Bytes\n", fifoMax);
#endif
  }

  /* --- create device attribute files --- */
  result = ebx_monitor_createDeviceAttributeFiles(ebx_monitor_misc.this_device);
  if (result != 0) {
    goto fail4;
  }

  /* Set the measurement mode: initialise the measurement function pointers depending on mode */
  ebx_monitor_setMonitorMode(monitorMode);

#if USE_TIMER_FOR_FRAME_SIMULATION
  ebx_monitor_initTimer();
#endif

  return result;


 fail4:
  if (nbrOfMeasurementPoints > 0) {
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

  message_read = false; /* just for testing using cat */

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

#if EBX_STATUS_OUTPUT
  printk(KERN_INFO CHARDEV_NAME"_read: read by \"%s\" (pid %i)\n",
	 current->comm,
	 current->pid);
#endif

  /* Get lock or signal restart if interrupted */
  if (mutex_lock_interruptible(&ebx_monitor_lock)) {
    printk(KERN_INFO CHARDEV_NAME"_read: ...return -ERESTARTSYS\n");
    return -ERESTARTSYS;
  }

#if 0 /* Partial read is supported !!! */
  /* Partial reads are unsupported */
  if (ebx_monitor_count > count) {
    ret = -EINVAL;
    goto err;
  }
#endif

  /* The default from 'cat' is to issue multiple reads readOneShot_atomic avoids that */
  if (atomic_read(&readOneShot_atomic) && message_read) {
#if EBX_STATUS_OUTPUT
    printk(KERN_INFO CHARDEV_NAME"_read: ...no data available and one shot active\n");
#endif
    mutex_unlock(&ebx_monitor_lock);
    return 0; /* just for testing using cat */
  }

  /* Return EOF if no data avalable to read */
  if (ebx_monitor_count == 0) {
    if (nbrOfMeasurementPoints == 0) {    /* no data available to read */
      printk(KERN_INFO CHARDEV_NAME"_read: ...no data available (running in easy mode, use sys-fs to retrieve data)\n");
      goto err;
    } else {
      if (fileP->f_flags & O_NONBLOCK) {
#if EBX_STATUS_OUTPUT
        printk(KERN_INFO CHARDEV_NAME"_read: ...no data available, return EOF\n");
#endif
        goto err;
      }
    }
  }

#if EBX_STATUS_OUTPUT
  if (fileP->f_flags & O_NONBLOCK) {
    printk(KERN_INFO CHARDEV_NAME"_read: access NON blocking\n");
  } else {
    printk(KERN_INFO CHARDEV_NAME"_read: access blocking\n");
  }
#endif

  /* Return -EAGAIN if user requested O_NONBLOCK and no data available */
  if ((ebx_monitor_count == 0) && (fileP->f_flags & O_NONBLOCK)) {
    ret = -EAGAIN;
    goto err;
  }

  if (!(fileP->f_flags & O_NONBLOCK)) {
#if EBX_STATUS_OUTPUT
    printk(KERN_INFO CHARDEV_NAME"_read: process blocking read access\n");
#endif
    /* Blocking read in progress */
    /* The loop is necessary, because wait_event tests are not synchronized */
    /* In blocking mode we wait until data are available and deliver the data */
    while (ebx_monitor_count == 0) {
      if (nbrOfMeasurementPoints > 0) {
#if EBX_STATUS_OUTPUT
	printk(KERN_INFO CHARDEV_NAME"_read: ...no data available...wait for data...\n");
#endif
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
#if EBX_DEBUG_OUTPUT
  printk(KERN_INFO CHARDEV_NAME"_read: kfifo_to_user copied %d characters, kfifo_len is now %u\n",
	 copied,
	 kfifo_len(&ourCharDevFifo));
#endif

  /* just to test nonblocking access using cat */
  message_read = true; /* just for testing using cat */

  /* init the atomic measure array index */
  atomic_set(&measureIdx_atomic, -1);

  /* Save state */
  ret = copied;
  *f_pos += ret;

  /* Keep on track with delivered data */
  ebx_monitor_count = ebx_monitor_count - copied;

  /* Start new measure when all data has been read */
  if (ebx_monitor_count == 0) {
    /* All data has been read (fifo is empty), start next measurement */
#if EBX_STATUS_OUTPUT
    printk(KERN_INFO CHARDEV_NAME"_read: ...all data read, set real mode\n");
#endif
    ebx_monitor_setMonitorMode(MonitorModeReal);
  }

  /* Unlock and signal writers */
  mutex_unlock(&ebx_monitor_lock);

  //TODO...trigger possible next statistic...  wake_up_interruptible(&ebx_monitor_writeq);
  return ret ? ret : copied;

 err:
  mutex_unlock(&ebx_monitor_lock);
  return ret;
} /* ebx_monitor_read */


static ssize_t
ebx_monitor_write(struct file * fileP, const char __user *buf, size_t count, loff_t *f_pos)
{
  const char CmdStart[]  = "start";
  const size_t CmdLength = strlen(CmdStart);
  ssize_t ret            = count;
  char* cmdP             = NULL;

#if EBX_STATUS_OUTPUT
  printk(KERN_INFO CHARDEV_NAME"_write: write by \"%s\" (pid %i)\n",
	 current->comm,
	 current->pid);
#endif

  /* Get lock or signal restart if interrupted */
  if (mutex_lock_interruptible(&ebx_monitor_lock)) {
    printk(KERN_INFO CHARDEV_NAME"_read: ...return -ERESTARTSYS\n");
    return -ERESTARTSYS;
  }

  /* Empty write is NOT allowed */
  if (count < 1) return -EINVAL;


  if (!(cmdP = kzalloc(sizeof(char)*(count + 2), GFP_KERNEL))) {
    ret = -ENOMEM;
    goto err;
  }

  /* Copy data from user to kernel */
  if (copy_from_user(cmdP, buf, count)) {
    ret = -EFAULT;
    goto err;
  }

  /* check received command: command has to be "start" or begin with "start..." */
  if (strncmp(cmdP, CmdStart, CmdLength) == 0) {
    /* Start new measurement */
#if EBX_STATUS_OUTPUT
    printk(KERN_INFO CHARDEV_NAME"_write: received command: %s", cmdP);
#endif
    ebx_monitor_setMonitorMode(MonitorModeDummy);
    ebx_monitor_count = 0;
    /* init the atomic measure array index */
    atomic_set(&measureIdx_atomic, -1);
    /* clear kfifo */
#if EBX_DEBUG_OUTPUT
    printk(KERN_INFO CHARDEV_NAME"_write: kfifo_len is %u\n",
	   kfifo_len(&ourCharDevFifo));
#endif
    kfifo_reset(&ourCharDevFifo);
#if EBX_DEBUG_OUTPUT
    printk(KERN_INFO CHARDEV_NAME"_write: kfifo_len is now %u\n",
	   kfifo_len(&ourCharDevFifo));
#endif
    ebx_monitor_setMonitorMode(MonitorModeReal);

  } else {
    printk(KERN_INFO CHARDEV_NAME"_write: received NOT supported command: %s\n", cmdP);
  }

 err:
  if (cmdP != NULL) {
    kfree(cmdP);
  }
  mutex_unlock(&ebx_monitor_lock);
  return ret;
} /* ebx_monitor_write */


static void
ebx_monitor_measurementsFinished(void)
{
#if EBX_STATUS_OUTPUT
  printk(KERN_INFO CHARDEV_NAME": measurement finished...data available, set dummy mode\n");
#endif
  ebx_monitor_setMonitorMode(MonitorModeDummy);

  if (deferredFifo == true) {
    if (schedule_work(&ebx_monitor_work) == 0) {
      printk(KERN_ERR CHARDEV_NAME": schedule_work NOT successful\n");
    }
#if EBX_STATUS_OUTPUT
    else {
      printk(KERN_ERR CHARDEV_NAME": schedule_work successful\n");
    }
#endif
  } else {
    ebx_monitor_writeDataToFifo();
  }
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
#if EBX_DEBUG_OUTPUT
    printk(KERN_INFO CHARDEV_NAME": %s", bufP);
#endif
    strLength = strlen(bufP) + 1;
    if (strLength != kfifo_in(&ourCharDevFifo, bufP, strLength)) {
      printk(KERN_INFO CHARDEV_NAME": ERROR kfifo_in failed\n");
    }
  }
  ebx_monitor_count = kfifo_len(&ourCharDevFifo);
#if EBX_STATUS_OUTPUT
  printk(KERN_INFO CHARDEV_NAME": data ready to be read (kfifo_len = %u)\n", ebx_monitor_count);
#endif
  wake_up_interruptible(&ebx_monitor_readq);
} /* ebx_monitor_writeDataToFifo */


static int
ebx_monitor_createDeviceAttributeFiles(struct device* inDeviceP)
{
  int result = 0;

  /* Only create the sys-fs file when needed */
  if (nbrOfMeasurementPoints == 0) {
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
  } /* if (nbrOfMeasurementPoints > 0) */

  result = device_create_file(inDeviceP, &dev_attr_readOneShot);
  if (result != 0) {
    goto fail;
  }

#if EBX_STATUS_OUTPUT
  printk(KERN_INFO CHARDEV_NAME": result of device_create_file: %d\n", result);
#endif
  return result;

 fail:
  printk(KERN_WARNING CHARDEV_NAME": error in device_create_file\n");
  return result;
} /* ebx_monitor_createDeviceAttributeFiles */


static void
ebx_monitor_removeDeviceAttributeFiles(struct device* inDeviceP)
{
  printk(KERN_INFO CHARDEV_NAME": remove device files\n");

  /* Only remove the sys-fs file when needed */
  if (nbrOfMeasurementPoints == 0) {
    device_remove_file(inDeviceP, &dev_attr_totalFrameCount);
    device_remove_file(inDeviceP, &dev_attr_totalFrameDelta);
    device_remove_file(inDeviceP, &dev_attr_lastFrameDelta);
    device_remove_file(inDeviceP, &dev_attr_averageFrameTime);
  } /* if (nbrOfMeasurementPoints > 0) */

  device_remove_file(inDeviceP, &dev_attr_readOneShot);
} /* ebx_monitor_removeDeviceAttributeFiles */


static void
ebx_monitor_gotnewframe_dummy(const struct timeval* inTimeOfNewFrameP)
{
  (void)inTimeOfNewFrameP;
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

  ktime_t frame_ktime    = ktime_get();
  ktime_t newFrame_ktime = timeval_to_ktime(*inTimeOfNewFrameP);

  idx = atomic_add_return(1, &measureIdx_atomic);
  if ((idx >= 0) && (idx <= measurementIdxMax)) {
#if EBX_DEBUG_OUTPUT
    printk(KERN_INFO CHARDEV_NAME": gotnewframe_real(), idx = %i, measurementIdxMax = %d\n", idx, measurementIdxMax);
#endif
    measureP[idx].id        = newFrame_ktime;
    measureP[idx].timeStamp = frame_ktime;
    measureP[idx].tag       = 0; /* the tag for a new frame is always zero */
  } else {
    ebx_monitor_measurementsFinished();
  }

  prevFrame_ktime = newFrame_ktime;
} /* ebx_monitor_gotnewframe_real */


static void
ebx_monitor_gotframe_dummy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag)
{
  (void)inTimeOfNewFrameP;
  (void)inTag;
  /* printk(KERN_INFO CHARDEV_NAME": gotframe_dummy() called\n"); */
} /* ebx_monitor_gotframe_dummy */


void
ebx_monitor_gotframe_easy(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag)
{
  (void)inTimeOfNewFrameP;
  (void)inTag;
  /* printk(KERN_INFO CHARDEV_NAME": gotframe_easy() called\n"); */
} /* ebx_monitor_gotframe_easy */


void
ebx_monitor_gotframe_real(const struct timeval* inTimeOfNewFrameP, const unsigned char inTag)
{
  int idx = -1;

  ktime_t frame_ktime    = ktime_get();
  ktime_t newFrame_ktime = timeval_to_ktime(*inTimeOfNewFrameP);

  idx = atomic_add_return(1, &measureIdx_atomic);
  if ((idx >= 0) && (idx <= measurementIdxMax)) {
#if EBX_DEBUG_OUTPUT
    printk(KERN_INFO CHARDEV_NAME": gotframe_real(), idx = %i: %lldus, %lldus, tag = %u\n",
	   idx, ktime_to_us(newFrame_ktime), ktime_to_us(frame_ktime), inTag);
#endif
    measureP[idx].id        = newFrame_ktime;
    measureP[idx].timeStamp = frame_ktime;
    measureP[idx].tag       = inTag;
  } else {
    ebx_monitor_measurementsFinished();
  }
} /* ebx_monitor_gotframe_real */


static void
ebx_monitor_setMonitorMode(MonitorModeEnum inMode)
{
  unsigned long iflags;

  switch(inMode) {
  case MonitorModeEasy:
    spin_lock_irqsave(&myMonitorLock, iflags);
    ebx_monitor_gotnewframe_funP = ebx_monitor_gotnewframe_easy;
    ebx_monitor_gotframe_funP    = ebx_monitor_gotframe_easy;
    spin_unlock_irqrestore(&myMonitorLock, iflags);
    break;
  case MonitorModeReal:
#if EBX_STATUS_OUTPUT
    printk(KERN_INFO CHARDEV_NAME": --- START Measurement ---\n");
#endif
    spin_lock_irqsave(&myMonitorLock, iflags);
    ebx_monitor_gotnewframe_funP = ebx_monitor_gotnewframe_real;
    ebx_monitor_gotframe_funP    = ebx_monitor_gotframe_real;
    spin_unlock_irqrestore(&myMonitorLock, iflags);
    break;
  case MonitorModeDummy:
#if EBX_STATUS_OUTPUT
    printk(KERN_INFO CHARDEV_NAME": --- STOP Measurement ---\n");
#endif
    spin_lock_irqsave(&myMonitorLock, iflags);
    ebx_monitor_gotnewframe_funP = ebx_monitor_gotnewframe_dummy;
    ebx_monitor_gotframe_funP    = ebx_monitor_gotframe_dummy;
    spin_unlock_irqrestore(&myMonitorLock, iflags);
    break;
  default:
    printk(KERN_INFO CHARDEV_NAME"_setMonitorMode: UNKNOWN mode\n");
    break;
  }

} /* ebx_monitor_setMonitorMode */


static void
ebx_monitor_workqueue(struct work_struct* inDataP)
{
#if EBX_STATUS_OUTPUT
  printk(KERN_INFO CHARDEV_NAME"_workqueue: triggers ebx_monitor_writeDataToFifo()\n");
#endif
  ebx_monitor_writeDataToFifo();
} /* ebx_monitor_workqueue */


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
