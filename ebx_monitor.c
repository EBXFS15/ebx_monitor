/*
 * Device:
 *   ls -l /dev/ebx_monitor
 *
 * Device Attributes:
 *   ls -l /sys/devices/virtual/misc/ebx_monitor
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
static ktime_t newFrame_ktime;
static ktime_t frame_ktime;

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
static char hello[] = "Hello";
static bool message_read; /* just for testing using cat */
/* DEBUG END */

/* --- Function Prototypes --- */
static int ebx_monitor_createDeviceAttributeFiles(struct device* inDeviceP);
static void ebx_monitor_measurementsFinished(void);

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

  init_waitqueue_head(&ebx_monitor_readq);
  mutex_init(&ebx_monitor_lock);

  if ((result = misc_register(&ebx_monitor_misc))) {
    printk(KERN_WARNING CHARDEV_NAME": error registering!\n");
    goto fail;
  }

  /* --- create device attribute files --- */
  result = ebx_monitor_createDeviceAttributeFiles(ebx_monitor_misc.this_device);

#if USE_TIMER_FOR_FRAME_SIMULATION
  ebx_monitor_initTimer();
#endif

  /* DEBUG START */
  ebx_monitor_count = (strlen(hello) + 1);
  /* DEBUG END */

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

  /* DEBUG START */
  if (tmp == 0) {
    printk(KERN_INFO CHARDEV_NAME": One shot is deactivated\n");
  } else {
    printk(KERN_INFO CHARDEV_NAME": One shot is activated\n");
  }
  /* DEBUG END */

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

  /* DEBUG START */
  printk(KERN_INFO/*DEBUG*/ CHARDEV_NAME": read by \"%s\" (pid %i)\n",
	 current->comm,
	 current->pid);
  /* DEBUG END */

  /* Get lock or signal restart if interrupted */
  if (mutex_lock_interruptible(&ebx_monitor_lock)) return -ERESTARTSYS;

  /* Partial reads are unsupported */
  if (ebx_monitor_count > count) {
    ret = -EINVAL;
    goto err;
  }

  /* The default from 'cat' is to issue multiple reads until the FIFO is
   *  depleted, readOneShot_atomic avoids that */
  if (atomic_read(&readOneShot_atomic) && message_read) {
    mutex_unlock(&ebx_monitor_lock);
    return 0; /* just for testing using cat */
  }


#if 0
  /* Return EOF if no data avalable to read */
  if (ebx_monitor_count == 0) {
    /* no data available to read */
    goto err;
  }
#endif

  /* DEBUG START */
  if (fileP->f_flags & O_NONBLOCK) {
    printk(KERN_INFO CHARDEV_NAME": read access NON blocking\n");
  } else {
    printk(KERN_INFO CHARDEV_NAME": read access blocking\n");
  }
  /* DEBUG END */

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
      printk(KERN_INFO CHARDEV_NAME": ... no data available\n");
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
  printk(KERN_INFO CHARDEV_NAME": copy data to user: \"%s\"\n", hello);
  if (copy_to_user(buf, hello, ebx_monitor_count)) {
    ret = -EFAULT;
    goto err;
  }
  /* DEBUG START */
  message_read = true; /* just for testing using cat */
  /* DEBUG END */

  /* Save state */
  ret = ebx_monitor_count;
  *f_pos += ret;

  /* Reset buffer */
  ebx_monitor_count = 0;

  /* Unlock and signal writers */
  mutex_unlock(&ebx_monitor_lock);
  //TODO...trigger possible next statistic...  wake_up_interruptible(&fifodev->writeq);
  return ret;

 err:
  mutex_unlock(&ebx_monitor_lock);
  return ret;
} /* ebx_monitor_read */


static void
ebx_monitor_measurementsFinished(void)
{
  wake_up_interruptible(&ebx_monitor_readq);
} /* ebx_monitor_measurementsFinished */


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
