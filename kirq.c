/*  Example  kernel module driver 
    by Ludovic Saint-Bauzel (saintbauzel@isir.upmc.fr)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <asm/io.h>

#include <asm/uaccess.h>	/* copy_{from,to}_user() */
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/cdev.h>  

#define  DEVICE_NAME "kirq"    ///< The device will appear at /dev/kirq using this value
#define  CLASS_NAME  "kirq"        ///< The device class -- this is a character device driver
static struct class*  charClass  = NULL; ///< The device-driver class struct pointer
static struct device* charDevice = NULL; ///< The device-driver device struct pointer
int major;


#define PERIOD 1200

#define AM33XX_CONTROL_BASE		0x44e10000
// spruh73m.pdf  Issue de L3 memory et L4 memory p179 p181 p182
#define GPIO0_REGISTER 0x44e07000
#define GPIO1_REGISTER 0x4804C000
#define GPIO2_REGISTER 0x481AC000
#define GPIO3_REGISTER 0x481AE000


// p. 183 
#define PWMSS0_REG 0x48300000
#define PWMSS1_REG 0x48302000
#define PWMSS2_REG 0x48304000

#define ECAP_OFF 0x100
#define EQEP_OFF 0x180
#define EPWM_OFF 0x200



// spruh73m.pdf Issue de register description GPIO p 4881 ...
#define GPIO_OE 0x134
#define GPIO_DATAIN 0x138
#define GPIO_DATAOUT 0x13C
#define GPIO_CTRL 0x130
#define GPIO_CLRDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194


// spruh73m.pdf Issue de p 1369
#define OFFSET_PWMSS_CTRL 0x664


#define OFFSET_PIN9_12 0x878
#define GPIO1_28_PIN9_12 28

#define GPIO_PIN9_12 60

#define OFFSET_PIN_9_14 0x848
#define GPIO1_18_PIN9_14 18

#define GPIO_PIN9_14 50

#define EPWM_TBCTL 0x00
// Bits in TBCTL 
#define CTRMODE 0x0
#define PHSEN 0x2
#define PRDLD 0x3
#define SYNCOSEL 0x4
#define HSPCLKDIV 0x7
#define CLKDIV 0xA 
// Values in TBCTL
#define TB_UP 0x0
#define TB_UPDOWN 0x2 
#define TB_DISABLE 0x0 
#define TB_SHADOW 0x0
#define TB_SYNC_DISABLE 0x3
#define TB_DIV1 0x0
#define TB_DIV2 0x1
#define TB_DIV4 0x2

#define EPWM_CMPCTL 0x0E
// Bits in CMPCTL
#define SHDWAMODE 0x4
#define SHDWBMODE 0x6
#define LOADAMODE 0x0
#define LOADBMODE 0x2
// Values in CMPCTL
#define CC_SHADOW 0x0
#define CC_CTR_ZERO 0x0

#define EPWM_TBCNT 0x08
#define EPWM_TBPRD 0x0A
#define EPWM_TBPHS 0x06

#define EPWM_CMPA 0x12
#define EPWM_CMPB 0x14

#define EPWM_AQCTLA 0x16
// Bits in AQCTL
#define ZRO 0
#define PRD 2
#define CAU 4
#define CAD 6
#define CBU 8
#define CBD 10
// Values
#define AQ_CLEAR 0x1
#define AQ_SET 0x2



//#define ECAP_OFF 0x100
/* ECAP registers and bits definitions */
#define CAP1			0x08
#define CAP2			0x0C
#define CAP3			0x10
#define CAP4			0x14
#define ECCTL2			0x2A
#define ECCTL2_APWM_POL_LOW     (0x1 << 10)
#define ECCTL2_APWM_MODE        (0x1 << 9)
#define ECCTL2_SYNC_SEL_DISA	((0x1 << 7) |(0x1 << 6))
#define ECCTL2_TSCTR_FREERUN	(0x1 << 4)

#define INTC_MIR_CLEAR2 0xC8
#define INTC_MIR_SET2 0xCC
#define EPWM0INT 86 
#define EPWM1INT 87


#define PWM_SET 0
#define GPIO_SET 1
#define GPIO_CLEAR 4


MODULE_DESCRIPTION("Simple ioctl IRQ GPIO input driver (char)");
MODULE_AUTHOR("TOURLOUR Thomas, MARILLESSE Lucas");
MODULE_LICENSE("GPL");


int counter=0;
static irqreturn_t test_irq_latency_interrupt_handler(unsigned long ptr)
{
  counter++;
  printk( KERN_DEBUG "irq handler counter increased : %d \n", counter);
  return IRQ_HANDLED;
}

static long char_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  unsigned short val;
  
  val=(unsigned short)arg;
  if(val > PERIOD)
    val = PERIOD;
  
  switch (cmd) {
  case PWM_SET :
    break;
  case GPIO_SET:
    break;
  case GPIO_CLEAR:
    break;
  default :
    printk(KERN_WARNING "kirq: 0x%x unsupported ioctl command\n", cmd);  
    return -EINVAL;
  }
  return 0;
}

static struct file_operations char_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl =	char_ioctl,
};

static int __init pinmuxIRQ(void)
{
  int ret=0;
  u16 irq;
  int regval ;                 
  void __iomem* cm_per_gpio;
  
  /********************* PINMUX spruh73m.pdf p. 179 - 1370 - 1426 ****************/
  cm_per_gpio = ioremap(/*A REMPLIR*/0x44E10848,4); // adresse du pinMux de la pin 9_14 (lu sur le tableau "BBBP9Header") 
  if(!cm_per_gpio) {
    printk("cm_per_gpio_reg ioremap: error\n");
  }
  else {
    regval = ioread32(cm_per_gpio) ; //get GPIO MUX register value
    iowrite32( /*A REMPLIR*/0X00000037 ,cm_per_gpio);// p. 1426 : GPIO MUX to GPIO INPUT (Mode 7)
    //                                             Pullup  (sel. and enabled)
    regval = ioread32(cm_per_gpio) ; //enabled? GPIO MUX
    printk("GPX1CON register mux (1e) : 0x%08x\n", regval);
    
  }
  
  ret = gpio_request_one( /*A REMPLIR*/GPIO_PIN9_14, GPIOF_IN,
			  DEVICE_NAME " gpio"); // on utilise P9_14 en mode GPIO ce qui correspond au GPIO 50, donc GPIO1[18]
  if (ret < 0) {
    printk(KERN_ALERT DEVICE_NAME " : failed to request GPIO pin.\n" );
    return ret;
  }
  ret  = gpio_to_irq(/*A REMPLIR*/GPIO_PIN9_14); /* On génere un numero d'IRQ pour la pin9_14 configurée en GPIO*/
  if (ret < 0) {
    printk(KERN_ALERT DEVICE_NAME " : failed to get IRQ for pin.\n");
    return ret;
  } else {
    irq = (u16)ret;
    //err = 0;
  }
  /*int request_irq(unsigned int irq,
   void (*handler)(int, void *, struct pt_regs *),
   unsigned long flags, 
   const char *dev_name,
   void *dev_id);
   void free_irq(unsigned int irq, void *dev_id);*/

  ret = request_any_context_irq(
				/*A REMPLIR*/irq, /* numéro d'IRQ généré au dessus*/
				/*A REMPLIR*/(irq_handler_t) test_irq_latency_interrupt_handler, /*fonction d'interruption à effectuer lorsque qu'un front descendant est détecté sur la pin9_14 */
				IRQF_TRIGGER_FALLING,
				DEVICE_NAME,
				(void*)NULL
				);
  if (ret < 0) {
    printk(KERN_ALERT DEVICE_NAME " : failed to enable IRQ for pin.\n");
    return ret;
    
  }
  
  return ret;
  
}


// this gets called on module init
static int __init kernmodex_init(void)
{
  int ret;
  /* int regval ;                  */
  /* short sregval; */
  /* void __iomem* cm_per_gpio; */
  
  printk(KERN_INFO "Loading example driver by Ludo...\n");

  ret = pinmuxIRQ();
  if (ret < 0)
    {
      printk("problem in pinmux of PWM");
      return ret;
    }


  /************** Char Device creation *****************************/
  ret = register_chrdev(major, CLASS_NAME, &char_fops);
  if (ret < 0) {
    printk(KERN_WARNING "KPWM: unable to get a major\n");

    return ret;
  }
  if (major == 0)
    major = ret; /* dynamic value */
  
  printk(KERN_INFO "KIRQ: successfully loaded with major %d\n", major);
  
  charClass = class_create(THIS_MODULE, CLASS_NAME);
  if (IS_ERR(charClass)){                // Check for error and clean up if there is
    unregister_chrdev(major, DEVICE_NAME);
    printk(KERN_ALERT "Failed to register device class\n");
    return PTR_ERR(charClass);          // Correct way to return an error on a pointer
  }
  printk(KERN_INFO "KIRQ device class registered correctly\n");
  
  
  // Register the device driver
  
  
  charDevice = device_create(charClass, NULL, MKDEV(major, 0), NULL, DEVICE_NAME );
  if (IS_ERR(charDevice)){               // Clean up if there is an error
    class_destroy(charClass);           // Repeated code but the alternative is goto statements
    unregister_chrdev(major, DEVICE_NAME);
    printk(KERN_ALERT "Failed to create the device\n");
    return PTR_ERR(charDevice);
  }
  printk(KERN_INFO "KIRQ: device class created correctly\n"); // Made it! device was initialized
  
  return 0;
  
}

// this gets called when module is getting unloaded
static void __exit kernmodex_exit(void)
{
  device_destroy(charClass, MKDEV(major, 0));

  class_unregister(charClass);  
  class_destroy(charClass);
  unregister_chrdev(major, CLASS_NAME);

  gpio_free(GPIO_PIN9_14);

  printk(KERN_INFO "Example driver by Ludo removed.\n");
  
  
}

// setting which function to call on module init and exit
module_init(kernmodex_init);
module_exit(kernmodex_exit);


MODULE_VERSION("0.3");
