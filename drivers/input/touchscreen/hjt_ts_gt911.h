
#ifndef _GOODIX_GT9XX_H_
#define _GOODIX_GT9XX_H_


//#include<mach/w55fa92_reg.h>
//#include<mach/w55fa92_gpio.h>
#include<linux/kernel.h>
#include<linux/hrtimer.h>
#include<linux/i2c.h>
#include<linux/input.h>
#include<linux/module.h>
#include<linux/delay.h>
#include<linux/i2c.h>
#include<linux/proc_fs.h>
#include<linux/string.h>
#include<linux/uaccess.h>
#include<linux/vmalloc.h>
#include<linux/interrupt.h>
#include<linux/io.h>
#include<linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/major.h>
#include <linux/kdev_t.h>
#ifdef CONFIG_OF
#include<linux/of_gpio.h>
#include<linux/regulator/consumer.h>
#endif
#ifdef CONFIG_FB
#include<linux/notifier.h>
#include<linux/fb.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif


//***************************PART1:ON/OFFdefine*******************************
#define GTP_CUSTOM_CFG        1   
#define GTP_CHANGE_X2Y        0   //swap x y
#define GTP_DRIVER_SEND_CFG   1   //driver send config, This switch is selected according to need.
#define GTP_CONFIG_MODE  0//When Touch screen was good, without manufacturer's data sheet: 0 = read the original configuration parameters from GT911, and reconfigure after modification
//When The situation of having manufacturer's data sheet 1: Configuration after modifying the data sheet
#define GTP_HAVE_TOUCH_KEY    0
#define GTP_POWER_CTRL_SLEEP  0    //power off when suspend
#define GTP_ICS_SLOT_REPORT   0    // slot protocol 


#define GTP_AUTO_UPDATE       0    // auto update fw by .binfile as default
#define GTP_HEADER_FW_UPDATE  0    // auto update fw by gtp_default_FWin gt9xx_firmware.h, function together with GTP_AUTO_UPDATE
#define GTP_AUTO_UPDATE_CFG   0    // auto update config by .cfg file,function together with GTP_AUTO_UPDATE


#define GTP_COMPATIBLE_MODE   0    // compatible with GT9XXF


#define GTP_CREATE_WR_NODE    0
#define GTP_ESD_PROTECT       0    // esd protection with acycle of 2 seconds


#define GTP_WITH_PEN          0
#define GTP_PEN_HAVE_BUTTON   0    // active pen has buttons, functiontogether with GTP_WITH_PEN


#define GTP_GESTURE_WAKEUP    0    // gesture wakeup 


#define GTP_DEBUG_ON         1
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0




struct gt9xx_event {
int touch_point;


u16 x[5];
u16 y[5];
u16 w[5];
};


struct goodix_ts_data {
   spinlock_t irq_lock;
   struct i2c_client *client;
   struct input_dev  *input_dev;
struct gt9xx_event event;
   struct hrtimer timer;
   struct work_struct  work;
   s32 irq_is_disable;
   s32 use_irq;
   u16 abs_x_max;
   u16 abs_y_max;
   u8  max_touch_num;
   u8  int_trigger_type;
   u8  green_wake_mode;
   u8  enter_update;
   u8  gtp_is_suspend;
   u8  gtp_rawdiff_mode;
   int  gtp_cfg_len;
   u8  fw_error;
   u8  pnl_init_error;


/*#if  defined(CONFIG_FB)*/
struct notifier_block notifier;

/*#elif defined(CONFIG_HAS_EARLYSUSPEND)
struct early_suspend early_suspend;
#endif*/
 
};




extern int gtp_rst_gpio;
extern int gtp_int_gpio;


//***************************PART2:TODO define **********************************
//STEP_1(REQUIRED): Define Configuration Information Group(s)
//Sensor_ID Map:
/*sensor_opt1 sensor_opt2 Sensor_ID
   GND         GND         0 
   VDDIO      GND          1 
   NC           GND         2 
   GND         NC/300K    3 
   VDDIO      NC/300K    4 
   NC           NC/300K    5 
*/
//TODO: define your own default or for Sensor_ID == 0 config here. 
// Thepredefined one is just a sample config, which is not suitable for your tp inmost cases.
//------This configuration table is provided by the touch screen manufacturer. If not, do not configure it. Read the original configuration parameters first, otherwise the touch screen may not be used after configuration.---//
#define CTP_CFG_GROUP0 {\
0x42,0x00,0x04,0x58,0x02,0x05,0x3D,0x00,0x01,0x08,\
0x28,0x08,0x50,0x3C,0x03,0x05,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x18,0x1A,0x1E,0x14,0x89,0x2A,0x0B,\
0x40,0x42,0xB5,0x06,0x00,0x00,0x00,0x02,0x02,0x1D,\
0x00,0x01,0x00,0x00,0x00,0x03,0x64,0x00,0x00,0x00,\
0x00,0x32,0x5A,0x94,0xC5,0x02,0x08,0x00,0x00,0x00,\
0x98,0x35,0x00,0x8A,0x3B,0x00,0x7A,0x43,0x00,0x6E,\
0x4B,0x00,0x62,0x55,0x00,0x62,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,\
0x12,0x14,0x16,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0F,0x10,\
0x12,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,\
0x24,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x60,0x01} 
  
//STEP_2(REQUIRED): Customize your I/O ports & I/O operations
//---According to the chip settings, the following settings are N32926---//
#define GTP_RST_PORT   103// 0x6d//GPE6------------------------------
#define GTP_INT_PORT   104// 25//GPG9
#define GTP_POWER_PORT   105// 25//GPG9
#define GTP_INT_IRQ_NUM   69 //          W55FA92_IRQ(3) // nIRQ1
//#define CLEAR_EXT_INTERRUPT_FLAG() writel(readl(REG_IRQTGSRC2) | (1<<GTP_INT_PORT), REG_IRQTGSRC2)//Clear INT interrupt flag GPG9
#define CLEAR_EXT_INTERRUPT_FLAG() *(volatile unsigned int *)(0x800000001600146c) |= 1<<27;
//#define READ_EXT_INTERRUPT_FLAG() readl(REG_IRQTGSRC2)& 0xffff0000//Read INT interrupt flag GPG
#define READ_EXT_INTERRUPT_FLAG() *(volatile unsigned int *)(0x8000000016001460);


//External interruption 1 
#define GPIO_SET_SRCGRP() do{\
writel(readl(REG_IRQSRCGPG)& ~(3 << 18), REG_IRQSRCGPG);\
writel(readl(REG_IRQSRCGPG)|(1<< 18), REG_IRQSRCGPG);\
}while(0)


//Set GPG9 interrupt type to drop edge 
#define GPIO_SET_INTMODE() do{\
*(volatile unsigned char *)(0x8000000016104b68) |= 1;\
*(volatile unsigned int *)(0x8000000016001470) |= (1<<27);\
*(volatile unsigned int *)(0x8000000016001474) |= (1<<27);\
}while(0)

#define EXT_SET_MODE() writel((readl(REG_AIC_SCR1)& ~(0x07000000)) | 0x07000000,REG_AIC_SCR1)//External interrupt 2, descent edge, level 7
       
#define GTP_CONFIG_PIN(pin) w55fa92_gpio_configure(pin/0x20,pin%0x20)  

#define GTP_GPIO_AS_INPUT(pin)          do{\
                                         gpio_direction_input(pin);\
                                      }while(0)
#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)

#define GTP_GPIO_AS_INT(pin)            do{\
                                         GTP_GPIO_AS_INPUT(pin);\
\
                                      }while(0)


#define GTP_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define GTP_GPIO_FREE(pin)             gpio_free(pin)
#define GTP_IRQ_TAB                    {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW,IRQ_TYPE_LEVEL_HIGH}




//STEP_3(optional): Specify your special config info if needed
#if GTP_CUSTOM_CFG
 #define GTP_MAX_HEIGHT   600
 #define GTP_MAX_WIDTH    1024
 #define GTP_INT_TRIGGER  1           // 0: Rising1: Falling
#else
 #define GTP_MAX_HEIGHT   4096
 #define GTP_MAX_WIDTH    4096
 #define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH         1


//STEP_4(optional): If keys are available and reported as keys, config your keyinfo here                            
#if GTP_HAVE_TOUCH_KEY
   #define GTP_KEY_TAB  {KEY_MENU, KEY_HOME, KEY_BACK}
#endif


//***************************PART3:OTHERdefine*********************************
#define GTP_DRIVER_VERSION         "V2.4<2014/11/28>"
#define GTP_I2C_NAME               "Goodix-TS"
#define GT91XX_CONFIG_PROC_FILE     "gt9xx_config"
#define GTP_POLL_TIME         10    
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_MIN_LENGTH 186
#define GTP_CONFIG_MAX_LENGTH 240
#define FAIL                  0
#define SUCCESS               1
#define SWITCH_OFF            0
#define SWITCH_ON             1


//********************For GT9XXF Start **********************//
#define GTP_REG_BAK_REF                 0x99D0
#define GTP_REG_MAIN_CLK                0x8020
#define GTP_REG_CHIP_TYPE               0x8000
#define GTP_REG_HAVE_KEY                0x804E
#define GTP_REG_MATRIX_DRVNUM           0x8069    
#define GTP_REG_MATRIX_SENNUM           0x806A


#define GTP_FL_FW_BURN              0x00
#define GTP_FL_ESD_RECOVERY         0x01
#define GTP_FL_READ_REPAIR          0x02


#define GTP_BAK_REF_SEND                0
#define GTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31


#define GTP_CHK_FW_MAX                  40
#define GTP_CHK_FS_MNT_MAX              300
#define GTP_BAK_REF_PATH               "/data/gtp_ref.bin"
#define GTP_MAIN_CLK_PATH              "/data/gtp_clk.bin"
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                 0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                  0xFF


//********************For GT9XXF End **********************//
//Registers define
#define GTP_READ_COOR_ADDR    0x814E
#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x814A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION       0x8140
#define GTP_REG_COMMAND  0x8040


#define GTP_COMMAND_READSTATUS        0
#define GTP_COMMAND_DIFFERENCE        1
#define GTP_COMMAND_SOFTRESET        2
#define GTP_COMMAND_UPDATE       3
#define GTP_COMMAND_CALCULATE          4
#define GTP_COMMAND_TURNOFF         5


#define RESOLUTION_LOC        3
#define TRIGGER_LOC           8


#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))
// Logdefine
#define GTP_INFO(fmt,arg...)          printk("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)         printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                       if(GTP_DEBUG_ON)\
                                      printk("<<-GTP-DEBUG->>[%d]"fmt"\n",__LINE__, ##arg);\
                                     }while(0)
                                     
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                       s32 i;\
                                       u8* a = array;\
                                      if(GTP_DEBUG_ARRAY_ON)\
                                       {\
                                          printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                          for (i =0; i < (num); i++)\
                                          {\
                                             printk("0x%02x,", (a)[i]);\
                                             if ((i + 1 ) %10 == 0)\
                                             {\
                                                 printk("\\\n");\
                                             }\
                                          }\
                                         printk("\n");\
                                      }\
                                     }while(0)
                                     
#define GTP_DEBUG_FUNC()       			do{\
                                       if(GTP_DEBUG_FUNC_ON)\
                                      printk("<<-GTP-FUNC->>Func:%s@Line:%d\n",__func__,__LINE__);\
                                     }while(0)
                                     
#define GTP_SWAP(x, y)                 do{\
                                       typeof(x) z = x;\
                                       x = y;\
                                       y = z;\
                                     }while (0)


//*****************************Endof Part III********************************
#ifdef CONFIG_OF
//intgtp_parse_dt_cfg(struct device *dev, u8 *cfg, int *cfg_len, u8 sid);
#endif
#endif/* _GOODIX_GT9XX_H_ */
