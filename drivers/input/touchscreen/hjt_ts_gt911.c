#include<linux/irq.h>
#include <linux/platform_device.h>
#include <linux/input/mt.h>
#include"hjt_ts_gt911.h"

static const char *goodix_input_phys = "input/ts";
static const char *goodix_ts_name = "goodix-ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL; 
int gtp_rst_gpio;
int gtp_int_gpio;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
= {GTP_REG_CONFIG_DATA>> 8, GTP_REG_CONFIG_DATA & 0xff};

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t*);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t,loff_t *);

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(s32 ms);


static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = gt91xx_config_read_proc,
	.write = gt91xx_config_write_proc,
};
static int gtp_register_powermanger(struct goodix_ts_data *ts);
static int gtp_unregister_powermanger(struct goodix_ts_data *ts);

/*******************************************************
Function:
Read data from the i2c slave device.
Input:
client:     i2c device.
buf[0~1]:   read start address.
buf[2~len-1]:   read data buffer.
len:    GTP_ADDR_LENGTH + read bytes count
Output:
numbers of i2c_msgs to transfer: 
2: succeed, otherwise: failed
 *********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret=-1;
	s32 retries = 0;

	// GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];
	//msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
	//msgs[1].scl_rate = 300 * 1000;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	if((retries >= 5))
	{   
		GTP_ERROR("I2C Read: 0x%04X, %d bytes failed,errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]),len-2, ret);
		{
			gtp_reset_guitar(client, 10);  
		}
	}
	return ret;
}

/*******************************************************
Function:
Write data to the i2c slave device.
Input:
client:     i2c device.
buf[0~1]:   write start address.
buf[2~len-1]:   data buffer
len:    GTP_ADDR_LENGTH + write bytes count
Output:
numbers of i2c_msgs to transfer: 
1: succeed, otherwise: failed
 *********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;
	//msg.scl_rate = 300 * 1000;    // for Rockchip, etc

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)break;
		retries++;
	}
	if((retries >= 5))
	{
		GTP_ERROR("I2C Write: 0x%04X, %d bytes failed,errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]),len-2, ret);
		{
			gtp_reset_guitar(client, 10);  
		}
	}
	return ret;
}

/*******************************************************
Function:
i2c Send_Command
Input:
client:  i2c device
command:   command
Output:
ret:    >0 ok
=0failed
 *********************************************************/
s8 GTP_Send_Command(u8 command,struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	u8 command_buf[3] = {(u8)(GTP_REG_COMMAND >> 8),(u8)GTP_REG_COMMAND&0xFF, 1};
	command_buf[2]= command;

	GTP_DEBUG_FUNC();

	while(retry++ < 5)
	{
		ret =gtp_i2c_write(client, command_buf , 1 + GTP_ADDR_LENGTH);
		if (ret > 0)
		{
			GTP_INFO("send command success!");
			return ret;
		}
	}
	GTP_ERROR("send command fail!");
	return ret;
}

/*******************************************************
Function:
i2c read twice, compare the results
Input:
client:  i2c device
addr:    operate address
rxbuf:   read data to store, if compare successful
len:     bytes to read
Output:
FAIL:    read failed
SUCCESS: read successful
 *********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	u8 retry = 0;

	while (retry++ < 3)
	{
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);

		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);

		if (!memcmp(buf, confirm_buf, len+2))
		{
			memcpy(rxbuf, confirm_buf+2, len);
			return SUCCESS;
		}
	}    
	GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!",addr, len);
	return FAIL;
}

/*******************************************************
Function:
i2c read config data check it
Input:
client:  i2c device
Output:
FAIL:    read failed
SUCCESS: read successful
 *********************************************************/
int gtp_i2c_read_cfg_check(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	u8 buf[GTP_CONFIG_MIN_LENGTH + GTP_ADDR_LENGTH];


	u8 retry = 0;

	memset(buf,0, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
	buf[0]= config[0];
	buf[1]= config[1];
	gtp_i2c_read(client,buf, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
	GTP_DEBUG_ARRAY(buf+GTP_ADDR_LENGTH,ts->gtp_cfg_len);
	if(memcmp(buf+GTP_ADDR_LENGTH,config+GTP_ADDR_LENGTH, ts->gtp_cfg_len-1) == 0)
	{
		GTP_INFO("cfgcheck ok!\r\n");
		return SUCCESS;
	}
	else
	{
		GTP_INFO("cfgcheck failed!\r\n");
		return FAIL;
	}

}

/*******************************************************
Function:
Send config.
Input:
client: i2c device.
Output:
result of i2c write operation. 
1: succeed, otherwise: failed
 *********************************************************/

s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->pnl_init_error)
	{
		GTP_INFO("Error occured in init_panel, no configsent");
		return 0;
	}

	GTP_INFO("Driver send config.");
	GTP_DEBUG_ARRAY(config+GTP_ADDR_LENGTH,ts->gtp_cfg_len);
	for (retry = 0; retry < 5; retry++)
	{
		ret = gtp_i2c_write(client, config , ts->gtp_cfg_len +GTP_ADDR_LENGTH);
		if (ret > 0)
		{
			break;
		}
	}

	msleep(100);
	gtp_i2c_read_cfg_check(client);
#endif
	return ret;
}
/*******************************************************
Function:
Disable irq function
Input:
ts: goodix i2c_client private data
Output:
None.
 *********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;
	GTP_DEBUG_FUNC();
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable)
	{
		ts->irq_is_disable = 1; 
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
Enable irq function
Input:
ts: goodix i2c_client private data
Output:
None.
 *********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;
	GTP_DEBUG_FUNC();
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) 
	{
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0; 
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
Report touch point event 
Input:
ts: goodix i2c_client private data
id: trackId
x:  input x coordinate
y:  input y coordinate
w:  input pressure
Output:
None.
 *********************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif
	input_report_abs(ts->input_dev,ABS_X, x);
	input_report_abs(ts->input_dev,ABS_Y, y);
	input_report_abs(ts->input_dev,ABS_PRESSURE, w);
	input_report_key(ts->input_dev,BTN_TOUCH, 1);
	input_sync(ts->input_dev);
	GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
Report touch release event
Input:
ts: goodix i2c_client private data
Output:
None.
 *********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
	input_report_abs(ts->input_dev,ABS_PRESSURE, 0);
	input_report_key(ts->input_dev,BTN_TOUCH, 0);
	input_sync(ts->input_dev);
	GTP_DEBUG("touchrelease\r\n");
}

/*******************************************************
Function:
Goodix touchscreen work function
Input:
work: work struct of goodix_workqueue
Output:
None.
 *********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR& 0xFF, 0};
	u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR>> 8, GTP_READ_COOR_ADDR & 0xFF};
	u8  touch_num = 0;
	u8  finger = 0;
	static u8  pre_touch=0;
	u8 test_data[12]={GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA &0xff};

	u8* coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i  = 0;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;

	// GTP_DEBUG_FUNC();
	ts = container_of(work, struct goodix_ts_data, work);
	if (ts->enter_update)
	{
		return;
	}

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0)
	{
		GTP_ERROR("I2C transfer error. errno:%d\n ",ret);
		if (ts->use_irq)
		{
			gtp_irq_enable(ts);
		}
		return;
	}
	finger = point_data[GTP_ADDR_LENGTH];
	if (finger == 0x00)
	{
		if (ts->use_irq)
		{
			gtp_irq_enable(ts);
		}
		return;
	}

	GTP_DEBUG("finger=0x%x\n", finger);
	if((finger & 0x80) == 0)
	{
		GTP_DEBUG("I2Cfinger:%X, status err return.\r\n",finger);
		printk("I2Cfinger:%X, status err return.\r\n",finger);
		goto exit_work_func;//Coordinates are not ready, data is invalid
	}

	touch_num = finger & 0x0f;

	if (touch_num > GTP_MAX_TOUCH)
	{
		GTP_DEBUG("touch_num:%X, toomuch\r\n",touch_num);
		printk("touch_num:%X, toomuch\r\n",touch_num);
		goto exit_work_func;//Exceed the maximum support points, error exit
	}

	if (touch_num > 1)
	{
		u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10)>> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};


		ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num- 1)); 
		memcpy(&point_data[12], &buf[2], 8 * (touch_num -1));
	}
	if (touch_num)
	{
		for (i= 0; i < touch_num; i++)
		{
			coor_data= &point_data[i * 8 + 3];
			id =coor_data[0] & 0x0F;
			ts->event.x[i]= input_x  = coor_data[1] | (coor_data[2] << 8);
			ts->event.y[i]= input_y  = coor_data[3] | (coor_data[4] << 8);
			ts->event.w[i]= input_w  = coor_data[5] | (coor_data[6] << 8);

			gtp_touch_down(ts,id, input_x, input_y, input_w);
			//gtp_touch_down(ts,id);
		}

	}
	else if(pre_touch)
	{
		gtp_touch_up(ts,id);
	}
	pre_touch= touch_num;

exit_work_func:
	if(!ts->gtp_rawdiff_mode)
	{
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0)
		{
			GTP_INFO("I2C write end_cmderror!");
		}
	}
	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}
}


/*******************************************************
Function:
Timer interrupt service routine for polling mode.
Input:
timer: timer struct pointer
Output:
Timer work mode. 
HRTIMER_NORESTART: no restart mode
 *********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data,timer);


	GTP_DEBUG_FUNC();


	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000),HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}


/*******************************************************
Function:
External interrupt service routine for interrupt mode.
Input:
irq:  interrupt number.
dev_id: private data pointer
Output:
Handle Result.
IRQ_HANDLED: interrupt handled successfully
 *********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	CLEAR_EXT_INTERRUPT_FLAG();//Clean INT port interrupt flag GPE1
	gtp_irq_disable(ts);
	queue_work(goodix_wq,&ts->work);
	return IRQ_HANDLED;
}
/*******************************************************
Function:
Synchronization.
Input:
ms: synchronization time in millisecond.
Output:
None.
 *******************************************************/
void gtp_int_sync(s32 ms)
{
	GTP_GPIO_OUTPUT(gtp_int_gpio, 0);
	msleep(ms);
	GTP_GPIO_AS_INT(gtp_int_gpio);
}

/*******************************************************
Function:
Reset chip.
Input:
ms: reset time in millisecond
Output:
None.
 *******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("Guitar reset");
	GTP_GPIO_OUTPUT(gtp_rst_gpio, 0);   // begin select I2C slave addr
	msleep(ms);                        // T2: > 10ms
	// HIGH: 0x28/0x29, LOW: 0xBA/0xBB
	GTP_GPIO_OUTPUT(gtp_int_gpio, client->addr == 0x14);
	msleep(2);                         // T3: > 100us
	GTP_GPIO_OUTPUT(gtp_rst_gpio, 1); 
	msleep(10);                         // T4: > 5ms
	GTP_GPIO_AS_INPUT(gtp_rst_gpio);    // end select I2C slaveaddr
	printk("1 byte gpio en %x\n",*(volatile unsigned char *)(0x8000000016104868));
	printk("1 gpio en %x\n",*(volatile unsigned int *)(0x8000000016104008));
	gtp_int_sync(50);  
	printk("2 byte gpio en %x\n",*(volatile unsigned char *)(0x8000000016104868));
	printk("2 gpio en %x\n",*(volatile unsigned int *)(0x8000000016104008));
}

/*******************************************************
Function:
Enter sleep mode.
Input:
ts: private data.
Output:
Executive outcomes.
1: succeed, otherwise failed.
 *******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8),(u8)GTP_REG_SLEEP, 5};


	GTP_DEBUG_FUNC();


	GTP_GPIO_OUTPUT(gtp_int_gpio, 0); //First pull down the INT foot, then send the shutdown command 5
	msleep(5);

	while(retry++ < 5)
	{
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0)
		{
			GTP_INFO("GTP enter sleep!");

			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send sleep cmd failed.");
	return ret;
}

/*******************************************************
Function:
Wakeup from sleep.
Input:
ts: private data.
Output:
Executive outcomes.
>0: succeed, otherwise: failed.
 *******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();
	while(retry++ < 10)
	{ 
		GTP_GPIO_OUTPUT(gtp_int_gpio, 1); // Pull up INT foot first, then reset and wake up
		msleep(5);
		ret = gtp_i2c_test(ts->client);
		if (ret > 0)
		{
			GTP_INFO("GTP wakeup sleep.");

#if (!GTP_GESTURE_WAKEUP)
			{
				gtp_int_sync(25);        
			}
#endif

			return ret;
		}
		gtp_reset_guitar(ts->client, 20);
	}
	GTP_ERROR("GTP wakeup sleep failed.");
	return ret;
}


/*******************************************************
Function:
Initialize gtp.
Input:
ts: goodix private data
Output:
Executive outcomes.
0: succeed, otherwise: failed
 *******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	s32 i= 0;
	u8 check_sum = 0;
	u8 opr_buf[16] = {0};
	u8 sensor_id = 0;
	u8 drv_cfg_version;
	u8 flash_cfg_version;


	u8 send_cfg_buf[] = CTP_CFG_GROUP0;
	u8 cfg_info_len = GTP_CONFIG_MIN_LENGTH;
	ts->gtp_cfg_len= cfg_info_len;

	gtp_int_sync(20);//Synchronize the INT foot to read the correct configuration information
	memset(config+GTP_ADDR_LENGTH,0, GTP_CONFIG_MAX_LENGTH);
	if(gtp_i2c_read(ts->client,config, GTP_CONFIG_MIN_LENGTH+GTP_ADDR_LENGTH) < 0)
	{
		GTP_DEBUG("readgt9xx config data failed! return.\r\n");
		return-1;
	}
	GTP_DEBUG("readconfig data ok!,as follows:\r\n");
	GTP_DEBUG_ARRAY(config+GTP_ADDR_LENGTH,GTP_CONFIG_MIN_LENGTH);//Read out the original configuration information to facilitate recovery, remember to save the printed configuration information

#if GTP_DRIVER_SEND_CFG

	/*check firmware */
	ret =gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
	if(SUCCESS == ret)
	{
		if(opr_buf[0] != 0xBE)
		{
			ts->fw_error= 1;
			GTP_ERROR("Firmwareerror, no config sent!");
			return-1;
		}
	}

#if GTP_CONFIG_MODE //Configuration according to manufacturer's data sheet
	memcpy(config+GTP_ADDR_LENGTH,send_cfg_buf, ts->gtp_cfg_len);

	ret =gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0],1);//Read version number
	if(ret == SUCCESS) 
	{
		GTP_DEBUG("ConfigVersion: 0x%02X; IC Config Version: 0x%02X", \
				config[GTP_ADDR_LENGTH],opr_buf[0]);

		flash_cfg_version= opr_buf[0];
		drv_cfg_version= config[GTP_ADDR_LENGTH];

		if(flash_cfg_version < 90 && flash_cfg_version >drv_cfg_version) 
		{
			config[GTP_ADDR_LENGTH]= 0x00;//Version written to 0, mandatory update version to 0X41A version
		}
	} 
	else 
	{
		GTP_ERROR("Failedto get ic config version!No config sent!");
		return-1;
	}
#endif//GTP_CONFIG_MODE

#if GTP_CUSTOM_CFG
	config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
	config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
	config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
	config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
	config[RESOLUTION_LOC+ 4] = (u8)GTP_MAX_TOUCH;
	config[TRIGGER_LOC] &= 0xfc; 
	config[TRIGGER_LOC]|= GTP_INT_TRIGGER;
#endif // GTP_CUSTOM_CFG

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
	{
		check_sum += config[i];
	}
	config[ts->gtp_cfg_len] = (~check_sum) + 1;
	config[ts->gtp_cfg_len+1]= 1;

	ret =gtp_send_cfg(ts->client);
	if(ret < 0)
	{
		GTP_ERROR("Sendconfig error.");
	}

	ts->abs_x_max= (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
	ts->abs_y_max= (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
	ts->int_trigger_type= (config[TRIGGER_LOC]) & 0x03; 

#if GTP_CONFIG_MODE //Configuration according to manufacturer's data sheet
	if(flash_cfg_version < 90 && flash_cfg_version >drv_cfg_version) 
	{
		config[GTP_ADDR_LENGTH]= 0x41;//Version is written to 0x41, and no forced updates are made when waking up
	}
#endif//GTP_CONFIG_MODE

#else// driver not send config

	ts->abs_x_max = GTP_MAX_WIDTH;
	ts->abs_y_max= GTP_MAX_HEIGHT;
	ts->int_trigger_type= GTP_INT_TRIGGER;

#endif// GTP_DRIVER_SEND_CFG

	GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
	msleep(10);
	return 0;
}


static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
	int i;

	if (*ppos)
	{
		return 0;
	}
	ptr += sprintf(ptr, "==== GT9XX config init value====\n");

	for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
	{
		ptr += sprintf(ptr, "0x%02X ", config[i + 2]);


		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}

	ptr += sprintf(ptr, "\n");
	ptr += sprintf(ptr, "==== GT9XX config real value====\n");
	gtp_i2c_read(i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
	for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
	{
		ptr += sprintf(ptr, "0x%02X ", temp_data[i+2]);
		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}
	*ppos += ptr - page;
	return (ptr - page);
}


static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer,size_t count, loff_t *off)
{
	s32 ret = 0;
	GTP_DEBUG("write count %d\n", count);

	if (count > GTP_CONFIG_MAX_LENGTH)
	{
		GTP_ERROR("size not match [%d:%d]\n",GTP_CONFIG_MAX_LENGTH, count);
		return -EFAULT;
	}

	if (copy_from_user(&config[2], buffer, count))
	{
		GTP_ERROR("copy from user fail\n");
		return -EFAULT;
	}

	ret = gtp_send_cfg(i2c_connect_client);

	if (ret < 0)
	{
		GTP_ERROR("send config failed.");
	}

	return count;
}
/*******************************************************
Function:
Read chip version.
Input:
client:  i2c device
version: buffer to keep ic firmware version
Output:
read operation return.
2: succeed, otherwise: failed
 *******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
	s32 ret = -1;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0)
	{
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version)
	{
		*version = (buf[7] << 8) | buf[6];
	}
	if (buf[5] == 0x00)
	{
		GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2],buf[3], buf[4], buf[7], buf[6]);
	}
	else
	{
		GTP_INFO("IC Version: %c%c%c%c_%02x%02x",buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
	}
	return ret;
}

/*******************************************************
Function:
I2c test Function.
Input:
client:i2c client.
Output:
Executive outcomes.
2: succeed, otherwise failed.
 *******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA &0xff};
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	while(retry++ < 5)
	{
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0)
		{
			return ret;
		}
		GTP_ERROR("GTP i2c test failed time %d.",retry);
		msleep(10);
	}
	return ret;
}

/*******************************************************
Function:
Request gpio(INT & RST) ports.
Input:
ts: private data.
Output:
Executive outcomes.
>= 0: succeed, < 0: failed
 *******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = GTP_GPIO_REQUEST(gtp_int_gpio, "GTP INT IRQ");
	if (ret < 0) 
	{
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",(s32)gtp_int_gpio, ret);
		return -ENODEV;
	}

	//GTP_CONFIG_PIN(GTP_INT_PORT); 
	printk("byte gpio en %x\n",*(volatile unsigned char *)(0x8000000016104868));
	printk("gpio en %x\n",*(volatile unsigned int *)(0x8000000016104008));
	GTP_GPIO_OUTPUT(GTP_INT_PORT,0);//To prepare for initialization, set GT911 I2C slave address

	ts->client->irq = gpio_to_irq(GTP_INT_PORT);

	ret = GTP_GPIO_REQUEST(gtp_rst_gpio, "GTP RST PORT");
	if (ret < 0) 
	{
		GTP_ERROR("Failed to request GPIO:%d,ERRNO:%d",(s32)gtp_rst_gpio,ret);
		GTP_GPIO_FREE(gtp_int_gpio);
		return -ENODEV;
	}
	//GTP_CONFIG_PIN(GTP_RST_PORT); 
	GTP_GPIO_OUTPUT(GTP_RST_PORT,0);// To prepare for initialization, set GT911 I2C slave address

	gtp_reset_guitar(ts->client, 20); // initialization
	return ret;
}

/*******************************************************
Function:
Request interrupt.
Input:
ts: private data.
Output:
Executive outcomes.
0: succeed, -1: failed.
 *******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	//   const u8 irq_table[] = GTP_IRQ_TAB;
	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT NOMBER:0x%x, INT trigger type:%x",ts->client->irq, ts->int_trigger_type);
#if 1
	ret  = request_irq(ts->client->irq, 
			goodix_ts_irq_handler,
			0,
			ts->client->name,
			ts);
#endif
	if (ret)
	{
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(gtp_int_gpio);
		GTP_GPIO_FREE(gtp_int_gpio);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0),HRTIMER_MODE_REL);
		return -1;
	}
	else 
	{
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		return 0;
	}
}

/*******************************************************
Function:
Request input device Function.
Input:
ts:private data.
Output:
Executive outcomes.
0: succeed, otherwise: failed.
 *******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL)
	{
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif
	set_bit(ABS_X,ts->input_dev->absbit);
	set_bit(ABS_Y,ts->input_dev->absbit);
	set_bit(ABS_PRESSURE,ts->input_dev->absbit);
	set_bit(BTN_TOUCH,ts->input_dev->keybit);


	input_set_abs_params(ts->input_dev,ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev,ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev,ABS_PRESSURE, 0, 255, 0 , 0);

	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = goodix_input_phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		GTP_ERROR("Register %s input device failed",ts->input_dev->name);
		return -ENODEV;
	}


	return 0;
}

/*******************************************************
Function:
I2c probe.
Input:
client: i2c device struct.
id: device id.
Output:
Executive outcomes. 
0: succeed.
 *******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;
	u16 version_info;

	printk("call goodix_ts_probe\n\n\n");
	GTP_DEBUG_FUNC();
	//do NOT remove these logs
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	printk(KERN_WARNING "++++++++++++++%s %s %d \n",__FILE__,__FUNCTION__,__LINE__);
	i2c_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		GTP_ERROR("I2C check functionality failed.");

		printk(KERN_WARNING "++++++++++++++%s %s %d \n",__FILE__,__FUNCTION__,__LINE__);
		return -ENODEV;
	}
	printk(KERN_WARNING "++++++++++++++%s %s %d \n",__FILE__,__FUNCTION__,__LINE__);
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL)
	{
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}

	/* usegpio defined in gt9xx.h */
	gtp_rst_gpio= GTP_RST_PORT;
	gtp_int_gpio= GTP_INT_PORT;

	printk("gtp_int_gpio=%d\n", gtp_int_gpio);
	printk("gtp_rst_gpio=%d\n", gtp_rst_gpio);
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	spin_lock_init(&ts->irq_lock);         // 2.6.39 later 

	i2c_set_clientdata(client, ts);   
	ts->gtp_rawdiff_mode = 0;

	ret = gtp_request_io_port(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP request IO port failed.");
		kfree(ts);
		return ret;
	}

	ret = gtp_i2c_test(client);
	if (ret < 0)
	{
		GTP_ERROR("I2C communication ERROR!");
	}

	ret = gtp_read_version(client, &version_info);
	if (ret < 0)
	{
		GTP_ERROR("Read version failed.");
	}

	ret = gtp_init_panel(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP init panel failed.");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}

	// Create proc file system
	gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL,&config_proc_ops);
	if (gt91xx_config_proc == NULL)
	{
		GTP_ERROR("create_proc_entry %s failed\n",GT91XX_CONFIG_PROC_FILE);
	}
	else
	{
		GTP_INFO("create proc entry %s success",GT91XX_CONFIG_PROC_FILE);
	}


	ret = gtp_request_input_dev(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP request input dev failed");
	}
	//*(volatile unsigned int *)0x900000001fe10500 |= (1<<1);
	GPIO_SET_INTMODE();
	ret = gtp_request_irq(ts); 
	if (ret < 0)
	{
		GTP_INFO("GTP works in polling mode.");
	}
	else
	{
		GTP_INFO("GTP works in interrupt mode.");
	}

	//GPIO_SET_SRCGRP();

	//EXT_SET_MODE();

	CLEAR_EXT_INTERRUPT_FLAG();//Clean INT interrupt sign GPE1
	// gtp_int_sync(20);
	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}

	/*register suspend and resume fucntion*/
	gtp_register_powermanger(ts);
	return 0;
}

/*******************************************************
Function:
Goodix touchscreen driver release function.
Input:
client: i2c device struct.
Output:
Executive outcomes. 0---succeed.
 *******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	GTP_DEBUG_FUNC();
	gtp_unregister_powermanger(ts);
	if (ts) 
	{
		if (ts->use_irq)
		{
			GTP_GPIO_AS_INPUT(gtp_int_gpio);
			GTP_GPIO_FREE(gtp_int_gpio);
			free_irq(client->irq, ts);
		}
		else
		{
			hrtimer_cancel(&ts->timer);
		}
	}   
	GTP_INFO("GTP driver removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

/*******************************************************
Function:
Early suspend function.
Input:
h: early_suspend struct.
Output:
None.
 *******************************************************/
static void goodix_ts_suspend(struct goodix_ts_data *ts)
{
	s8 ret = -1;    
	GTP_DEBUG_FUNC();
	if (ts->enter_update) {
		return;
	}
	GTP_INFO("System suspend.");
	ts->gtp_is_suspend = 1;
	if (ts->use_irq)
	{
		gtp_irq_disable(ts);
	}
	else
	{
		hrtimer_cancel(&ts->timer);
	}
	ret = gtp_enter_sleep(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP early suspend failed.");
	}
	// to avoid waking up while not sleeping
	//  delay 48 + 10ms to ensure reliability    
	msleep(58);   
}

/*******************************************************
Function:
Late resume function.
Input:
h: early_suspend struct.
Output:
None.
 *******************************************************/
static void goodix_ts_resume(struct goodix_ts_data *ts)
{
	s8 ret = -1; 
	GTP_DEBUG_FUNC();
	if (ts->enter_update) {
		return;
	}
	GTP_INFO("System resume.");
	ret = gtp_wakeup_sleep(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP later resume failed.");
	}
	gtp_send_cfg(ts->client);
	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}
	else
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0),HRTIMER_MODE_REL);
	}
	ts->gtp_is_suspend = 0;
}

#if  defined(CONFIG_FB) 
/*frame buffer notifier block control the suspend/resume procedure */
static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event,void *data)
{
	struct fb_event *ev_data = data;
	struct goodix_ts_data *ts = container_of(noti, struct goodix_ts_data, notifier);
	int*blank;

	if(ev_data && ev_data->data && event == FB_EVENT_BLANK&& ts) {
		blank= ev_data->data;
		if(*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resumeby fb notifier.");
			goodix_ts_resume(ts);

		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			GTP_DEBUG("Suspendby fb notifier.");
			goodix_ts_suspend(ts);
		}
	}

	return 0;
}
#elif defined(CONFIG_PM)
/* buscontrol the suspend/resume procedure */
static int gtp_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);


	if(ts) {
		GTP_DEBUG("Suspendby i2c pm.");
		goodix_ts_suspend(ts);
	}

	return 0;
}
static int gtp_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	if(ts) {
		GTP_DEBUG("Resumeby i2c pm.");
		goodix_ts_resume(ts);
	}
	return 0;
}

static struct dev_pm_ops gtp_pm_ops = {
	.suspend= gtp_pm_suspend,
	.resume = gtp_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*earlysuspend module the suspend/resume procedure */
static void gtp_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts = container_of(h, struct goodix_ts_data, early_suspend);

	if(ts) {
		GTP_DEBUG("Suspendby earlysuspend module.");
		goodix_ts_suspend(ts);
	}
}
static void gtp_early_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts = container_of(h, struct goodix_ts_data, early_suspend);


	if(ts) {
		GTP_DEBUG("Resumeby earlysuspend module.");
		goodix_ts_resume(ts);
	} 
}
#endif

static int gtp_register_powermanger(struct goodix_ts_data *ts)
{
#if  defined(CONFIG_FB)
	ts->notifier.notifier_call= gtp_fb_notifier_callback;
	fb_register_client(&ts->notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend= goodix_ts_early_suspend;
	ts->early_suspend.resume= goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif


	return 0;
}

static int gtp_unregister_powermanger(struct goodix_ts_data *ts)
{
#if  defined(CONFIG_FB)
	fb_unregister_client(&ts->notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
	return 0;
}

/* end*/

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};
static struct of_device_id hjtgt911_tsc_of_match[] = {
	{ .compatible = "loongson,Goodix-TS", },
	{ },
};
MODULE_DEVICE_TABLE(of, hjtgt911_tsc_of_match);

static struct i2c_driver goodix_ts_driver = {
	.probe      = goodix_ts_probe,
	.remove     = goodix_ts_remove,
	.id_table   = goodix_ts_id,
	.driver = {
		.name     = GTP_I2C_NAME,
		.owner    = THIS_MODULE,
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		.pm = &gtp_pm_ops,
#endif
		.of_match_table = of_match_ptr(hjtgt911_tsc_of_match),
	},
};

/*******************************************************   
Function:
Driver Install function.
Input:
None.
Output:
Executive Outcomes. 0---succeed.
 ********************************************************/
static int goodix_ts_init(void)
{
	s32 ret;
	GTP_DEBUG_FUNC();   
	GTP_INFO("GTP driver installing...");
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq)
	{
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
	printk("befor i2c_add_driver\n");
	ret = i2c_add_driver(&goodix_ts_driver);
	printk("after i2c_add_driver ret %d\n", ret);
	return ret; 
}

/*******************************************************   
Function:
Driver uninstall function.
Input:
None.
Output:
Executive Outcomes. 0---succeed.
 ********************************************************/
static void  goodix_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
	{
		destroy_workqueue(goodix_wq);
	}
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTPSeries Driver");
MODULE_LICENSE("GPL");
