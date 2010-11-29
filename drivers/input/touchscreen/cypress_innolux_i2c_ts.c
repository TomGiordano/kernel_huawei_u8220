/* drivers/input/touchscreen/cypress_innolux_i2c_ts.c
 *
 * Copyright (C) 2009 HUAWEI.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cypress_innolux_i2c_ts.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/vreg.h>

// #define TS_DEBUG 
#undef CYPRESS_DEBUG

#ifdef TS_DEBUG
#define CYPRESS_DEBUG(fmt, args...) printk(KERN_ERR fmt, ##args)
#else
#define CYPRESS_DEBUG(fmt, args...)
#endif

#define TS_X_OFFSET  3
#define TS_Y_OFFSET  TS_X_OFFSET

static struct workqueue_struct *cypress_wq;

struct cypress_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int use_irq;
	struct hrtimer timer;	
	int (*power)(struct i2c_client* client, int on);
	struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_ts_early_suspend(struct early_suspend *h);
static void cypress_ts_late_resume(struct early_suspend *h);
#endif

static int cypress_ts_power(struct i2c_client *client, int on);

static void cypress_ts_work_func(struct work_struct *work)
{
    int i;
	int ret;
	int bad_data = 0;	
	struct i2c_msg msg[2];
	uint8_t start_reg;
	static uint16_t last_x = 0; 
	static uint16_t last_y = 0;
	static bool is_first_point = true;
	uint8_t buf[5];
	uint16_t position[2][2];
    uint8_t finger;
  
	struct cypress_ts_data *ts = container_of(work, struct cypress_ts_data, work);

    start_reg = 0x00;
    msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	for (i = 0; i < ((ts->use_irq && !bad_data)? 1 : 5 ); i++)
	{
	  ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) 
		{
			CYPRESS_DEBUG("%d times i2c_transfer failed\n",i);
			bad_data = 1;
			continue;
		}
		if (i == 5) 
		{
			CYPRESS_DEBUG("five times i2c_transfer error\n");
		}

        finger = buf[0];
		position[0][0] = buf[2] | (uint16_t)(buf[1] & 0x03) << 8;
		position[0][1] = buf[4] | (uint16_t)(buf[3] & 0x03) << 8;
		CYPRESS_DEBUG("----x_position[0][0]=%d,y_position[0][1]=%d----\n",position[0][0],position[0][1]);	
		CYPRESS_DEBUG("----register 0x00=0x%x----\n",finger);	

		if (finger == 0x81)
        {
			if (is_first_point) 
			{
				input_report_abs(ts->input_dev, ABS_X, position[0][0]);
				input_report_abs(ts->input_dev, ABS_Y, position[0][1]);
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_sync(ts->input_dev);
				last_x = position[0][0];
				last_y = position[0][1];
				is_first_point = false;
				CYPRESS_DEBUG("----first point:x_position[0][0]=%d,y_position[0][1]=%d----\n",position[0][0],position[0][1]);	
			}
			else 
			{
				 if (((position[0][0]-last_x) >= TS_X_OFFSET) 
					     || ((last_x-position[0][0]) >= TS_X_OFFSET) 			
					     || ((position[0][1]-last_y) >= TS_Y_OFFSET) 
					     || ((last_y-position[0][1]) >= TS_Y_OFFSET)) 
				 {
					input_report_abs(ts->input_dev, ABS_X, position[0][0]);
					input_report_abs(ts->input_dev, ABS_Y, position[0][1]);
					input_report_key(ts->input_dev, BTN_TOUCH, 1);
					input_sync(ts->input_dev);
					last_x = position[0][0];
				  last_y = position[0][1];
				  CYPRESS_DEBUG("----next point:x_position[0][0]=%d,y_position[0][1]=%d----\n",position[0][0],position[0][1]);	
				}
		  }
	  }
	  else if (finger == 0x80)
	  {
	    int k;
	    is_first_point = true;
			input_report_key(ts->input_dev, BTN_TOUCH, 0);	
			input_sync(ts->input_dev);
		  for (k = 0; k < 3 ; k++)
		  {
		    ret = i2c_smbus_write_byte_data(ts->client, 0x0b, 0x80);
				if (!ret) 
					break;
				else 
					CYPRESS_DEBUG("cpt write 0x0b 0x80 failed\n");
		  }
			if (k == 3) 
				cypress_ts_power(ts->client, 1);
	  }
	}

	if (ts->use_irq)
	{
		enable_irq(ts->client->irq);
	    CYPRESS_DEBUG("cypress_ts_work_func,enable_irq\n");
	}
}

static enum hrtimer_restart cypress_ts_timer_func(struct hrtimer *timer)
{
	struct cypress_ts_data *ts = container_of(timer, struct cypress_ts_data, timer);
	CYPRESS_DEBUG("cypress_ts_timer_func\n");
	queue_work(cypress_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t cypress_ts_irq_handler(int irq, void *dev_id)
{
	struct cypress_ts_data *ts = dev_id;
	disable_irq(ts->client->irq);
 	CYPRESS_DEBUG("cypress_ts_irq_handler,disable irq\n");
	queue_work(cypress_wq, &ts->work);
	return IRQ_HANDLED;
}

static int cypress_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cypress_ts_data *ts;
	struct vreg *v_gp6;

	int ret = 0;
	int gpio_config, rc;
	int i;
	struct cypress_i2c_platform_data *pdata;
//	struct mpp *mpp_ts_reset;
  
	CYPRESS_DEBUG(" In cypress_ts_probe cypress_innolux: \n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		CYPRESS_DEBUG(KERN_ERR "cypress_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	/* power on touchscreen */
    v_gp6 = vreg_get(NULL,"gp6");
    ret = IS_ERR(v_gp6);
    if(ret) 
        return ret;

    ret = vreg_set_level(v_gp6,2800);
    if (ret)
        return ret;
    ret = vreg_enable(v_gp6);
    if (ret)
        return ret;
		
    mdelay(100);

    /* driver  detect its device  */  
	for(i = 0; i < 10; i++) 
	{		
		ret = i2c_smbus_read_byte_data(client, 0x0c);
		if (ret < 0)
			continue;
		else
			goto  succeed_find_device;
	}
	
	if ( i == 10) 
	{	
		printk(KERN_WARNING"no cypress_ts device\n ");	
		goto err_find_touchpanel_failed;
	}

succeed_find_device:
	cypress_wq = create_singlethread_workqueue("cypress_wq");
	if (!cypress_wq) 
	{
		CYPRESS_DEBUG(KERN_ERR "create cypress_wq error\n");
		return -ENOMEM;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	INIT_WORK(&ts->work, cypress_ts_work_func);

	pdata = client->dev.platform_data;

	ts->power = cypress_ts_power;
	if (ts->power) 
	{
		ret = ts->power(ts->client, 1);
		if (ret < 0) 
		{
			CYPRESS_DEBUG(KERN_ERR "cypress_ts_probe reset failed\n");
			goto err_power_failed;
		}
		msleep(200);
	}
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		CYPRESS_DEBUG(KERN_ERR "cypress_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "cypress-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, 320, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, 480, 0, 0);
	ret = input_register_device(ts->input_dev);
	if (ret) 
	{
		CYPRESS_DEBUG(KERN_ERR "cypress_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}   
	gpio_config = GPIO_CFG(57, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
	rc = gpio_tlmm_config(gpio_config, GPIO_ENABLE);
	CYPRESS_DEBUG(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, 23, rc);
	if (rc) 
		return -EIO;
	if (gpio_request(57, "cypress_ts_int\n"))
		pr_err("failed to request gpio cypress_ts_int\n");
	
	ret = gpio_configure(57, GPIOF_INPUT | IRQF_TRIGGER_FALLING);/*gpio 57is interupt for touchscreen.*/
	if (ret) 
	{
		CYPRESS_DEBUG(KERN_ERR "cypress_ts_probe: gpio_configure 57 failed\n");
		goto err_input_register_device_failed;
	}

	if (client->irq) 
	{
		ret = request_irq(client->irq, cypress_ts_irq_handler, 0, client->name, ts);		
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "cypress_ts_probe: request_irq failed\n");
	}
	if (!ts->use_irq) 
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = cypress_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cypress_ts_early_suspend;
	ts->early_suspend.resume = cypress_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	CYPRESS_DEBUG(KERN_INFO "cypress_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_find_touchpanel_failed:
err_check_functionality_failed:
	return ret;
}

static int cypress_ts_power(struct i2c_client *client, int on)
{
  int ret;
  struct mpp *mpp_ts_reset;
  CYPRESS_DEBUG("cypress_ts_power on");
  
  mpp_ts_reset = mpp_get(NULL, "mpp14");

	if (!mpp_ts_reset)
	{
		CYPRESS_DEBUG(KERN_ERR "%s: mpp14 get failed\n", __func__);
		goto err_mpp_get;
	}
	
	if (on) 
	{			
		ret = mpp_config_digital_out(mpp_ts_reset,
		      MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_LOW));	
		if (ret) 
		{
			CYPRESS_DEBUG(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
		}
		
        msleep(50);
		ret = mpp_config_digital_out(mpp_ts_reset,
		      MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_HIGH));	
		if (ret) 
		{
			CYPRESS_DEBUG(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
		}
	}
	else 
	{	  
	    ret = i2c_smbus_write_byte_data(client, 0x0a, 0x08);
		if (ret) 
		{
			CYPRESS_DEBUG(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
		}
	}	
	return ret;	
err_mpp_get:
    return 0;	
}

static int cypress_ts_remove(struct i2c_client *client)
{
	struct cypress_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int cypress_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct cypress_ts_data *ts = i2c_get_clientdata(client);
	CYPRESS_DEBUG("In cypress_ts_suspend\n");
	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
		enable_irq(client->irq);
	}
	ret = cypress_ts_power(client,0);	
	if (ret < 0) {
			CYPRESS_DEBUG(KERN_ERR "cypress_ts_probe power off failed\n");			
	}
	return 0;
}

static int cypress_ts_resume(struct i2c_client *client)
{
	int ret;
	struct cypress_ts_data *ts = i2c_get_clientdata(client);

	CYPRESS_DEBUG("In cypress_ts_resume\n");
	
	ret = cypress_ts_power(client,1);	
	if (ret < 0) 
	{
			CYPRESS_DEBUG(KERN_ERR "cypress_ts_probe power on failed\n");			
	}

	msleep(200);  /* wait for device reset; */
	
	if (ts->use_irq) 
	{
		enable_irq(client->irq);
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_ts_early_suspend(struct early_suspend *h)
{
	struct cypress_ts_data *ts;
	CYPRESS_DEBUG("cypress_ts_early_suspend\n");
	
	ts = container_of(h, struct cypress_ts_data, early_suspend);
	cypress_ts_suspend(ts->client, PMSG_SUSPEND);  
}

static void cypress_ts_late_resume(struct early_suspend *h)
{
	struct cypress_ts_data *ts;
	CYPRESS_DEBUG("cypress_ts_late_resume\n");
	
	ts = container_of(h, struct cypress_ts_data, early_suspend);
	cypress_ts_resume(ts->client);	
}
#endif

static const struct i2c_device_id cypress_ts_id[] = {
	{ CYPRESS_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver cypress_ts_driver = {
	.probe		= cypress_ts_probe,
	.remove		= cypress_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= cypress_ts_suspend,
	.resume		= cypress_ts_resume,
#endif
	.id_table	= cypress_ts_id,
	.driver = {
		.name	= CYPRESS_I2C_NAME,
	},
};

static int __devinit cypress_ts_init(void)
{
  CYPRESS_DEBUG(KERN_ERR "cypress_ts_init\n ");
	return i2c_add_driver(&cypress_ts_driver);
}

static void __exit cypress_ts_exit(void)
{
	i2c_del_driver(&cypress_ts_driver);
	if (cypress_wq)
		destroy_workqueue(cypress_wq);
}

module_init(cypress_ts_init);
module_exit(cypress_ts_exit);

MODULE_DESCRIPTION("Innolux Touchscreen Driver");
MODULE_LICENSE("GPL");
