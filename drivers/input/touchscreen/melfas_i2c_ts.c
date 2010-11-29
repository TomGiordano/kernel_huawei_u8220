
/* drivers/input/keyboard/melfas_i2c_ts.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
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
#include <linux/melfas_i2c_ts.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/vreg.h>

#ifdef TS_DEBUG
#define MELFAS_DEBUG(fmt, args...) printk(KERN_ERR fmt, ##args)
#else
#define MELFAS_DEBUG(fmt, args...)
#endif

#define TS_X_OFFSET  3
#define TS_Y_OFFSET  TS_X_OFFSET

static struct workqueue_struct *melfas_wq;

struct melfas_ts_data {
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
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static int melfas_ts_power(struct i2c_client *client, int on);

static void melfas_ts_work_func(struct work_struct *work)
{
  int i;
	int ret;
	int bad_data = 0;	
	struct i2c_msg msg[2];
	uint8_t start_reg;
	static uint16_t last_x = 0; 
	static uint16_t last_y = 0;
	static uint8_t input_info = 0;
	static bool is_first_point = true;
	uint8_t buf[7];   
	uint16_t position[2][2];

	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);

  start_reg = 0x10;
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
			MELFAS_DEBUG("%d times i2c_transfer failed\n",i);
			bad_data = 1;
			continue;
		}
		if (i == 5) 
		{
			MELFAS_DEBUG("five times i2c_transfer error\n");
		}
		
    input_info = buf[0] & 0x07;
		position[0][0] = buf[2] | (uint16_t)(buf[1] & 0x03) << 8;
		position[0][1] = buf[4] | (uint16_t)(buf[3] & 0x03) << 8;
				
		if (input_info == 1)
    {
			if (is_first_point) 
			{
				input_report_abs(ts->input_dev, ABS_X, position[0][0]);
				input_report_abs(ts->input_dev, ABS_Y, position[0][1]);					 	 
	            // delete code
				last_x = position[0][0];
				last_y = position[0][1];
				is_first_point = false;
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
	                // delete code
					last_x = position[0][0];
				  last_y = position[0][1];
				}
		  }
	  }
	  else if (input_info == 0)
	  {
	    is_first_point = true;
	    // delete code
	  }
	  input_report_abs(ts->input_dev, ABS_PRESSURE, buf[5]);
	  input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, buf[6]);
      input_report_key(ts->input_dev, BTN_TOUCH, input_info);
	  input_sync(ts->input_dev);
	}

	if (ts->use_irq)
		enable_irq(ts->client->irq);
	MELFAS_DEBUG("melfas_ts_work_func,enable_irq\n");
}

static enum hrtimer_restart melfas_ts_timer_func(struct hrtimer *timer)
{
	struct melfas_ts_data *ts = container_of(timer, struct melfas_ts_data, timer);
	MELFAS_DEBUG("melfas_ts_timer_func\n");
	queue_work(melfas_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	struct melfas_ts_data *ts = dev_id;
	disable_irq(ts->client->irq);
 	MELFAS_DEBUG("melfas_ts_irq_handler,disable irq\n");
	queue_work(melfas_wq, &ts->work);
	return IRQ_HANDLED;
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	int ret = 0;
	int gpio_config, rc;
	int i;
	struct melfas_i2c_platform_data *pdata;
	struct mpp *mpp_ts_reset;
	struct vreg *v_gp6;
  
	MELFAS_DEBUG(" In melfas_ts_probe: \n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		MELFAS_DEBUG(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
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
    
  mpp_ts_reset = mpp_get(NULL, "mpp14");
	if (!mpp_ts_reset)
	{
		MELFAS_DEBUG(KERN_ERR "%s: mpp14 get failed\n", __func__);
		goto err_mpp_get;
	}
	ret = mpp_config_digital_out(mpp_ts_reset,
	      MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_HIGH ));	
	if (ret) 
	{
		MELFAS_DEBUG(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
	}
	
  mdelay(300);
/* driver  detect its device  */  
	for(i = 0; i < 10; i++) 
	{		
		ret = i2c_smbus_read_byte_data(client, 0x00);
		if (ret < 0)
			continue;
		else
			goto  succeed_find_device;
	}
	if ( i == 10) 
	{	
		MELFAS_DEBUG(KERN_ERR "no melfas_ts device\n ");	
		goto err_find_touchpanel_failed;
	}

succeed_find_device:
	melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!melfas_wq) 
	{
		MELFAS_DEBUG(KERN_ERR "create melfas_wq error\n");
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
	INIT_WORK(&ts->work, melfas_ts_work_func);

	pdata = client->dev.platform_data;

	ts->power = melfas_ts_power;
	if (ts->power) 
	{
		ret = ts->power(ts->client, 1);
		if (ret < 0) 
		{
			MELFAS_DEBUG(KERN_ERR "melfas_ts_probe reset failed\n");
			goto err_power_failed;
		}
		msleep(200);
	}
	
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		MELFAS_DEBUG(KERN_ERR "melfas_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "melfas-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, 320, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, 480, 0, 0);
	/* add pressure and width */
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	ret = input_register_device(ts->input_dev);
	if (ret) 
	{
		MELFAS_DEBUG(KERN_ERR "melfas_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}   
	gpio_config = GPIO_CFG(57, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
	rc = gpio_tlmm_config(gpio_config, GPIO_ENABLE);
	MELFAS_DEBUG(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, 23, rc);
	if (rc) 
		return -EIO;
	if (gpio_request(57, "melfas_ts_int\n"))
		pr_err("failed to request gpio melfas_ts_int\n");
	
	ret = gpio_configure(57, GPIOF_INPUT | IRQF_TRIGGER_FALLING);/*gpio 57is interupt for touchscreen.*/
	if (ret) 
	{
		MELFAS_DEBUG(KERN_ERR "melfas_ts_probe: gpio_configure 57 failed\n");
		goto err_input_register_device_failed;
	}

	if (client->irq) 
	{
		ret = request_irq(client->irq, melfas_ts_irq_handler, 0, client->name, ts);		
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "melfas_ts_probe: request_irq failed\n");
	}
	if (!ts->use_irq) 
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = melfas_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	MELFAS_DEBUG(KERN_INFO "melfas_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);

err_mpp_get:
err_alloc_data_failed:
err_find_touchpanel_failed:
err_check_functionality_failed:
	return ret;
}

static int melfas_ts_power(struct i2c_client *client, int on)
{
  int ret;
  struct mpp *mpp_ts_reset;
  MELFAS_DEBUG("melfas_ts_power on");
  mpp_ts_reset = mpp_get(NULL, "mpp14");
	if (!mpp_ts_reset)
	{
		MELFAS_DEBUG(KERN_ERR "%s: mpp14 get failed\n", __func__);
		goto err_mpp_get;
	}
  
	if (on) 
	{			  
		ret = mpp_config_digital_out(mpp_ts_reset,
		      MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_HIGH ));	
		if (ret) 
		{
			MELFAS_DEBUG(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
		} 
	}
	else 
	{
		ret = mpp_config_digital_out(mpp_ts_reset,
		      MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_LOW));	
		if (ret) 
		{
			MELFAS_DEBUG(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
		}       	
	}	
	return ret;	
	
err_mpp_get:
    return 0;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
	MELFAS_DEBUG("In melfas_ts_suspend\n");
	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
		enable_irq(client->irq);
	}
	ret = melfas_ts_power(client,0);	
	if (ret < 0) {
			MELFAS_DEBUG(KERN_ERR "melfas_ts_probe power off failed\n");			
	}
	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	MELFAS_DEBUG("In melfas_ts_resume\n");
	
	ret = melfas_ts_power(client,1);	
	if (ret < 0) 
	{
			MELFAS_DEBUG(KERN_ERR "melfas_ts_probe power on failed\n");			
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
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	MELFAS_DEBUG("melfas_ts_early_suspend\n");
	
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);  
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	MELFAS_DEBUG("melfas_ts_late_resume\n");
	
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);	
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver melfas_ts_driver = {
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
	.id_table	= melfas_ts_id,
	.driver = {
		.name	= MELFAS_I2C_NAME,
	},
};

static int __devinit melfas_ts_init(void)
{
  MELFAS_DEBUG(KERN_ERR "melfas_ts_init\n ");
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");


