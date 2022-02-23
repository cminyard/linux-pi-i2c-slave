// SPDX-License-Identifier: GPL-2.0+

/*
 * BCM283x I2C slave device
 *
 * Some of the Broadcom devices have an I2C slave device available.
 * It is only slave capable.  The same device can also be a SPI slave,
 * but it cannot do both at the same time.  This only implements an
 * I2C slave device, not the SPI portion.
 *
 * This device is fairly broken if you want to do a general I2C slave.
 * There is no general way to provide receive data to the master
 * device.  There is a FIFO for transmit, and you can fill that with
 * data and it will transmit it if an I2C master requests data.  But
 * there's not a non-racy way to clear the transmit FIFO.  And the
 * transmit FIFO, for some inexplicable reason, gives you interrupts
 * when you fill it up, not when it's emptied.  And there's no
 * interrupt that a transmit operation has started.  Basically,
 * providing data to the master like a normal I2C slave device is
 * almost impossible.
 *
 * The data written from the master is a slightly better situation.
 * You have a read FIFO, but it will not tell you via interrupt that a
 * single byte has been received.  The FIFO appears to be 16 bytes and
 * it will tell you if it's 1/8 full.  And there's no interrupt
 * indication of a START or STOP condition.
 *
 * This driver is not a general I2C slave driver.  It only implements
 * data written from the master, it will not provide data to the
 * master.  If the master tries to read data, it will just get zeros,
 * the device will not NAK.  And the data written must be at least two
 * bytes.  When it receives two bytes, an interrupt will trigger, the
 * interrupt is disabled and a 30us timer is started that constantly
 * monitors the device, reads data, and detects when the read
 * operation is over and synthesizes a STOP and re-enabled the
 * interrupt.
 *
 * This is sufficient to build an IPMI IPMB device receive side (you
 * will need to use another i2c device for transmitting data), which
 * is probably all this driver is good for.
 */

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/hrtimer.h>

#define DRIVER_NAME "bcm283x-i2c-slave"

struct bcm283x_i2c_slave_info {
	struct i2c_adapter adapter;
	void __iomem *regs;
	int irq;
	struct device *dev;

	struct i2c_client *slave;

	struct hrtimer timer;
};

static inline u32 get_bitval(u32 d, u32 shift, u32 mask)
{
    return (d >> shift) & mask;
}

static inline void set_bitval(u32 *d, u32 v, u32 shift, u32 mask)
{
    *d &= ~(mask << shift);
    *d |= v << shift;
}

/* Registers for the device. */
#define BCM283X_IS_DR		0x0
#define BCM283X_IS_DR_DATA(d)		get_bitval(d, 0, 0xff)
#define BCM283X_IS_DR_OE(d)		get_bitval(d, 8, 1)
#define BCM283X_IS_DR_UE(d)		get_bitval(d, 9, 1)
#define BCM283X_IS_DR_TXBUSY(d)		get_bitval(d, 16, 1)
#define BCM283X_IS_DR_RXFE(d)		get_bitval(d, 17, 1)
#define BCM283X_IS_DR_TXFF(d)		get_bitval(d, 18, 1)
#define BCM283X_IS_DR_RXFF(d)		get_bitval(d, 19, 1)
#define BCM283X_IS_DR_TXFE(d)		get_bitval(d, 20, 1)
#define BCM283X_IS_DR_RXBUSY(d)		get_bitval(d, 21, 1)
#define BCM283X_IS_DR_TXFLEVEL(d)	get_bitval(d, 22, 0x1f)
#define BCM283X_IS_DR_RXFLEVEL(d)	get_bitval(d, 27, 0x1f)

#define BCM283X_IS_RSR		0x4
#define BCM283X_IS_RSR_GET_OE(d)	get_bitval(d, 0, 1)
#define BCM283X_IS_RSR_GET_UE(d)	get_bitval(d, 1, 1)
#define BCM283X_IS_RSR_SET_OE(d,v)	set_bitval(d, v, 0, 1)
#define BCM283X_IS_RSR_SET_UE(d,v)	set_bitval(d, v, 1, 1)

#define BCM283X_IS_SLV		0x8
#define BCM283X_IS_SLV_GET(d)		get_bitval(d, 0, 0x7f)
#define BCM283X_IS_SLV_SET(d,v)		set_bitval(d, v, 0, 0x7f)

#define BCM283X_IS_CR		0xc
#define BCM283X_IS_CR_GET_EN(d)	get_bitval(d, 0, 1)
#define BCM283X_IS_CR_GET_SPI(d)	get_bitval(d, 1, 1)
#define BCM283X_IS_CR_GET_I2C(d)	get_bitval(d, 2, 1)
#define BCM283X_IS_CR_GET_CPHA(d)	get_bitval(d, 3, 1)
#define BCM283X_IS_CR_GET_CPOL(d)	get_bitval(d, 4, 1)
#define BCM283X_IS_CR_GET_ENSTAT(d)	get_bitval(d, 5, 1)
#define BCM283X_IS_CR_GET_ENCTRL(d)	get_bitval(d, 6, 1)
#define BCM283X_IS_CR_GET_BRK(d)	get_bitval(d, 7, 1)
#define BCM283X_IS_CR_GET_TXE(d)	get_bitval(d, 8, 1)
#define BCM283X_IS_CR_GET_RXE(d)	get_bitval(d, 9, 1)
#define BCM283X_IS_CR_GET_INV_RXF(d)	get_bitval(d, 10, 1)
#define BCM283X_IS_CR_GET_TESTFIFO(d)	get_bitval(d, 11, 1)
#define BCM283X_IS_CR_GET_HOSTCTRLEN(d) get_bitval(d, 12, 1)
#define BCM283X_IS_CR_GET_INV_TXF(d)	get_bitval(d, 13, 1)
#define BCM283X_IS_CR_SET_EN(d,v)	set_bitval(d, v, 0, 1)
#define BCM283X_IS_CR_SET_SPI(d,v)	set_bitval(d, v, 1, 1)
#define BCM283X_IS_CR_SET_I2C(d,v)	set_bitval(d, v, 2, 1)
#define BCM283X_IS_CR_SET_CPHA(d,v)	set_bitval(d, v, 3, 1)
#define BCM283X_IS_CR_SET_CPOL(d,v)	set_bitval(d, v, 4, 1)
#define BCM283X_IS_CR_SET_ENSTAT(d,v)	set_bitval(d, v, 5, 1)
#define BCM283X_IS_CR_SET_ENCTRL(d,v)	set_bitval(d, v, 6, 1)
#define BCM283X_IS_CR_SET_BRK(d,v)	set_bitval(d, v, 7, 1)
#define BCM283X_IS_CR_SET_TXE(d,v)	set_bitval(d, v, 8, 1)
#define BCM283X_IS_CR_SET_RXE(d,v)	set_bitval(d, v, 9, 1)
#define BCM283X_IS_CR_SET_INV_RXF(d,v)	set_bitval(d, v, 10, 1)
#define BCM283X_IS_CR_SET_TESTFIFO(d,v)	set_bitval(d, v, 11, 1)
#define BCM283X_IS_CR_SET_HOSTCTRLEN(d,v) set_bitval(d, v, 12, 1)
#define BCM283X_IS_CR_SET_INV_TXF(d,v)	set_bitval(d, v, 13, 1)

#define BCM283X_IS_FR		0x10
#define BCM283X_IS_FR_TXBUSY(d)		get_bitval(d, 0, 1)
#define BCM283X_IS_FR_RXFE(d)		get_bitval(d, 1, 1)
#define BCM283X_IS_FR_TXFF(d)		get_bitval(d, 2, 1)
#define BCM283X_IS_FR_RXFF(d)		get_bitval(d, 3, 1)
#define BCM283X_IS_FR_TXFE(d)		get_bitval(d, 4, 1)
#define BCM283X_IS_FR_RXBUSY(d)		get_bitval(d, 5, 1)
#define BCM283X_IS_FR_TXFLEVEL(d)	get_bitval(d, 6, 0x1f)
#define BCM283X_IS_FR_RXFLEVEL(d)	get_bitval(d, 11, 0x1f)

#define BCM283X_IS_IFLS		0x14
#define BCM283X_IS_IFLS_GET_TXIFLSEL(d)		get_bitval(d, 0, 0x7)
#define BCM283X_IS_IFLS_GET_RXIFLSEL(d)		get_bitval(d, 3, 0x7)
#define BCM283X_IS_IFLS_SET_TXIFLSEL(d,v)	set_bitval(d, v, 0, 0x7)
#define BCM283X_IS_IFLS_SET_RXIFLSEL(d,v)	set_bitval(d, v, 3, 0x7)

#define BCM283X_IS_IMSC		0x18
#define BCM283X_IS_RIS		0x1c
#define BCM283X_IS_MIS		0x20
#define BCM283X_IS_ICR		0x24
/* Bit for enable/mask/clear in all registers above */
#define BCM283X_IS_RX		0x1
#define BCM283X_IS_TX		0x2
#define BCM283X_IS_BI		0x4
#define BCM283X_IS_OE		0x8
#define BCM283X_IS_GET_RX(d)	get_bitval(d, 0, 1)
#define BCM283X_IS_GET_TX(d)	get_bitval(d, 1, 1)
#define BCM283X_IS_GET_BE(d)	get_bitval(d, 2, 1)
#define BCM283X_IS_GET_OE(d)	get_bitval(d, 3, 1)
#define BCM283X_IS_SET_RX(d,v)	set_bitval(d, v, 0, 1)
#define BCM283X_IS_SET_TX(d,v)	set_bitval(d, v, 1, 1)
#define BCM283X_IS_SET_BE(d,v)	set_bitval(d, v, 2, 1)
#define BCM283X_IS_SET_OE(d,v)	set_bitval(d, v, 3, 1)

#define BCM283X_IS_DMACR	0x28
#define BCM283X_IS_TDR		0x2c
#define BCM283X_IS_GPUSTAT	0x30
#define BCM283X_IS_HCTRL	0x34
#define BCM283X_IS_DEBUG1	0x38
#define BCM283X_IS_DEBUG2	0x3c

static inline void bcm283x_writereg(struct bcm283x_i2c_slave_info *is,
				    u32 reg, u32 val)
{
    writel(val, is->regs + reg);
}

static inline u32 bcm283x_readreg(struct bcm283x_i2c_slave_info *is, u32 reg)
{
    return readl(is->regs + reg);
}

static const struct of_device_id bcm283x_i2c_slave_of_ids[] = {
	{ .compatible = "brcm,bcm2835-i2c-slave", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm283x_i2c_slave_of_ids);

static u32 bcm283x_i2c_slave_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

/* We cannot do master transfers, just error out on this. */
static int bcm283x_i2c_slave_xfer(struct i2c_adapter *adapter,
				  struct i2c_msg *msgs, int num)
{
    return -ENOTSUPP;
}

static void bcm283x_i2c_slave_rx_data(struct bcm283x_i2c_slave_info *is)
{
	u32 dr = bcm283x_readreg(is, BCM283X_IS_DR);
	u8 val = 0;

	while (true) {
		if (BCM283X_IS_DR_RXFLEVEL(dr) == 0) {
			break;
		}
		val = BCM283X_IS_DR_DATA(dr);
		i2c_slave_event(is->slave, I2C_SLAVE_WRITE_RECEIVED, &val);
		dr = bcm283x_readreg(is, BCM283X_IS_DR);
	}
	if (BCM283X_IS_DR_RXBUSY(dr)) {
		/* Still receiving, restart timer. */
		hrtimer_forward_now(&is->timer, 30000);
		hrtimer_restart(&is->timer);
	} else {
		/* Done receiving, clear and restart the receive irq. */
		i2c_slave_event(is->slave, I2C_SLAVE_STOP, &val);
		bcm283x_writereg(is, BCM283X_IS_ICR, BCM283X_IS_RX);
		bcm283x_writereg(is, BCM283X_IS_IMSC, BCM283X_IS_RX);
	}
}

static irqreturn_t bcm283x_i2c_slave_isr(int irq, void *data)
{
	struct bcm283x_i2c_slave_info *is = data;
	u32 isr = bcm283x_readreg(is, BCM283X_IS_RIS);
	u32 clisr = 0;
	u8 val = 0;

	if (BCM283X_IS_GET_RX(isr)) {
		BCM283X_IS_SET_RX(&clisr, 1);

		bcm283x_writereg(is, BCM283X_IS_IMSC, 0); /* Disable the irq */

		i2c_slave_event(is->slave, I2C_SLAVE_WRITE_REQUESTED, &val);
		bcm283x_i2c_slave_rx_data(is);
	}
	bcm283x_writereg(is, BCM283X_IS_ICR, clisr);
	
	return IRQ_HANDLED;
}

static enum hrtimer_restart bcm283x_i2c_slave_timeout(struct hrtimer *t)
{
	struct bcm283x_i2c_slave_info *is;

	is = container_of(t, struct bcm283x_i2c_slave_info, timer);
	bcm283x_i2c_slave_rx_data(is);
	return HRTIMER_NORESTART;
}

static int bcm283x_i2c_slave_reg(struct i2c_client *client)
{
	struct bcm283x_i2c_slave_info *is = i2c_get_adapdata(client->adapter);
	u32 val;

	if (is->slave)
		return -EBUSY;

	is->slave = client;

	/* Set the slave address. */
	bcm283x_writereg(is, BCM283X_IS_SLV, client->addr);

	/* Interrupt FIFO level select */
	val = 0;
	BCM283X_IS_IFLS_SET_RXIFLSEL(&val, 0); /* irq on 1/8 full (2 bytes) */
	bcm283x_writereg(is, BCM283X_IS_IFLS, val);

	/* Clear error register */
	bcm283x_writereg(is, BCM283X_IS_RSR, 0);

	/* Interrupt masks, we only care about receive interrupts */
	bcm283x_writereg(is, BCM283X_IS_IMSC, BCM283X_IS_RX);

	/* Control register. */
	val = 0;
	BCM283X_IS_CR_SET_EN(&val, 1);
	BCM283X_IS_CR_SET_I2C(&val, 1);
	BCM283X_IS_CR_SET_RXE(&val, 1);
	BCM283X_IS_CR_SET_TXE(&val, 1); /* We'll just send zeros. */
	bcm283x_writereg(is, BCM283X_IS_CR, val);

	return 0;
}

static void bcm283x_i2c_slave_reset_regs(struct bcm283x_i2c_slave_info *is)
{
	bcm283x_writereg(is, BCM283X_IS_SLV, 0);
	bcm283x_writereg(is, BCM283X_IS_IFLS, 0);
	bcm283x_writereg(is, BCM283X_IS_IMSC, 0);
	bcm283x_writereg(is, BCM283X_IS_CR, 0);
	bcm283x_writereg(is, BCM283X_IS_ICR, 0xf); /* Clear all interrupts */
}

static int bcm283x_i2c_slave_unreg(struct i2c_client *client)
{
	struct bcm283x_i2c_slave_info *is = i2c_get_adapdata(client->adapter);

	if (!is->slave)
		return -EINVAL;

	bcm283x_i2c_slave_reset_regs(is);
	is->slave = NULL;
	hrtimer_cancel(&is->timer);

	return 0;
}

static const struct i2c_algorithm bcm283x_i2c_slave_algo = {
	.master_xfer = bcm283x_i2c_slave_xfer,
	.functionality = bcm283x_i2c_slave_func,
	.reg_slave	= bcm283x_i2c_slave_reg,
	.unreg_slave	= bcm283x_i2c_slave_unreg,
};

static int bcm283x_i2c_slave_probe(struct platform_device *pdev)
{
	struct bcm283x_i2c_slave_info *is = platform_get_drvdata(pdev);
	struct resource *res;
	int ret;

	is = devm_kzalloc(&pdev->dev, sizeof(*is), GFP_KERNEL);
	if (!is)
		return -ENOMEM;

	platform_set_drvdata(pdev, is);
	is->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	is->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(is->regs))
		return PTR_ERR(is->regs);

	is->irq = platform_get_irq(pdev, 0);
	if (is->irq < 0)
		return is->irq;

	hrtimer_init(&is->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	is->timer.function = bcm283x_i2c_slave_timeout;

	strscpy(is->adapter.name, pdev->name, sizeof(is->adapter.name));
	is->adapter.owner	= THIS_MODULE;
	is->adapter.algo	= &bcm283x_i2c_slave_algo;
	is->adapter.dev.parent	= &pdev->dev;
	is->adapter.nr		= pdev->id;
	is->adapter.dev.of_node	= pdev->dev.of_node;

	i2c_set_adapdata(&is->adapter, is);

	bcm283x_i2c_slave_reset_regs(is);

	ret = request_irq(is->irq, bcm283x_i2c_slave_isr,
			  IRQF_SHARED, pdev->name, is);
	if (ret) {
		dev_err(is->dev, "can't claim irq %d\n", is->irq);
		goto out;
	}

	ret = i2c_add_numbered_adapter(&is->adapter);
	if (ret < 0)
		goto out_irq_free;

	return 0;   /* Return OK */

 out_irq_free:
	if (is->irq >= 0)
		free_irq(is->irq, is);
 out:
	return ret;
}

static int bcm283x_i2c_slave_remove(struct platform_device *pdev)
{
	struct bcm283x_i2c_slave_info *is = platform_get_drvdata(pdev);

	i2c_del_adapter(&is->adapter);

	if (is->irq >= 0)
		free_irq(is->irq, is);

	bcm283x_i2c_slave_reset_regs(is);
	is->slave = NULL;

	return 0;
}

static struct platform_driver bcm283x_i2c_slave_driver = {
	.probe = bcm283x_i2c_slave_probe,
	.remove = bcm283x_i2c_slave_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = bcm283x_i2c_slave_of_ids,
	},
};

static int __init bcm283x_i2c_slave_init(void)
{
	return platform_driver_register(&bcm283x_i2c_slave_driver);
}
subsys_initcall(bcm283x_i2c_slave_init);

static void __exit bcm283x_i2c_slave_exit(void)
{
	platform_driver_unregister(&bcm283x_i2c_slave_driver);
}
module_exit(bcm283x_i2c_slave_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corey Minyard");
MODULE_DESCRIPTION("I2C slave adapter driver for BCM283x devices");
