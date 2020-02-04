#include <linux/platform_device.h>
#include <linux/of_mdio.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/phy.h>
#include <linux/memory.h>
#include <linux/mutex.h>
#include <linux/of_memory_accessor.h>
#include <linux/delay.h>
#include <linux/ctype.h>


static int vsc85x4_probe(struct phy_device *phydev)
{
	
	return 0;
}

static void vsc85x4_remove(struct phy_device *phydev)
{
	}

static int vsc85x4_config_init(struct phy_device *phydev)
{
	
	return 0;
}

static int vsc85x4_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int vsc85x4_read_status(struct phy_device *phydev)
{
		return 0;
}

static struct of_device_id vsc85x4_match[] = {
	{
		.compatible = "vitesse,vsc8504",
	},
	{
		.compatible = "vitesse,vsc8514",
	},
	{},
};
MODULE_DEVICE_TABLE(of, vsc85x4_match);

static struct phy_driver vsc8504_phy_driver = {
	.phy_id		= 0x000704c2,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Vitesse VSC8504",
	.config_init	= vsc85x4_config_init,
	.probe		= vsc85x4_probe,
	.remove		= vsc85x4_remove,
	.config_aneg	= vsc85x4_config_aneg,
	.read_status	= vsc85x4_read_status,
	.driver		= {
		.owner = THIS_MODULE,
		.of_match_table = vsc85x4_match,
	},
};

static struct phy_driver vsc8514_phy_driver = {
	.phy_id		= 0x00070670,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Vitesse VSC8514",
	.config_init	= vsc85x4_config_init,
	.probe		= vsc85x4_probe,
	.remove		= vsc85x4_remove,
	.config_aneg	= vsc85x4_config_aneg,
	.read_status	= vsc85x4_read_status,
	.driver		= {
		.owner = THIS_MODULE,
		.of_match_table = vsc85x4_match,
	},
};



static int __init vsc85x4_mod_init(void)
{
	int rv;

	rv = phy_driver_register(&vsc8504_phy_driver);
	
	if(rv)
		return rv;

	rv = phy_driver_register(&vsc8514_phy_driver);

	return rv;
}
module_init(vsc85x4_mod_init);

static void __exit vsc85x4_mod_exit(void)
{
	phy_driver_unregister(&vsc8504_phy_driver);	
	phy_driver_unregister(&vsc8514_phy_driver);	
}
module_exit(vsc85x4_mod_exit);

MODULE_DESCRIPTION("Driver for Vitesse VSC85X4 PHY");
MODULE_AUTHOR("Eric Chang");
MODULE_LICENSE("GPL");

