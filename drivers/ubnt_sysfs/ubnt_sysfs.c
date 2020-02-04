/*
Supported SYSFS interfaces: (depends on models)
/sys/module/ubnt_platform/ethX/autoneg
/sys/module/ubnt_platform/ethX/carrier
/sys/module/ubnt_platform/ethX/change_sda
/sys/module/ubnt_platform/ethX/duplex
/sys/module/ubnt_platform/ethX/power
/sys/module/ubnt_platform/ethX/speed
/sys/module/ubnt_platform/ethX/sfp_present
/sys/module/ubnt_platform/ethX/sfp_data
/sys/module/ubnt_platform/ethX/led

/sys/module/ubnt_platform/global/fan_ctrl
/sys/module/ubnt_platform/global/freset
/sys/module/ubnt_platform/global/last_led
/sys/module/ubnt_platform/global/temp
*/

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/netdevice.h>
#include <linux/i2c.h>
#include <linux/printk.h>
#include <linux/init.h> 
#include <linux/fs.h> 
#include <linux/slab.h>
#include <linux/gpio.h>    

//Cavium related header files
#include <asm/octeon/cvmx.h>
#include <asm/octeon/cvmx-helper.h>
#include <asm/octeon/cvmx-clock.h>
#include <asm/octeon/cvmx-qlm.h>
#include <asm/octeon/cvmx-helper-bgx.h>
#include <asm/octeon/cvmx-helper-board.h>
#include <asm/octeon/cvmx-helper-cfg.h>
#include <asm/octeon/cvmx-bgxx-defs.h>
#include <asm/octeon/cvmx-gserx-defs.h>
#include <asm/octeon/cvmx-xcv-defs.h>
#include <asm/octeon/cvmx-gpio-defs.h>



struct intf_kobj {
	struct kobject kobject;

	u8 intf_idx;
	struct phy_device *phydev;

	/* status */
	u8 carrier;
	u8 autoneg;
	u16 speed;
	u8 duplex;
	u8 powered_off;

	/* ctrl */
	u8 poe;

	/* SFP */
	u8 gpio_sfp_present;
	u8 gpio_sfp_up;
	u8 gpio_sfp_i2c;
	u8 gpio_sfp_led;
	u8 sfp_present; /* !mod_def0 */
	u8 sfp_up; /* !rx_los */
	u8 sfp_i2c_data[96];
};

#define _UBNT_MAX_NUM_INTF	6
static struct intf_kobj _intf_obj[_UBNT_MAX_NUM_INTF];

struct glb_kobj {
	struct kobject kobject;
};

static struct glb_kobj _glb_obj;

//  ===============  Start here =================
//--------->global configuration
#define DEBUG
#define UBNT_SYSFS_I2C 
#define UBNT_SYSFS_MDIO
#define UBNT_SYSFS_GPIO

#define USE_ADT7475
#define USE_TMP421

struct eth_info{
	int mii_bus;
	int phy_addr;
	char eth_name[30];
};

static struct eth_info ETH_PHY_INFO[]={
	{.mii_bus = 0, .phy_addr = 4, .eth_name = "eth0" },
	{.mii_bus = 0, .phy_addr = 5, .eth_name = "eth1" },
	{.mii_bus = 0, .phy_addr = 6, .eth_name = "eth2" },
	{.mii_bus = 0, .phy_addr = 7, .eth_name = "eth3" },
	{.mii_bus = 0, .phy_addr = 8, .eth_name = "eth4" },
	{.mii_bus = 0, .phy_addr = 9, .eth_name = "eth5" },
};	

//--------->end of global configuration

#ifdef DEBUG
#define DPRINT(format, args...) printk(KERN_ERR format , ##args)
#else
#define DPRINT(format, args...)
#endif

#define ARRAYSIZE(a) (sizeof(a) / sizeof(a[0]))

void conv_digits_p_mentisa(int origin_digits, u8* result, int mentisa_digits)
{
	int len, i;
	u8  buf[20]={0};
	
	sprintf(result, "%d", origin_digits);
	
	if(!mentisa_digits)
		return;
	
	len = strlen (result);

	for(i = 0; i < len; i++)
		buf[i] = result[i];

	for(i = len - mentisa_digits; i <= len; i++){
		if(i == (len - mentisa_digits))
			result[i]='.';
		else
			result[i]=buf[i-1];
	}
	
}

struct net_device* ubnt_sysfs_get_netdev_byname(char* netdev_name)
{

	struct net_device *dev;

	read_lock(&dev_base_lock);
	dev = first_net_device(&init_net);

	while (strcmp(netdev_name,dev->name)) {
     	dev = next_net_device(dev);
	}
	
//	DPRINT( "found [%s]\n", dev->name);
	read_unlock(&dev_base_lock);

	return dev;
}

// mdio related functions
#ifdef UBNT_SYSFS_MDIO
#define UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY

#ifdef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY
#define UBNT_SYSFS_MII_BUS_NAME_0 "eth0"
#define UBNT_SYSFS_MII_BUS_NAME_1 ""
#define UBNT_SYSFS_MII_BUS_NUM 1
#else  // refine just because removing the dependencies
#define UBNT_SYSFS_MII_BUS_NAME_0 "8001180000001800"
#define UBNT_SYSFS_MII_BUS_NAME_1 "8001180000001900"
#define UBNT_SYSFS_MII_BUS_NUM 2
#endif

struct mii_bus_info{
	struct mii_bus *ubnt_sysfs_mii_bus;
	char ubnt_sysfs_mii_bus_name[30];
};

static struct mii_bus_info UBNT_SYSFS_MII_BUS[] ={
	{.ubnt_sysfs_mii_bus = NULL, .ubnt_sysfs_mii_bus_name = UBNT_SYSFS_MII_BUS_NAME_0 },
	{.ubnt_sysfs_mii_bus = NULL, .ubnt_sysfs_mii_bus_name = UBNT_SYSFS_MII_BUS_NAME_1 },
};

#ifndef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY
struct mii_bus *ubnt_sysfs_mii_bus = NULL;
extern struct mii_bus* mdiobus_find(char *name);
#endif 

int ubnt_sysfs_mii_bus_init(void)
{
#ifndef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY

	int i, ret = 0;

	for(i = 0; i < UBNT_SYSFS_MII_BUS_NUM; i++){

		UBNT_SYSFS_MII_BUS[i].ubnt_sysfs_mii_bus = mdiobus_find(UBNT_SYSFS_MII_BUS[i].ubnt_sysfs_mii_bus_name);

		if(!UBNT_SYSFS_MII_BUS[i].ubnt_sysfs_mii_bus)
			return -1;
	}
#else
	int ret = 0; 
#endif 
	return ret;

}
#ifdef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY
struct mii_bus* get_mii_bus(int eth_num)
{
	struct net_device *dev;

	dev = ubnt_sysfs_get_netdev_byname(ETH_PHY_INFO[eth_num].eth_name);

	if(dev->phydev)
		return dev->phydev->bus;
	else
		return NULL;
}
#endif

#ifdef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY
int ubnt_sysfs_mii_read(int eth_num, int phy_addr, int regnum)
{
	struct mii_bus *miibus = get_mii_bus(eth_num); 
#else
int ubnt_sysfs_mii_read(int mii_bus, int phy_addr, int regnum)
{

	struct mii_bus *miibus = UBNT_SYSFS_MII_BUS[mii_bus].ubnt_sysfs_mii_bus;
#endif
	if(!miibus)
		return -1;
	
	return mdiobus_read(miibus, phy_addr, regnum);
	
}
#ifdef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY
int ubnt_sysfs_mii_write(int eth_num, int phy_addr, int regnum, int val)
{
	struct mii_bus *miibus = get_mii_bus(eth_num); 
#else
int ubnt_sysfs_mii_write(int mii_bus, int phy_addr, int regnum, int val)
{

	struct mii_bus *miibus = UBNT_SYSFS_MII_BUS[mii_bus].ubnt_sysfs_mii_bus;
#endif	

	if(!miibus)
		return -1;
	
	return mdiobus_write(miibus, phy_addr, regnum, val);
}
#ifdef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY
int ubnt_sysfs_eth_phy_read(int eth_num, int regnum)
{
	return ubnt_sysfs_mii_read(eth_num, ETH_PHY_INFO[eth_num].phy_addr, regnum);
#else
int ubnt_sysfs_eth_phy_read(int eth_num, int regnum)
{
	return ubnt_sysfs_mii_read(ETH_PHY_INFO[eth_num].mii_bus, ETH_PHY_INFO[eth_num].phy_addr, regnum);
#endif
}


#ifdef UBNT_SYSFS_MDIO_REMOVE_DEPENDENCY
int ubnt_sysfs_eth_phy_write(int eth_num, int regnum, int val)
{
	return ubnt_sysfs_mii_write(eth_num, ETH_PHY_INFO[eth_num].phy_addr, regnum, val);
#else
int ubnt_sysfs_eth_phy_write(int eth_num, int regnum, int val)
{
	return ubnt_sysfs_mii_write(ETH_PHY_INFO[eth_num].mii_bus, ETH_PHY_INFO[eth_num].phy_addr, regnum, val);
#endif

}

#endif

// I2C related functions 

#ifdef UBNT_SYSFS_I2C

struct i2c_config_info{
	int i2c_bus_num;
	int i2c_dev_addr;
	char i2c_name[20];
};


enum i2c_dev_enum {
	SFP_DEV = 0,
	ADT7475_DEV = 1,
	TMP421_DEV = 2,
};

static struct i2c_config_info I2C_CONFIG[]={
	{.i2c_bus_num = 0, .i2c_dev_addr = 0x50 , .i2c_name = "sfp_i2c" }, 
	{.i2c_bus_num = 1, .i2c_dev_addr = 0x2e , .i2c_name = "adt7475_i2c" }, 
	{.i2c_bus_num = 1, .i2c_dev_addr = 0x4c , .i2c_name = "tmp421_i2c" }, 
};	

static struct i2c_client *ubnt_sysfs_i2c_client[] = {NULL};

static DEFINE_MUTEX(ubnt_sysfs_i2c_lock);
int ubnt_sysfs_i2c_read(struct i2c_client *pi2c_client, int reg)
{
	int ret;

	if(!pi2c_client)
		return -1;
	mutex_lock(&ubnt_sysfs_i2c_lock);
	ret = i2c_smbus_read_byte_data(pi2c_client, reg);
	mutex_unlock(&ubnt_sysfs_i2c_lock);

	return ret;
}

static int ubnt_sysfs_i2c_write(struct i2c_client *pi2c_client, int reg, u8 val)
{
	int ret;

	if(!pi2c_client)
		return -1;
	
	mutex_lock(&ubnt_sysfs_i2c_lock);
	ret = i2c_smbus_write_byte_data(pi2c_client, reg, val);
	mutex_unlock(&ubnt_sysfs_i2c_lock);

	return ret;
}

int i2c_dev_read(int dev_num, int reg)
{
	return ubnt_sysfs_i2c_read(ubnt_sysfs_i2c_client[dev_num], reg);
}

int i2c_dev_write(int dev_num, int reg, int val)
{
	return ubnt_sysfs_i2c_write(ubnt_sysfs_i2c_client[dev_num], reg, val);
}

static struct i2c_client* ubnt_sysfs_i2c_init(char *name, unsigned int bus_num, int addr)
{
	int bus = -1;
	struct i2c_adapter *pi2c_adapt = NULL;
	struct i2c_client  *pi2c_client = NULL;
	
	bus = octeon_i2c_cvmx2i2c(bus_num);
	pi2c_adapt = kmalloc(sizeof(struct i2c_adapter), GFP_ATOMIC);

	if (!pi2c_adapt){
		 /* error allocting memory ! */
		DPRINT("Error allocating i2c adaptor !! \n");
		return NULL;
	}
	
	pi2c_client = kmalloc(sizeof(struct i2c_client), GFP_ATOMIC);

	if (!pi2c_client){
		 /* error allocting memory ! */
		DPRINT("Error allocating i2c client !! \n");
		return NULL;
	}
	
	if (!(pi2c_adapt = i2c_get_adapter(bus))) {
		DPRINT("Fail to get i2c adapter %d\n", bus);
		return NULL;
	}
	
	strlcpy(pi2c_client->name, name, sizeof(pi2c_client->name));
	pi2c_client->addr = addr;
	pi2c_client->adapter = pi2c_adapt;
	
	return pi2c_client;
	
}

static void ubnt_sysfs_i2c_exit(struct i2c_client *pi2c_client)
{
	if(pi2c_client){
		if(pi2c_client->adapter){
			i2c_put_adapter(pi2c_client->adapter);
			kfree(pi2c_client->adapter);
		}
			
		kfree(pi2c_client);
	}
}

#ifdef USE_TMP421
#define TMP421_CONFIG_RANGE			0x04
#define TMP421_CONFIG_REG_1			0x09
static const u8 TMP421_TEMP_MSB[4]		= { 0x00, 0x01, 0x02, 0x03 };
static const u8 TMP421_TEMP_LSB[4]		= { 0x10, 0x11, 0x12, 0x13 };

static int temp_from_u16(u16 reg)
{
	/* Mask out status bits */
	int temp = reg & ~0xf;

	/* Add offset for extended temperature range. */
	temp -= 64 * 256;

	return (temp * 1000 + 128) / 256;
}

static int temp_from_s16(s16 reg)
{
	/* Mask out status bits */
	int temp = reg & ~0xf;

	return (temp * 1000 + 128) / 256;
}

int tmp421_get_temp(int temp_id)
{
	u16 reg;
	u8 config;
	int temp;
	
	reg  = i2c_dev_read(TMP421_DEV, TMP421_TEMP_MSB[temp_id]) << 8;
	reg |= i2c_dev_read(TMP421_DEV, TMP421_TEMP_LSB[temp_id]);

	config = i2c_dev_read(TMP421_DEV, TMP421_CONFIG_REG_1);

	if (config & TMP421_CONFIG_RANGE)
		temp = temp_from_u16(reg);
	else
		temp = temp_from_s16(reg);

	return temp;

}

#endif

#ifdef USE_ADT7475
#define REG_EXTEND1		    0x76
#define REG_EXTEND2		    0x77
#define CONFIG5_TWOSCOMP	0x01
#define REG_CONFIG5		    0x7C
#define REG_PWM_BASE		0x30
#define REG_PWM_CONFIG_BASE	0x5C
#define PWM_REG(idx) (REG_PWM_BASE + (idx))
#define PWM_CONFIG_REG(idx) (REG_PWM_CONFIG_BASE + (idx))

static int reg2temp(u8 config5, u16 reg)
{
	if (config5 & CONFIG5_TWOSCOMP) {
		if (reg >= 512)
			return (reg - 1024) * 250;
		else
			return reg * 250;
	} else
		return (reg - 256) * 250;
}

int adt7475_get_temp(int temp_id)
{
	u16 ext;
	u16 reg;
	u8 config5;

	ext = (i2c_dev_read(ADT7475_DEV, REG_EXTEND2) << 8) |
			 i2c_dev_read(ADT7475_DEV, REG_EXTEND1);

	
	config5 = i2c_dev_read(ADT7475_DEV, REG_CONFIG5); 

	reg = (i2c_dev_read(ADT7475_DEV, 0x25 + temp_id)<<2) | ((ext >> ((temp_id + 5) * 2)) & 3);

//	DPRINT("ext =%d, config5=%d, reg=%d\n", ext, config5, reg);

	return reg2temp(config5, reg);

}

static int adt7475_set_pwm_manual_mode(void)
{
	i2c_dev_write(ADT7475_DEV, PWM_CONFIG_REG(0), 0xe2);
	i2c_dev_write(ADT7475_DEV, PWM_CONFIG_REG(1), 0xe2);
	i2c_dev_write(ADT7475_DEV, PWM_CONFIG_REG(2), 0xe2);
	
	return 0;
}

static int adt7475_set_pwm(int fan_id, int pwm_val)
{
	i2c_dev_write(ADT7475_DEV, PWM_REG(fan_id), pwm_val); 
	
	return 0;
}

static int adt7475_get_pwm(int fan_id)
{
	return i2c_dev_read(ADT7475_DEV, PWM_REG(fan_id)); 		
}


#endif

#endif

// gpio related functions
#ifdef UBNT_SYSFS_GPIO
#define USE_PRIV_GPIO
#ifdef USE_PRIV_GPIO

#define RX_DAT 0x80
#define TX_SET 0x88
#define TX_CLEAR 0x90

struct gpio_config_info{
	unsigned int (*cfg_reg)(unsigned int);	
	u64 register_base;
};

static unsigned int bit_cfg_reg(unsigned int gpio)
{
	if (gpio < 16)
		return 8 * gpio;
	else
		return 8 * (gpio - 16) + 0x100;
}

static struct gpio_config_info GPIO_CONFIG ={
	.cfg_reg = bit_cfg_reg,
	.register_base = 0x8001070000000800,	
};

#endif

struct gpio_pin_info{
	u8 chip_num;
	u8 pin_num;
	u8 direction; // 1: input, 0: output
	u8 native; // 1: native , 0: external
	int out_init_val;
};

enum er_gen2_gpio_pin_assign_enum {
	POE_GLB_ENABLE = 0,
	POE_48V_SET1   = 1,
	POE_24V_SET1   = 2,
	POE_48V_SET2   = 3,
	POE_24V_SET2   = 4,
	POE_48V_SET3   = 5,
	POE_24V_SET3   = 6,
	POE_48V_SET4   = 7,
	POE_24V_SET4   = 16,
	POE_48V_SET5   = 9,
	POE_24V_SET5   = 10,
	POE_48V_DET    = 15,
	SFP_DET        = 12,
	SFP_LOS        = 13,
	SFP_FLT        = 14,
};	

static struct gpio_pin_info MISC_GPIO[] ={
	{.chip_num = 0, .pin_num = 11 , .direction = 1, .native = 1, .out_init_val = 0}, //factory reset
	{.chip_num = 0, .pin_num = 18 , .direction = 0, .native = 1, .out_init_val = 0}, //phy int
};


static struct gpio_pin_info SFP_GPIO[] ={
	{.chip_num = 0, .pin_num = 12 , .direction = 1, .native = 1, .out_init_val = 0}, //detect
	{.chip_num = 0, .pin_num = 13 , .direction = 1, .native = 1, .out_init_val = 0}, // lost
	{.chip_num = 0, .pin_num = 14 , .direction = 1, .native = 1, .out_init_val = 0}, // fault

};

static struct gpio_pin_info POE_GPIO[] ={
	{.chip_num = 0, .pin_num = 1   , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 2   , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 3   , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 4   , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 5   , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 6   , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 7   , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 16  , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 9   , .direction = 0, .native = 1, .out_init_val = 0},
	{.chip_num = 0, .pin_num = 10  , .direction = 0, .native = 1, .out_init_val = 0}, 
	{.chip_num = 0, .pin_num = 15  , .direction = 1, .native = 1, .out_init_val = 0}, // 48V det
	{.chip_num = 0, .pin_num = 0   , .direction = 0, .native = 1, .out_init_val = 0}, //GLB enable POE
};
#define GPIO_CONFIG_NUM 3
static struct gpio_pin_info *er_gen2_gpio_info[] ={&SFP_GPIO[0], &POE_GPIO[0], &MISC_GPIO[0]};
static int er_gen2_gpio_info_row_size[]={ARRAYSIZE(SFP_GPIO),ARRAYSIZE(POE_GPIO),ARRAYSIZE(MISC_GPIO)};

#ifdef USE_PRIV_GPIO

static void ubnt_sysfs_gpio_set(unsigned offset, int value)
{
	u64 mask = 1ull << offset;
	u64 reg = GPIO_CONFIG.register_base + (value ? TX_SET : TX_CLEAR);
	cvmx_write_csr(reg, mask);
}

static int ubnt_sysfs_gpio_get(unsigned offset)
{
	u64 read_bits = cvmx_read_csr(GPIO_CONFIG.register_base + RX_DAT);

	return ((1ull << offset) & read_bits) != 0;
	
}


static int ubnt_sysfs_gpio_direction_input(unsigned offset)
{
	cvmx_write_csr(GPIO_CONFIG.register_base + GPIO_CONFIG.cfg_reg(offset), 0);
	return 0;
}

static int ubnt_sysfs_gpio_direction_output(unsigned offset, int value)
{
	union cvmx_gpio_bit_cfgx cfgx;

	ubnt_sysfs_gpio_set(offset, value);

	cfgx.u64 = 0;
	cfgx.s.tx_oe = 1;

	cvmx_write_csr(GPIO_CONFIG.register_base + GPIO_CONFIG.cfg_reg(offset), cfgx.u64);
}


int ubnt_sysfs_gpio_native_set_direction(int gpio_num, int direction, int out_init_val)
{
	if(direction)
		ubnt_sysfs_gpio_direction_input(gpio_num);  
	else
		ubnt_sysfs_gpio_direction_output(gpio_num ,out_init_val);

	return 0;
	
}

int ubnt_sysfs_gpio_native_get(int gpio_num)
{
	return ubnt_sysfs_gpio_get(gpio_num); 
}
int ubnt_sysfs_gpio_native_set(int gpio_num, int val)
{
	ubnt_sysfs_gpio_set(gpio_num, val);
	return 0;
}

#else
int ubnt_sysfs_gpio_native_set_direction(int gpio_num, int direction, int out_init_val)
{
	gpio_request(gpio_num, "sysfs");

	if(direction)
		gpio_direction_input(gpio_num);  
	else
		gpio_direction_output(gpio_num ,out_init_val);

	return 0;
	
}

int ubnt_sysfs_gpio_native_get(int gpio_num)
{
	gpio_get_value(gpio_num); 
}
int ubnt_sysfs_gpio_native_set(int gpio_num, int val)
{
	gpio_set_value(gpio_num, val);
	return 0;
}
#endif

int ubnt_sysfs_gpio_init(void)
{
	int i,j;
	int gpio_num, direction, out_init_val;
	
	for(i = 0; i < ARRAYSIZE(er_gen2_gpio_info); i++)
	{
		for(j = 0; j < er_gen2_gpio_info_row_size[i] ; j++)
		{
	
			if(er_gen2_gpio_info[i][j].native){
				gpio_num  = er_gen2_gpio_info[i][j].pin_num;
				direction =  er_gen2_gpio_info[i][j].direction;
				out_init_val =er_gen2_gpio_info[i][j].out_init_val;		
				ubnt_sysfs_gpio_native_set_direction(gpio_num, direction, out_init_val);
			}
			else{
			// for external GPIO such as I2C GPIO expander
			}	
		}	
	}

	return 0;
}

int er_gen2_get_freset_status(void){

	return ubnt_sysfs_gpio_native_get(MISC_GPIO[0].pin_num);

}	


int er_gen2_set_poe_24v(int eth_num, int val){

	if(!eth_num)
		return -1;

	return ubnt_sysfs_gpio_native_set(POE_GPIO[(eth_num -1)*2 + 1].pin_num, val);

}	

int er_gen2_set_poe_48v(int eth_num, int val){

	if(!eth_num)
		return -1;

	return ubnt_sysfs_gpio_native_set(POE_GPIO[(eth_num -1)*2].pin_num, val);
	
}	

#endif

#define UBNT_SYSFS_SUPPORT
#ifdef UBNT_SYSFS_SUPPORT
#define UBNT_SYSFS_REMOVE_DEPENDENCY
#ifndef UBNT_SYSFS_REMOVE_DEPENDENCY
extern ssize_t set_sda(struct net_device *netdev, int speed, int duplex, int autoneg);
#endif

// *** porting this depends on different PHYs 
static int get_autoneg(int eth_num)
{
	int ret = 0, val;
	
	ubnt_sysfs_eth_phy_write(eth_num, 0x1f, 0x0);
	val = ubnt_sysfs_eth_phy_read(eth_num, 0);

	if(val < 0){
		DPRINT("Fail to access eth phy !! \n");
		ret = -1;
		goto leave_here;		
	}	

	if((val &= 0x1000))
		ret = 1;
leave_here:
	return ret;	
}

ssize_t get_duplex(struct net_device *netdev)
{
	if(netdev->phydev)
		return netdev->phydev->duplex;
	else
		return -1;
}
ssize_t get_speed(struct net_device *netdev)
{
	if(netdev->phydev)
		return netdev->phydev->speed;
	else
		return -1;
}
ssize_t get_carrier(struct net_device *netdev)
{
	if(netdev->phydev)
		return netdev->phydev->link;
	else
		return -1;
}

// *** porting this depends on different PHYs 
static void change_phy_sda(int eth_num, int speed, int duplex, int autoneg)
{
	int val;
	
	ubnt_sysfs_eth_phy_write(eth_num, 0x1f, 0x0);
	val = ubnt_sysfs_eth_phy_read(eth_num, 0);
	
	if(val <0){
		DPRINT("Fail to read eth phy !! \n");
		goto leave_here;
	}	

	if(autoneg)
		val |= 0x1000;			
	else
		val &= 0xefff;

	if(duplex)
		val |= 0x0100;			
	else
		val &= 0xfeff;
	
	val &= 0xdfbf;
		
	switch (speed) {
		case 0:
			break;
		case 1:
			val |= 0x2000;
			break;
		case 2:
			val |= 0x40;
			break;
		default:
			break;
	}

	if(ubnt_sysfs_eth_phy_write(eth_num, 0, val) < 0)
		DPRINT("Fail to write eth phy !! \n");
leave_here:
	return;	
}

// *** porting this depends on different PHYs
static void ethphy_power_control(int eth_num, int ctrl)
{
	int val;

	ubnt_sysfs_eth_phy_write(eth_num, 0x1f, 0x0);
	val = ubnt_sysfs_eth_phy_read(eth_num, 0);
	
	if(val <0){
		DPRINT("Fail to read eth phy !! \n");
		goto leave_here;
	}	
	if(!ctrl)
		val |= 0x800;			
	else
		val &= 0xf7ff;

	if(ubnt_sysfs_eth_phy_write(eth_num, 0, val) < 0)
		DPRINT("Fail to write eth phy !!\n");
leave_here:
	return;
}


#endif

//  ===============  End here =================

/*** sysfs ***/

/*
Description:
Show carrier status of a interface

Detail:
0 -> none carrier detected
1 -> carrier detected
*/
static ssize_t carrier_sh(struct intf_kobj *intf, char *buf)
{
	int carrier;
	struct net_device *dev;

	dev = ubnt_sysfs_get_netdev_byname(ETH_PHY_INFO[intf->intf_idx].eth_name);
	
	carrier = get_carrier(dev);
	if(carrier < 0)
		return sprintf(buf, " link is not configured !! \n");
	intf->carrier = carrier;
	return sprintf(buf, "%x\n", carrier);
}

/*
Description:
Show auto-negotiation status of a interface

Detail:
0 -> auto-negotiation off 
1 -> auto-negotiation on
*/
static ssize_t autoneg_sh(struct intf_kobj *intf, char *buf)
{
	int autoneg;
	if((autoneg = get_autoneg(intf->intf_idx))< 0)
			return 0;

	intf->autoneg = autoneg;
	
	return sprintf(buf, "%d\n", autoneg);
}

/*
Description:
Show current link speed of a interface

Detail:
10,100,1000,10000
*/
static ssize_t speed_sh(struct intf_kobj *intf, char *buf)
{
	int speed;
	struct net_device *dev;

	dev = ubnt_sysfs_get_netdev_byname(ETH_PHY_INFO[intf->intf_idx].eth_name);
	speed = get_speed(dev);
	if(speed < 0)
		return sprintf(buf, " link is not configured !! \n");
	intf->speed = speed;
	return sprintf(buf, "%d\n", speed);
}

/*
Description:
Show current duplex status of a interface

Detail:
0 -> half
1 -> full
*/
static ssize_t duplex_sh(struct intf_kobj *intf, char *buf)
{
	int duplex;
	struct net_device *dev;

	dev = ubnt_sysfs_get_netdev_byname(ETH_PHY_INFO[intf->intf_idx].eth_name);
	duplex = get_duplex(dev);

	if (duplex < 0)
		return sprintf(buf, " link is not configured !! \n");
	intf->duplex = duplex;
	return sprintf(buf, "%x\n", duplex);
}

/*
Description:
Change speed, duplex and auto-negotiation setting of a interface

Detail:
bit[0-3] -> auto-negotiation (0:off, 1:on)
bit[4-7] -> duplex (0:half, 1:full)
bit[8-11]-> speed (10,100,1000,10000)
*/
static void change_sda_st(struct intf_kobj *intf, const char *buf)
{
	int autoneg, duplex, speed, set_speed;
	int foo;
	struct net_device *dev;

	dev = ubnt_sysfs_get_netdev_byname(ETH_PHY_INFO[intf->intf_idx].eth_name);

	sscanf(buf, "%du", &foo);
	autoneg = foo & 0x0f;
	duplex  = (foo & 0xf0)>>4;
	speed   = (foo & 0xf00)>>8;

	DPRINT(" autoneg =%d, duplex =%d, speed = %d \n", autoneg, duplex, speed);
	
	change_phy_sda(intf->intf_idx, speed, duplex, autoneg);
#ifndef UBNT_SYSFS_REMOVE_DEPENDENCY
	if(set_sda(dev, speed, duplex, autoneg)){
		DPRINT(" sda change is failure !! \n");
		goto leave_here;
	}
#endif	
leave_here:
	return;
}

/*
Description:
Set power state of physical port

Detail:
0 -> power down
1 -> power up
*/
static void power_st(struct intf_kobj *intf, const char *buf)
{
	int ctrl;
	
	sscanf(buf, "%du", &ctrl);
	ethphy_power_control(intf->intf_idx, ctrl);

}

/*
Description:
Set poe power state of physical port

Detail:
0 -> both off
1 -> 24V
2 -> 48v
*/
static void poe_power_st(struct intf_kobj *intf, const char *buf)
{
	int sel;
	
	sscanf(buf, "%du", &sel);
	
	if(!sel){
		er_gen2_set_poe_48v(intf->intf_idx, 0);
		er_gen2_set_poe_24v(intf->intf_idx, 0);
	}
	else if (sel == 1){
		er_gen2_set_poe_48v(intf->intf_idx, 0);
		er_gen2_set_poe_24v(intf->intf_idx, 1);
	}
	else{
		er_gen2_set_poe_48v(intf->intf_idx, 1);
		er_gen2_set_poe_24v(intf->intf_idx, 0);
	}
	return 0;
}

/*
Description:
Show the presence of SFP module

Detail:
0 -> none SFP module inserted
1 -> SFP module inserted
*/
static ssize_t sfp_present_sh(struct intf_kobj *intf, char *buf)
{
	int val, val1, val2;
		
	val = ubnt_sysfs_gpio_native_get(SFP_GPIO[0].pin_num);
	val1 = ubnt_sysfs_gpio_native_get(SFP_GPIO[1].pin_num);
	val2 = ubnt_sysfs_gpio_native_get(SFP_GPIO[2].pin_num);

	DPRINT("SFP_DET = %x, SFP_LOS = %x, SFP_FLT = %x\n", val, val1, val2 );
	
	return 0;
}

/*
Description:
Show SFP module data of SFP port

Detail:
Dump the 96 bytes HEX raw data of eeprom from SFP module if module is inserted.
*/
static ssize_t sfp_data_sh(struct intf_kobj *intf, char *buf)
{

	int val, i;

	for(i = 0; i < 96; i++)
	{
		val = i2c_dev_read(SFP_DEV, i);
		if(val < 0)
			return DPRINT("i2c read error occurs!!\n");
		else {
			intf->sfp_i2c_data[i] = val;
		}	
	}

#ifdef DEBUG
	{
		
		for (i = 0; i < 6; i++){
			u8 *p = &intf->sfp_i2c_data[i*16];
			DPRINT(" %02x: "
				       "%02x %02x %02x %02x %02x %02x "
				       "%02x %02x %02x %02x %02x %02x "
				       "%02x %02x %02x %02x\n", i*16,
				       p[0], p[1], p[2], p[3],
				       p[4], p[5], p[6], p[7],
				       p[8], p[9], p[10], p[11],
				       p[12], p[13], p[14], p[15]);
		}
	}
	
#endif

	return 0;

}

/*
Description:
Set the led of a interface

Detail:
0 -> led blinking
1 -> led on
2 -> led off
3 -> normal mode (controlled by PHY or other MCU by default)

*/
static void led_st(struct intf_kobj *intf, const char *buf)
{
	return 0;
}


struct intf_attr {
	struct attribute attr;
	 ssize_t(*show) (struct intf_kobj *, char *buf);
	void (*store) (struct intf_kobj *, const char *buf);
};

static struct intf_attr _attr_carrier = {
	.attr = { .name = "carrier", .mode = 0444 },
	.show = carrier_sh,
};

static struct intf_attr _attr_autoneg = {
	.attr = { .name = "autoneg", .mode = 0444 },
	.show = autoneg_sh,
};

static struct intf_attr _attr_speed = {
	.attr = { .name = "speed", .mode = 0444 },
	.show = speed_sh,
};

static struct intf_attr _attr_duplex = {
	.attr = { .name = "duplex", .mode = 0444 },
	.show = duplex_sh,
};

static struct intf_attr _attr_change_sda = {
	.attr = { .name = "change_sda", .mode = 0220 },
	.store = change_sda_st,
};

static struct intf_attr _attr_power = {
	.attr = { .name = "power", .mode = 0220 },
	.store = power_st,
};

static struct intf_attr _attr_poe_power = {
	.attr = { .name = "poe_power", .mode = 0220 },
	.store = poe_power_st,
};


static struct intf_attr _attr_sfp_present = {
	.attr = { .name = "sfp_present", .mode = 0444 },
	.show = sfp_present_sh,
};

static struct intf_attr _attr_sfp_data = {
	.attr = { .name = "sfp_data", .mode = 0444 },
	.show = sfp_data_sh,
};

static struct intf_attr _attr_led = {
	.attr = { .name = "led", .mode = 0220 },
	.store = led_st,
};

static struct attribute *_intf_def_attrs_copper_def[] = {
	&_attr_carrier.attr,
	&_attr_autoneg.attr,
	&_attr_speed.attr,
	&_attr_duplex.attr,
	&_attr_power.attr,
	&_attr_poe_power.attr,
	&_attr_led.attr,
	NULL
};

static struct attribute *_intf_def_attrs_sfp_def[] = {
	&_attr_carrier.attr,
	&_attr_autoneg.attr,
	&_attr_speed.attr,
	&_attr_duplex.attr,
	&_attr_change_sda.attr,
	&_attr_power.attr,
	&_attr_poe_power.attr,
	&_attr_sfp_present.attr,
	&_attr_sfp_data.attr,
	&_attr_led.attr,
	NULL
};

static ssize_t intf_attr_sh(struct kobject *kobj, struct attribute *attr,
			    char *buf)
{
	struct intf_kobj *i = container_of(kobj, struct intf_kobj, kobject);
	struct intf_attr *a = container_of(attr, struct intf_attr, attr);

	if (!a->show) {
		return -EIO;
	}

	return a->show(i, buf);
}

static ssize_t intf_attr_st(struct kobject *kobj, struct attribute *attr,
			    const char *buf, size_t count)
{
	struct intf_kobj *i = container_of(kobj, struct intf_kobj, kobject);
	struct intf_attr *a = container_of(attr, struct intf_attr, attr);

	if (!a->store) {
		return -EIO;
	}

	a->store(i, buf);
	return count;
}

static struct sysfs_ops _intf_sysfs_ops = {
	.show = intf_attr_sh,
	.store = intf_attr_st
};

static void intf_kobj_release(struct kobject *kobj)
{
	struct intf_kobj *i = container_of(kobj, struct intf_kobj, kobject);
	kfree(i);
}

static struct kobj_type _intf_kobj_type = {
	.release = intf_kobj_release,
	.sysfs_ops = &_intf_sysfs_ops,
	.default_attrs = NULL
};

static int intf_sysfs_init(void)
{
	int i, ret;

	for (i = 0; i < _UBNT_MAX_NUM_INTF; i++) {
		memset(&_intf_obj[i], 0, sizeof(struct intf_kobj));
		_intf_obj[i].intf_idx = i;

		_intf_kobj_type.default_attrs = _intf_def_attrs_sfp_def;
		kobject_init(&_intf_obj[i].kobject, &_intf_kobj_type);
		
		if ((ret = kobject_add(&_intf_obj[i].kobject,
				       NULL /*&(THIS_MODULE->mkobj.kobj)*/,
				       "eth%d", i)) != 0) {
			/* TODO clean up existing objects */
			return ret;
		}

	}

	return 0;
}

/*** global sysfs ***/

/*
Description:
Show current temperature status

Detail:

*/
static ssize_t temp_sh(struct glb_kobj *glb, char *buf)
{
	
	int temp1, temp2, temp3 ,temp4, temp5;
	u8 temp1_buf[20]={0}, temp2_buf[20]={0}, temp3_buf[20]={0}, temp4_buf[20]={0} , temp5_buf[20]={0};
	
	temp1 = adt7475_get_temp(0);
	temp2 = adt7475_get_temp(1);
	temp3 = adt7475_get_temp(2);

	temp4 = tmp421_get_temp(0);
	temp5 = tmp421_get_temp(1);

	conv_digits_p_mentisa(temp1, &temp1_buf[0], 3);
	conv_digits_p_mentisa(temp2, &temp2_buf[0], 3);
	conv_digits_p_mentisa(temp3, &temp3_buf[0], 3);
	conv_digits_p_mentisa(temp4, &temp4_buf[0], 3);
	conv_digits_p_mentisa(temp5, &temp5_buf[0], 3);

	return sprintf(buf, "ADT7475:: temp1 = %s C, temp2 = %s C, temp3 = %s C && TMP421:: temp1 = %s C, temp2 = %s C\n", 
					temp1_buf, temp2_buf, temp3_buf, temp4_buf, temp5_buf );
}


/*
Description:
Show current status of factory reset button

Detail:
0 -> nothing happened
1 -> button pressed
*/

static ssize_t freset_sh(struct glb_kobj *glb, char *buf)
{
	int sts;
	sts =  (er_gen2_get_freset_status() != 0)? 0:1;
	return sprintf(buf, "The status of factory button  is %d \n", sts );
}

/*
Description:
Show current fan control status

Detail:
<0 - 100> if PWM fan control
<0/1> if only on/off can be controlled
*/
static ssize_t fan_ctrl_sh(struct glb_kobj *glb, char *buf)
{
	int fan1, fan2, fan3;
	u8 fan1_buf[20]={0}, fan2_buf[20]={0}, fan3_buf[20]={0};
	
	fan1 = adt7475_get_pwm(0);
	fan2 = adt7475_get_pwm(1);
	fan3 = adt7475_get_pwm(2);
	
	conv_digits_p_mentisa(fan1, &fan1_buf[0], 0);
	conv_digits_p_mentisa(fan2, &fan2_buf[0], 0);
	conv_digits_p_mentisa(fan3, &fan3_buf[0], 0);
		
	return sprintf(buf, "fan1 = %s, fan2 = %s, fan3 = %s, fan4 = %s\n", 
					fan1_buf, fan2_buf, fan3_buf, fan3_buf );
}

/*
Description:
Fan control

Detail:
<0 - 100> if PWM fan control
<0/1> if only on/off can be controlled
*/
static void fan_ctrl_st(struct glb_kobj *glb, const char *buf)
{
	int pwm_val, pwm_ctl1, pwm_ctl2, pwm_ctl3;

	sscanf(buf, "%du", &pwm_val);
	pwm_ctl1 = pwm_val & 0xff;
	pwm_ctl2 = (pwm_val & 0xff00)>>8;
	pwm_ctl3 = (pwm_val & 0xff0000)>>16;

	adt7475_set_pwm(0, pwm_ctl1);
	adt7475_set_pwm(1, pwm_ctl2);
	adt7475_set_pwm(2, pwm_ctl3);
}


struct glb_attr {
	struct attribute attr;
	ssize_t(*show) (struct glb_kobj *, char *buf);
	void (*store) (struct glb_kobj *, const char *buf);
};

static struct glb_attr _fan_ctrl_attr = {
	.attr = {.name = "fan_ctrl",.mode = 0664},
	.show = fan_ctrl_sh,
	.store = fan_ctrl_st,
};

static struct glb_attr _temp_attr = {
	.attr = {.name = "temp",.mode = 0444},
	.show = temp_sh,
};

static struct glb_attr _freset_attr = {
	.attr = {.name = "freset",.mode = 0444},
	.show = freset_sh,
};

static struct attribute *_glb_def_attrs[] = {
	&_fan_ctrl_attr.attr,
	&_temp_attr.attr,
	&_freset_attr.attr,
	NULL
};

static ssize_t glb_attr_sh(struct kobject *kobj, struct attribute *attr,
			   char *buf)
{
	struct glb_kobj *i = container_of(kobj, struct glb_kobj, kobject);
	struct glb_attr *a = container_of(attr, struct glb_attr, attr);

	if (!a->show) {
		return -EIO;
	}

	return a->show(i, buf);
}

static ssize_t glb_attr_st(struct kobject *kobj, struct attribute *attr,
		               const char *buf, size_t count)
{
	struct glb_kobj *i = container_of(kobj, struct glb_kobj, kobject);
	struct glb_attr *a = container_of(attr, struct glb_attr, attr);

	if (!a->store) {
		return -EIO;
	}

	a->store(i, buf);
	return count;
}

static struct sysfs_ops _glb_sysfs_ops = {
	.show = glb_attr_sh,
	.store = glb_attr_st
};

static void glb_kobj_release(struct kobject *kobj)
{
	struct glb_kobj *i = container_of(kobj, struct glb_kobj, kobject);
	kfree(i);
}

static struct kobj_type _glb_kobj_type = {
	.release = glb_kobj_release,
	.sysfs_ops = &_glb_sysfs_ops,
	.default_attrs = _glb_def_attrs
};

static int glb_sysfs_init(void)
{
	memset(&_glb_obj, 0, sizeof(struct glb_kobj));

	kobject_init(&_glb_obj.kobject, &_glb_kobj_type);
	return kobject_add(&_glb_obj.kobject, NULL/*&(THIS_MODULE->mkobj.kobj)*/,
			   "global");
}

extern void sfp_reset_carrier(void);
int __init ubnt_sysfs_init(void)
{
	int ret = 0 ,i;
	
	if ((ret = intf_sysfs_init()))
		return ret;

	if ((ret = glb_sysfs_init()))
		return ret;

#ifdef UBNT_SYSFS_I2C

	for(i = 0; i < ARRAYSIZE(I2C_CONFIG); i++){
		ubnt_sysfs_i2c_client[i] = ubnt_sysfs_i2c_init(I2C_CONFIG[i].i2c_name, I2C_CONFIG[i].i2c_bus_num, I2C_CONFIG[i].i2c_dev_addr); 	
		if(!ubnt_sysfs_i2c_client[i]){
			DPRINT(" I2C initilization error !! \n");
			ret = - 1;
		}
	}

	if (ret == -1)
		return -ENOMEM;

	adt7475_set_pwm_manual_mode();
	
#endif	

#ifdef UBNT_SYSFS_GPIO
	ubnt_sysfs_gpio_init();
#endif

#ifdef UBNT_SYSFS_MDIO
	ret = ubnt_sysfs_mii_bus_init();
	if(ret){
		DPRINT(" mii initilization error !! \n");
		return -ENOMEM;
	}

#endif


}

static void __exit ubnt_sysfs_exit(void)
{
#ifdef UBNT_SYSFS_I2C
	int i;
	for(i = 0; i < ARRAYSIZE(I2C_CONFIG); i++){
		ubnt_sysfs_i2c_exit(ubnt_sysfs_i2c_client[i]);
	}	
#endif
}

module_init(ubnt_sysfs_init);
module_exit(ubnt_sysfs_exit);
MODULE_LICENSE("Proprietary");
