#ifndef ASUS_BATHEALTH_H
#define ASUS_BATHEALTH_H

#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/slab.h>


//ASUS_BS battery health upgrade +++
#define BAT_HEALTH_NUMBER_MAX 21

struct BAT_HEALTH_DATA {
	int magic;
	int bat_current;
	unsigned long long bat_current_avg;
	unsigned long long accumulate_time; //second
	unsigned long long accumulate_current; //uA
	int bat_health;
	unsigned long start_time;
	unsigned long end_time;
};

struct BAT_HEALTH_DATA_BACKUP {
    char date[20];
    int health;
};

#define	BATTERY_HEALTH_UPGRADE_TIME 1 //ASUS_BS battery health upgrade
#define	BATTERY_METADATA_UPGRADE_TIME 60 //ASUS_BS battery health upgrade
#define BAT_HEALTH_DATA_OFFSET  0x0
#define BAT_HEALTH_DATA_MAGIC  0x86
#define BAT_HEALTH_DATA_BACKUP_MAGIC 0x87
#define ZD553KL_DESIGNED_CAPACITY 3000 //mAh
#define BAT_HEALTH_DATA_FILE_NAME   "/factory/bat_health_binary"
#define BAT_HEALTH_DATA_SD_FILE_NAME   "/sdcard/.bh"
#define BAT_HEALTH_START_LEVEL 70
#define BAT_HEALTH_END_LEVEL 100


// asus file operations
#define FILE_OP_READ   0
#define FILE_OP_WRITE   1

#define CYCLE_COUNT_DATA_MAGIC  0x85
#define CYCLE_COUNT_FILE_NAME   "/factory/.bs"
#define BAT_PERCENT_FILE_NAME   "/factory/Batpercentage"
#define BAT_SAFETY_FILE_NAME   "/factory/bat_safety"
#define CYCLE_COUNT_SD_FILE_NAME   "/sdcard/.bs"
#define BAT_PERCENT_SD_FILE_NAME   "/sdcard/Batpercentage"
#define BAT_CYCLE_SD_FILE_NAME   "/sdcard/Batcyclecount"
#define CYCLE_COUNT_DATA_OFFSET  0x0

#define	BATTERY_SAFETY_UPGRADE_TIME 1*60 // one hour

//ASUS_BSP battery safety upgrade +++
/* Cycle Count Date Structure saved in emmc
 * magic - magic number for data verification
 * charge_cap_accum - Accumulated charging capacity
 * charge_last_soc - last saved soc before reset/shutdown
 * [0]:battery_soc [1]:system_soc [2]:monotonic_soc
 */
struct CYCLE_COUNT_DATA{
	int magic;
	int cycle_count;
	unsigned long battery_total_time;
	unsigned long high_vol_total_time;
	unsigned long high_temp_total_time;
	unsigned long high_temp_vol_time;
	u32 reload_condition;
};

#define HIGH_TEMP   350
#define HIGHER_TEMP 450
#define FULL_CAPACITY_VALUE 100
#define BATTERY_USE_TIME_CONDITION1  (12*30*24*60*60) //12Months
#define BATTERY_USE_TIME_CONDITION2  (18*30*24*60*60) //18Months
#define CYCLE_COUNT_CONDITION1  100
#define CYCLE_COUNT_CONDITION2  400
#define HIGH_TEMP_VOL_TIME_CONDITION1 (15*24*60*60)  //15Days
#define HIGH_TEMP_VOL_TIME_CONDITION2 (30*24*60*60)  //30Days
#define HIGH_TEMP_TIME_CONDITION1     (6*30*24*60*60) //6Months
#define HIGH_TEMP_TIME_CONDITION2     (12*30*24*60*60) //12Months
#define HIGH_VOL_TIME_CONDITION1     (6*30*24*60*60) //6Months
#define HIGH_VOL_TIME_CONDITION2     (12*30*24*60*60) //12Months

enum calculation_time_type {
	TOTOL_TIME_CAL_TYPE,
	HIGH_VOL_CAL_TYPE,
	HIGH_TEMP_CAL_TYPE,
	HIGH_TEMP_VOL_CAL_TYPE,
};
//ASUS_BSP battery safety upgrade ---


static void battery_health_data_reset(void);
static int batt_health_csc_backup(void);
static int resotre_bat_health(void);
static int backup_bat_health(void);
static int init_batt_cycle_count_data(void);
static void write_back_cycle_count_data(void);


#define BAT_DBG(fmt, ...) 			\
	do {					\
		pr_info(fmt, ##__VA_ARGS__);	\
	} while (0)

#endif
