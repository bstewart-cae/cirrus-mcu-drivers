/**
 * @file cs42l92.c
 *
 * @brief The CS42L92 Driver module
 *
 * @copyright
 * Copyright (c) Cirrus Logic 2021-2022 All Rights Reserved, http://www.cirrus.com/
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/***********************************************************************************************************************
 * INCLUDES
 **********************************************************************************************************************/
#include <stddef.h>
#include "cs42l92.h"
#include "bsp_driver_if.h"
#include "string.h"

/***********************************************************************************************************************
 * LOCAL LITERAL SUBSTITUTIONS
 **********************************************************************************************************************/

/**
 * @defgroup CS42L92_POLL_
 * @brief Polling constants for polling times and counts
 *
 * @{
 */
#define CS42L92_POLL_ACK_CTRL_MS                (10)    ///< Delay in ms between polling ACK controls
#define CS42L92_POLL_ACK_CTRL_MAX               (10)    ///< Maximum number of times to poll ACK controls
/** @} */

/**
 * @defgroup CS42L92_REGION_LOCK_
 * @brief Region lock codes
 *
 * @{
 */
#define CS42L92_REGION_UNLOCK_CODE0             (0x5555)       ///< First code required to unlock a region
#define CS42L92_REGION_UNLOCK_CODE1             (0xAAAA)       ///< Second code required to unlock a region
#define CS42L92_REGION_LOCK_CODE                (0x0)          ///< A code that will lock a region
/** @} */

/***********************************************************************************************************************
 * ENUMS, STRUCTS, UNIONS, TYPEDEFS
 **********************************************************************************************************************/

/**
 * @defgroup CS42L92_reg_sequence_
 * @brief A structure for describing a register address and value to write to it
 *
 * @{
 */
typedef struct
{
    uint32_t reg_addr;
    uint32_t reg_val;
    uint32_t delay_us;
} cs42l92_reg_sequence_t;
/** @} */

/**
 * FLL defines
 */
#define CS42L92_FLLHJ_INT_MAX_N             (1023)
#define CS42L92_FLLHJ_INT_MIN_N             (1)
#define CS42L92_FLLHJ_FRAC_MAX_N            (255)
#define CS42L92_FLLHJ_FRAC_MIN_N            (2)
#define CS42L92_FLLHJ_LP_INT_MODE_THRESH    (100000)
#define CS42L92_FLLHJ_LOW_THRESH            (192000)
#define CS42L92_FLLHJ_MID_THRESH            (1152000)
#define CS42L92_FLLHJ_MAX_THRESH            (13000000)
#define CS42L92_FLLHJ_LOW_GAINS             (0x23f0)
#define CS42L92_FLLHJ_MID_GAINS             (0x22f2)
#define CS42L92_FLLHJ_HIGH_GAINS            (0x21f0)
#define CS42L92_FLL_MAX_FOUT                (50000000)
#define CS42L92_FLL_MAX_REFDIV              (8)

#define CS42L92_FLL_CONTROL1_OFFS           (0x00)
#define CS42L92_FLL_CONTROL2_OFFS           (0x04)
#define CS42L92_FLL_CONTROL3_OFFS           (0x08)
#define CS42L92_FLL_CONTROL4_OFFS           (0x0c)
#define CS42L92_FLL_CONTROL5_OFFS           (0x10)
#define CS42L92_FLL_CONTROL6_OFFS           (0x14)

/***********************************************************************************************************************
 * LOCAL VARIABLES
 **********************************************************************************************************************/
/**
 * @defgroup CS42L92_DSP_RAM_BANK_
 * @brief CS42L92 DSP ram banks register offset array
 *
 * @{
 */
static const cs42l92_dsp_ram_bank_t cs42l92_dsp1_ram_banks[] = {
    {CS42L92_DSP1_XM_SRAM_IBUS_SETUP_1, CS42L92_DSP1_XM_SRAM_IBUS_SETUP_11},
    {CS42L92_DSP1_YM_SRAM_IBUS_SETUP_1, CS42L92_DSP1_YM_SRAM_IBUS_SETUP_6},
    {CS42L92_DSP1_PM_SRAM_IBUS_SETUP_1, CS42L92_DSP1_PM_SRAM_IBUS_SETUP_5}
};
/** @} */

/**
 * @defgroup CS42L92_DSP_RAM_BANK_
 * @brief Number of DSP ram bank entries
 *
 * @{
 */
#define N_DSP1_RAM_BANKS (sizeof(cs42l92_dsp1_ram_banks) / sizeof(cs42l92_dsp_ram_bank_t))
/** @} */

/**
 * @defgroup CS42L92_DSP_RAM_BANK_
 * @brief Flag representing both odd and even parts of a DSP ram bank
 *
 * @{
 */
#define CS42L92_DSP_RAM_BANK_ODD_EVEN           (CS42L92_DSP1_XM_SRAM_IBUS_O_EXT_N_1 \
                                               | CS42L92_DSP1_XM_SRAM_IBUS_E_EXT_N_1)
/** @} */

/**
 * CS42L92 interrupt regs to check
 *
 * Each element is in format of {irq register offset from base, mask, flag associated with this event}
 *
 * @see cs42l92_event_handler
 */
const irq_reg_t cs42l92_event_data[] =
{
    { 0x4,  CS42L92_BOOT_DONE_MASK1_MASK,         CS42L92_EVENT_FLAG_BOOT_DONE},    //< CS42L92_IRQ1_EINT_2
    { 0x0,  CS42L92_SYSCLK_FAIL_MASK1_MASK,       CS42L92_EVENT_FLAG_SYSCLK_FAIL},  //< CS42L92_IRQ1_EINT_1
    { 0x0,  CS42L92_SYSCLK_ERR_MASK1_MASK,        CS42L92_EVENT_FLAG_SYSCLK_ERR},   //< CS42L92_IRQ1_EINT_1
    { 0x0,  CS42L92_CTRLIF_ERR_MASK1_MASK,        CS42L92_EVENT_FLAG_CTRLIF_ERR},   //< CS42L92_IRQ1_EINT_1
    { 0x18, CS42L92_DSP1_MPU_ERR_MASK1_MASK,      CS42L92_EVENT_FLAG_MPU_ERR},      //< CS42L92_IRQ1_EINT_7
    { 0x20, CS42L92_DSP1_IRQ0_MASK1_MASK,         CS42L92_EVENT_FLAG_DSP1_IRQ0},    //< CS42L92_IRQ1_EINT_9
    { 0x18, CS42L92_DSP1_WDT_EXPIRE_STS1_MASK,    CS42L92_EVENT_FLAG_WDT_EXPIRE},   //< CS42L92_IRQ1_EINT_7
    { 0x18, CS42L92_DSP1_AHB_SYS_ERR_MASK1_MASK,  CS42L92_EVENT_FLAG_AHB_SYS_ERR},  //< CS42L92_IRQ1_EINT_7
    { 0x18, CS42L92_DSP1_AHB_PACK_ERR_MASK1_MASK, CS42L92_EVENT_FLAG_AHB_PACK_ERR}  //< CS42L92_IRQ1_EINT_7
};

/**
 * Number of entries in the CS42L92 interrupt regs structure
 *
 * @see cs42l92_event_handler
 */
#define N_IRQ_REGS  ((sizeof(cs42l92_event_data)) / (sizeof(irq_reg_t)))

/***********************************************************************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************************************************************/
#ifdef CS42L92_USEFUL_UNUSED
/**
 * Find if an algorithm is in the algorithm list and return true if it is.
 * Returns false if not.
 */
static bool find_algid(fw_img_info_t *fw_info, uint32_t algid_id)
{
    if (fw_info)
    {
        for (uint32_t i = 0; i < fw_info->header.alg_id_list_size; i++)
        {
            if (fw_info->alg_id_list[i] == algid_id)
                return true;
        }
    }

    return false;
}

bool cs42l92_find_algid(cs42l92_t *driver, uint32_t dsp_core, uint32_t algid_id)
{
    bool ret;

    if (dsp_core > CS42L92_NUM_DSP)
        return false;

    if (dsp_core != 0)
    {
        return find_algid(driver->dsp_info[dsp_core - 1].fw_info, algid_id);
    }
    else
    {
        // search all DSPs if dsp_core is 0
        for (uint32_t i = 0; i < CS42L92_NUM_DSP; i++)
        {
            ret = find_algid(driver->dsp_info[i].fw_info, algid_id);

            if (ret)
                return true;
        }
    }

    return false;
}
#endif

uint32_t cs42l92_find_symbol(cs42l92_t *driver, uint32_t dsp_core, uint32_t symbol_id)
{
    uint32_t ret;

    if (dsp_core > CS42L92_NUM_DSP)
        return false;

    if (dsp_core != 0)
    {
        return fw_img_find_symbol(driver->dsp_info[dsp_core - 1].fw_info, symbol_id);
    }
    else
    {
        // search all DSPs if dsp_core is 0
        for (uint32_t i = 0; i < CS42L92_NUM_DSP; i++)
        {
            ret = fw_img_find_symbol(driver->dsp_info[i].fw_info, symbol_id);

            if (ret)
                return ret;
        }
    }

    return 0;
}

/**
 * Notify the driver when the CS42L92 INTb GPIO drops low.
 *
 * This callback is registered with the BSP in the register_gpio_cb() API call.
 *
 * The primary task of this callback is to transition the driver mode from CS42L92_MODE_HANDLING_CONTROLS to
 * CS42L92_MODE_HANDLING_EVENTS, in order to signal to the main thread to process events.
 *
 * @param [in] status           BSP status for the INTb IRQ.
 * @param [in] cb_arg           A pointer to callback argument registered.  For the driver, this arg is used for a
 *                              pointer to the driver state cs42l92_t.
 *
 * @return none
 *
 * @see bsp_driver_if_t member register_gpio_cb.
 * @see bsp_callback_t
 *
 */
static void cs42l92_irq_callback(uint32_t status, void *cb_arg)
{
    cs42l92_t *d;

    d = (cs42l92_t *) cb_arg;

    if (status == BSP_STATUS_OK)
    {
        // Switch driver mode to CS42L92_MODE_HANDLING_EVENTS
        d->mode = CS42L92_MODE_HANDLING_EVENTS;
    }

    return;
}

/**
 * Reads the contents of a single register/memory address
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             32-bit address to be read
 * @param [out] val             Pointer to register value read
 *
 * @return
 * - CS42L92_STATUS_FAIL        if the call to BSP failed
 * - CS42L92_STATUS_OK          otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
uint32_t cs42l92_read_reg(cs42l92_t *driver, uint32_t addr, uint32_t *val)
{
    uint32_t ret;
    regmap_cp_config_t *cp = REGMAP_GET_CP(driver);

    ret = regmap_read(cp, addr, val);
    if (ret)
    {
        return CS42L92_STATUS_FAIL;
    }

    return CS42L92_STATUS_OK;
}

/**
 * Writes the contents of a single register/memory address
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             32-bit address to be written
 * @param [in] val              32-bit value to be written
 *
 * @return
 * - CS42L92_STATUS_FAIL        if the call to BSP failed
 * - CS42L92_STATUS_OK          otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
uint32_t cs42l92_write_reg(cs42l92_t *driver, uint32_t addr, uint32_t val)
{
    uint32_t ret;
    regmap_cp_config_t *cp = REGMAP_GET_CP(driver);

    ret = regmap_write(cp, addr, val);
    if (ret)
    {
        return CS42L92_STATUS_FAIL;
    }

    return CS42L92_STATUS_OK;
}

uint32_t cs42l92_update_reg(cs42l92_t *driver, uint32_t addr, uint32_t mask, uint32_t val)
{
    uint32_t ret;
    regmap_cp_config_t *cp = REGMAP_GET_CP(driver);

    ret = regmap_update_reg(cp, addr, mask, val);
    if (ret)
    {
        return CS42L92_STATUS_FAIL;
    }

    return CS42L92_STATUS_OK;
}

/**
 * Writes the contents of a single register/memory address that ACK's with a default value
 *
 * This performs the same function as cs42l92_write_reg, with the addition of, after writing the value to the address
 * specified, will periodically read back the register and verify that a default value is restored (0)
 * indicating the write succeeded.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             32-bit address to be written
 * @param [in] val              32-bit value to be written
 *
 * @return
 * - CS42L92_STATUS_FAIL        if the call to BSP failed or if register is never restored to 0
 * - CS42L92_STATUS_OK          otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
/* static */ uint32_t cs42l92_write_acked_reg(cs42l92_t *driver, uint32_t addr, uint32_t val)
{
    int count;
    uint32_t temp_val;

    cs42l92_write_reg(driver, addr, val);

    for (count = 0 ; count < CS42L92_POLL_ACK_CTRL_MAX; count++)
    {
        bsp_driver_if_g->set_timer(CS42L92_POLL_ACK_CTRL_MS, NULL, NULL);

        cs42l92_read_reg(driver, addr, &temp_val);
        if (temp_val == 0)
        {
            return CS42L92_STATUS_OK;
        }
    }
    return CS42L92_STATUS_FAIL;
}

/**
 * Writes the contents of multiple register/memory addresses
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] reg_sequence     An array of register/address write entries
 * @param [in] length           The length of the array
 *
 * @return
 * - CS42L92_STATUS_FAIL        if the call to BSP failed
 * - CS42L92_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_write_reg_sequence(cs42l92_t *driver, cs42l92_reg_sequence_t *reg_sequence, uint32_t length)
{
    uint32_t ret = CS42L92_STATUS_FAIL;
    cs42l92_reg_sequence_t *sequence_entry = reg_sequence;

    for (uint32_t index = 0; index < length; ++index)
    {
        // Write the next register/address in the sequence
        ret = cs42l92_write_reg(driver, sequence_entry->reg_addr, sequence_entry->reg_val);
        if (ret != CS42L92_STATUS_OK)
        {
            return CS42L92_STATUS_FAIL;
        }

        // Delay if required
        if (sequence_entry->delay_us > 0)
        {
            bsp_driver_if_g->set_timer(sequence_entry->delay_us, NULL, NULL);
        }
        ++sequence_entry;
    }

    return ret;
}

/**
 * Power up from Standby
 *
 * This function performs all necessary steps to transition the CS42L92 to be ready to generate events.
 * Completing this results in the driver transition to POWER_UP state.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the dsp info state
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - OTP_BOOT_DONE is not set
 *      - DSP Scratch register is not cleared
 * - CS42L92_STATUS_OK          otherwise
 *
 */
static uint32_t cs42l92_power_up(cs42l92_t *driver, cs42l92_dsp_t *dsp_info)
{
    uint32_t ret, val;
    cs42l92_reg_sequence_t config[] = {
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_LOCK_CONFIG,     CS42L92_REGION_UNLOCK_CODE0, 0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_LOCK_CONFIG,     CS42L92_REGION_UNLOCK_CODE1, 0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XMEM_ACCESS_0,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YMEM_ACCESS_0,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_WINDOW_ACCESS_0, 0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XREG_ACCESS_0,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YREG_ACCESS_0,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XMEM_ACCESS_1,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YMEM_ACCESS_1,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_WINDOW_ACCESS_1, 0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XREG_ACCESS_1,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YREG_ACCESS_1,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XMEM_ACCESS_2,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YMEM_ACCESS_2,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_WINDOW_ACCESS_2, 0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XREG_ACCESS_2,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YREG_ACCESS_2,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XMEM_ACCESS_3,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YMEM_ACCESS_3,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_WINDOW_ACCESS_3, 0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_XREG_ACCESS_3,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_YREG_ACCESS_3,   0x0,                         0},
        { dsp_info->base_addr + CS42L92_DSP_OFF_MPU_LOCK_CONFIG,     CS42L92_REGION_LOCK_CODE,    0}
    };

    // Lock all regions
    ret = cs42l92_write_reg_sequence(driver, config, sizeof(config) / sizeof(cs42l92_reg_sequence_t));
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    ret = cs42l92_update_reg(driver, CS42L92_DSP_CLOCK1, CS42L92_DSP_CLK_EN_MASK, CS42L92_DSP_CLK_EN);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    ret = cs42l92_read_reg(driver, CS42L92_DSP_CLOCK1, &val);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    // Get the frequency
    val = (val & CS42L92_DSP_CLK_FREQ_MASK) >> CS42L92_DSP_CLK_FREQ_SHIFT;

    // Set the DSP clock frequency
    ret = cs42l92_update_reg(driver,
                             dsp_info->base_addr + CS42L92_DSP_OFF_CLOCK_FREQ,
                             CS42L92_DSP1_CLK_FREQ_SEL_MASK,
                             val);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    // Enable the DSP
    ret = cs42l92_update_reg(driver,
                             dsp_info->base_addr + CS42L92_DSP_OFF_CCM_CORE_CONTROL,
                             CS42L92_DSP1_CCM_CORE_EN_MASK,
                             CS42L92_DSP1_CCM_CORE_EN);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    return ret;
}

/**
 * Power down to Standby
 *
 * This function performs all necessary steps to transition the CS42L92 to be in Standby power mode. This includes
 * disabling clocks to the HALO Core DSP.  Completing this results in the driver transition to CAL_STANDBY or
 * DSP_STANDBY state.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the dsp info state
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Control port activity fails
 *      - Firmware control addresses cannot be resolved by Symbol ID
 * - CS42L92_STATUS_OK          otherwise
 *
 */
static uint32_t cs42l92_power_down(cs42l92_t *driver, cs42l92_dsp_t *dsp_info)
{
    uint32_t ret;

    // Stop Watchdog Timer
    ret = cs42l92_update_reg(driver, dsp_info->base_addr + CS42L92_DSP_OFF_WDT_CONTROL, CS42L92_DSP1_WDT_EN_MASK, 0);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    // Disable DSP
    ret = cs42l92_update_reg(driver,
                             dsp_info->base_addr + CS42L92_DSP_OFF_CCM_CORE_CONTROL,
                             CS42L92_DSP1_CCM_CORE_EN_MASK,
                             0);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    ret = cs42l92_update_reg(driver, CS42L92_DSP_OFF_CORE_SOFT_RESET, CS42L92_SFT_RESET_MASK, CS42L92_SFT_RESET_MASK);

    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    ret = cs42l92_update_reg(driver, CS42L92_DSP_CLOCK1, CS42L92_DSP_CLK_EN_MASK, 0);

    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

    return ret;
}

/**
 * Memory enable
 *
 * This function performs all necessary steps to enable the memory of the DSP core on the CS42L92
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the DSP data structure
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Control port activity fails
 * - CS42L92_STATUS_OK          otherwise
 *
 */
static uint32_t cs42l92_power_mem_ena(cs42l92_t *driver, cs42l92_dsp_t *dsp_info)
{
    uint32_t ret = CS42L92_STATUS_OK;
    const cs42l92_dsp_ram_bank_t *ram_bank_ptr = dsp_info->ram_banks;

    for (uint32_t index = 0; index < dsp_info->n_ram_banks; ++index)
    {
        uint32_t reg_end = ram_bank_ptr->reg_end;
        for (uint32_t reg_addr = ram_bank_ptr->reg_start; reg_addr <= reg_end; reg_addr += 4)
        {
            ret = cs42l92_write_reg(driver, reg_addr, CS42L92_DSP_RAM_BANK_ODD_EVEN);
            if (ret != CS42L92_STATUS_OK)
            {
                return CS42L92_STATUS_FAIL;
            }
        }
        ram_bank_ptr++;
    }

    return ret;
}

/**
 * Memory disable
 *
 * This function performs all necessary steps to disable the memory of the DSP core on the CS42L92.  After
 * calling this function, the contents of DSP memory will be lost.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the DSP data structure
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Control port activity fails
 * - CS42L92_STATUS_OK          otherwise
 *
 */
static uint32_t cs42l92_power_mem_dis(cs42l92_t *driver, cs42l92_dsp_t *dsp_info)
{
    uint32_t ret = CS42L92_STATUS_OK;
    const cs42l92_dsp_ram_bank_t *ram_bank_ptr = dsp_info->ram_banks;

    for (uint32_t index = 0; index < dsp_info->n_ram_banks; ++index)
    {
        uint32_t reg_end = ram_bank_ptr->reg_end;
        for (uint32_t reg_addr = ram_bank_ptr->reg_start; reg_addr <= reg_end; reg_addr += 4)
        {
            ret = cs42l92_write_reg(driver, reg_addr, 0);
            if (ret != CS42L92_STATUS_OK)
            {
                return CS42L92_STATUS_FAIL;
            }
        }
        ram_bank_ptr++;
    }

    return ret;
}

/**
 * Handle events indicated by the IRQ pin ALERTb
 *
 * This function performs all steps to handle IRQ and other asynchronous events the driver is aware of,
 * resulting in calling of the notification callback (cs42l92_notification_callback_t).
 *
 * @param [in] driver           Pointer to the driver state
 *
 * @return
 * - CS42L92_STATUS_FAIL        Control port activity fails
 * - CS42L92_STATUS_OK          otherwise
 *
 * @see CS42L92_EVENT_FLAG_
 * @see cs42l92_notification_callback_t
 *
 */
static uint32_t cs42l92_event_handler(cs42l92_t *driver)
{
    uint32_t ret;
    uint32_t temp_reg_val;
    uint32_t old_reg = 0;
    uint32_t new_reg;

    driver->event_flags = 0;
    for (uint32_t i = 0; i < N_IRQ_REGS; i++)
    {
        new_reg = CS42L92_IRQ1_EINT_1 + cs42l92_event_data[i].irq_reg_offset;
        if (old_reg != new_reg)
        {
            ret = cs42l92_read_reg(driver, new_reg, &temp_reg_val);
            if (ret != CS42L92_STATUS_OK)
            {
                return CS42L92_STATUS_FAIL;
            }
        }
        old_reg = new_reg;

        if (temp_reg_val & cs42l92_event_data[i].mask)
        {
            driver->event_flags |= cs42l92_event_data[i].event_flag;
            ret = cs42l92_write_reg(driver,
                                    CS42L92_IRQ1_EINT_1 + cs42l92_event_data[i].irq_reg_offset,
                                    cs42l92_event_data[i].mask);
        }
    }

    return CS42L92_STATUS_OK;
}


static uint32_t cs42l92_fll_validate(cs42l92_fll_t *fll,
                                     uint32_t fin,
                                     uint32_t fout)
{
    if (!fout || !fin)
    {
        return CS42L92_STATUS_FAIL;
    }

    // Can not change output on active FLL
    if (fll->is_enabled && fout != fll->fout)
    {
        return CS42L92_STATUS_FAIL;
    }

    if (fin / CS42L92_FLL_MAX_REFDIV > CS42L92_FLLHJ_MAX_THRESH)
    {
        return CS42L92_STATUS_FAIL;
    }

    if (fout > CS42L92_FLL_MAX_FOUT)
    {
        return CS42L92_STATUS_FAIL;
    }

    return CS42L92_STATUS_OK;
}

static bool cs42l92_fll_int_osc_is_used(cs42l92_t *driver)
{
    bool is_used = false;

    for (uint32_t i = 0; i < CS42L92_NUM_FLL; ++i)
    {
        if (driver->fll[i].using_int_osc)
        {
            is_used = true;
            break;
        }
    }
    return is_used;
}

static uint32_t gcd(uint32_t n1, uint32_t n2)
{
    while (n1 != n2)
    {
        if (n1 > n2)
        {
            n1 -= n2;
        }
        else
        {
            n2 -= n1;
        }
    }
    return n1;
}

static uint32_t cs42l92_fll_do_config(cs42l92_t *driver, cs42l92_fll_t *fll)
{
    int32_t refdiv, fref, fout, lockdet_thr, fllgcd;
    bool frac = false;
    uint32_t fll_n, min_n, max_n, ratio, theta, lambda, hp, fbdiv;
    uint32_t gains, num;
    uint32_t ret;
    uint32_t fin=fll->ref_freq;

    for (refdiv = 0; refdiv < 4; refdiv++)
    {
        if ((fin / (1 << refdiv)) <= CS42L92_FLLHJ_MAX_THRESH)
        {
            break;
        }
    }

    fref = fin / (1 << refdiv);
    fout = fll->fout;
    frac = fout % fref;

    // Calc fb_div according to fref and whether frac mode
    if (fref < CS42L92_FLLHJ_LOW_THRESH)
    {
        lockdet_thr = 2;
        gains = CS42L92_FLLHJ_LOW_GAINS;
        fbdiv = (frac) ? 256 : 4;
    }
    else if (fref < CS42L92_FLLHJ_MID_THRESH)
    {
        lockdet_thr = 8;
        gains = CS42L92_FLLHJ_MID_GAINS;
        fbdiv = (frac) ? 16 : 2;
    }
    else
    {
        lockdet_thr = 8;
        gains = CS42L92_FLLHJ_HIGH_GAINS;
        fbdiv = 1;
    }

    // Use high performance mode for fractional configurations
    if (frac)
    {
        hp = 0x3;
        min_n = CS42L92_FLLHJ_FRAC_MIN_N;
        max_n = CS42L92_FLLHJ_FRAC_MAX_N;
    }
    else
    {
        if (fref < CS42L92_FLLHJ_LP_INT_MODE_THRESH)
        {
            hp = 0x0;
        }
        else
        {
            hp = 0x1;
        }
        min_n = CS42L92_FLLHJ_INT_MIN_N;
        max_n = CS42L92_FLLHJ_INT_MAX_N;
    }

    ratio = fout / fref;

    while (ratio / fbdiv < min_n)
    {
        fbdiv /= 2;
        if (fbdiv < min_n)
        {
            return CS42L92_STATUS_FAIL;
        }
    }
    while (frac && (ratio / fbdiv > max_n))
    {
        fbdiv *= 2;
        if (fbdiv >= 1024)
        {
            return CS42L92_STATUS_FAIL;
        }
    }

    // Calc N.K, theta, lambda
    fllgcd = gcd(fout, fbdiv * fref);
    num = fout / fllgcd;
    lambda = (fref * fbdiv) / fllgcd;
    fll_n = num / lambda;
    theta = num % lambda;

    // Some sanity checks before any registers are written
    if (fll_n < min_n || fll_n > max_n)
    {
        return CS42L92_STATUS_FAIL;
    }
    if (fbdiv < 1 || (frac && fbdiv >= 1024) || (!frac && fbdiv >= 256))
    {
        return CS42L92_STATUS_FAIL;
    }

    // Write lockdet_thr, phasedet, refclk_div, N to CTRL2
    ret = cs42l92_update_reg(driver,
                             fll->base + CS42L92_FLL_CONTROL2_OFFS,
                             CS42L92_FLL1_LOCKDET_THR_MASK |
                             CS42L92_FLL1_PHASEDET_MASK |
                             CS42L92_FLL1_REFCLK_DIV_MASK |
                             CS42L92_FLL1_N_MASK,
                             (lockdet_thr << CS42L92_FLL1_LOCKDET_THR_SHIFT) |
                             (1 << CS42L92_FLL1_PHASEDET_SHIFT) |
                             (refdiv << CS42L92_FLL1_REFCLK_DIV_SHIFT) |
                             (fll_n << CS42L92_FLL1_N_SHIFT));
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    // Write lambda, theta to CTRL3
    ret = cs42l92_write_reg(driver,
                            fll->base + CS42L92_FLL_CONTROL3_OFFS,
                            (lambda << CS42L92_FLL1_LAMBDA_SHIFT) |
                            (theta << CS42L92_FLL1_THETA_SHIFT));
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    // Write gain_coarse, hp, fb_div to CTRL4
    ret = cs42l92_update_reg(driver,
                             fll->base + CS42L92_FLL_CONTROL4_OFFS,
                             (0xffff << CS42L92_FLL1_FD_GAIN_COARSE_SHIFT) |
                             CS42L92_FLL1_HP_MASK |
                             CS42L92_FLL1_FB_DIV_MASK,
                             (gains << CS42L92_FLL1_FD_GAIN_COARSE_SHIFT) |
                             (hp << CS42L92_FLL1_HP_SHIFT) |
                             (fbdiv << CS42L92_FLL1_FB_DIV_SHIFT));
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    return ret;
}

static uint32_t cs42l92_fll_apply_config(cs42l92_t *driver, cs42l92_fll_t *fll,
                                         bool already_enabled)
{
    uint32_t ret;

   if (already_enabled)
   {
        if (fll->is_hold != true)
        {
            // FLLn_HOLD must be set before configuring any registers
            ret = cs42l92_update_reg(driver,
                                    fll->base + CS42L92_FLL_CONTROL1_OFFS,
                                    CS42L92_FLL1_HOLD_MASK, CS42L92_FLL1_HOLD);
            if (ret == CS42L92_STATUS_FAIL)
            {
                return ret;
            }
            fll->is_hold = true;
        }
   }

    ret = cs42l92_fll_do_config(driver, fll);
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    ret = cs42l92_update_reg(driver,
                             fll->base + CS42L92_FLL_CONTROL2_OFFS,
                             CS42L92_FLL1_REFCLK_SRC_MASK,
                             fll->ref_src << CS42L92_FLL1_REFCLK_SRC_SHIFT);
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    return ret;
}

static uint32_t cs42l92_fll_init(cs42l92_t *driver,
                                 uint32_t fll_id)
{
    uint32_t ret = CS42L92_STATUS_OK;
    cs42l92_fll_t *fll = &driver->fll[fll_id];

    fll->id = fll_id;
    fll->ref_src = CS42L92_FLL_SRC_NO_INPUT;
    fll->using_int_osc = false;
    fll->sts_addr = CS42L92_IRQ1_STS_6;

    fll->is_enabled = false;

    switch (fll_id)
    {
    case CS42L92_FLL1:
        fll->base = CS42L92_FLL1_CONTROL1;
        fll->sts_mask = CS42L92_FLL1_LOCK_STS1_MASK;
        // Set hold status according to data sheet
        fll->is_hold = true;
        break;
    case CS42L92_FLL2:
        fll->base = CS42L92_FLL2_CONTROL1;
        fll->sts_mask = CS42L92_FLL2_LOCK_STS1_MASK;
        fll->is_hold = false;
        break;
    default:
        ret = CS42L92_STATUS_FAIL;
        break;
    }

    return ret;
}

static uint32_t cs42l92_common_patch(cs42l92_t *driver)
{
    uint32_t temp_reg_val;
    uint32_t ret;
    uint32_t iter_timeout = 0;

    cs42l92_write_reg(driver, 0x0808, 0x0002);
    do
    {
        bsp_driver_if_g->set_timer(5, NULL, NULL);
        ret = cs42l92_read_reg(driver, 0x0804, &temp_reg_val);
        if (ret != CS42L92_STATUS_OK)
        {
            return CS42L92_STATUS_FAIL;
        }
        iter_timeout++;
        if (iter_timeout > 20)
        {
            return CS42L92_STATUS_FAIL;
        }
    } while ((temp_reg_val & 0x2) != 0x2);
    cs42l92_write_reg(driver, 0x0808, 0x0003);
    cs42l92_write_block(driver, 0x410ac, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} , 8);
    cs42l92_write_block(driver, 0x4c8a0, (uint8_t[]){0x00, 0x4D, 0x68, 0x0B, 0x69, 0x0B, 0x9F, 0x00, \
                                                     0x42, 0x00, 0x00, 0x4D, 0x00, 0x4D, 0x69, 0x0B, \
                                                     0x38, 0x0F, 0x40, 0x00, 0x00, 0x00, 0x4D, 0x68, \
                                                     0x78, 0x08, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x26, \
                                                     0x88, 0x10, 0x00, 0x0E, 0x00, 0x01, 0x00, 0x01 \
                                                    } , 40);
    cs42l92_write_block(driver, 0x4c8d0, (uint8_t[]){0x4D, 0x68, 0x38, 0x0F, 0x0F, 0x80, 0x00, 0x00, \
                                                     0x00, 0x4D, 0x69, 0x04, 0x28, 0x0F, 0x02, 0x00, \
                                                     0x00, 0x00, 0x4D, 0x69, 0x68, 0x00, 0x0F, 0x20, \
                                                     0x00, 0x00, 0x00, 0x4D, 0x4D, 0x68, 0x08, 0x0F, \
                                                     0x0F, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x68, 0x20, \
                                                     0x08, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x26, 0x78, \
                                                     0x00, 0x00, 0x00, 0x02} , 52);
    cs42l92_write_block(driver, 0x4c910, (uint8_t[]){0x48, 0x14, 0x10, 0x1F, 0x01, 0x04, 0x00, 0x00, \
                                                     0x4C, 0xA4, 0x08, 0x0F, 0x1F, 0x02, 0x00, 0x00, \
                                                     0x00, 0x4D, 0x68, 0x38, 0x1F, 0x01, 0x80, 0x00, \
                                                     0x00, 0x4D, 0x69, 0x04, 0x1F, 0x01, 0x02, 0x00, \
                                                     0x00, 0x4D, 0x69, 0x28, 0x0F, 0x01, 0x20, 0x00, \
                                                     0x00, 0x4D, 0x78, 0x10, 0x30, 0x0F, 0x04, 0x00, \
                                                     0x00, 0x00, 0x4D, 0x68, 0x73, 0x08, 0x0F, 0x40, \
                                                     0x02, 0x00, 0x00, 0x4D, 0x4D, 0x68, 0x18, 0x1F, \
                                                     0x01, 0x00, 0x00, 0x00, 0x4D, 0x78, 0x08, 0x0F, \
                                                     0x1F, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x69, 0x04, \
                                                     0x1F, 0x01, 0x00, 0x00, 0x00, 0x4D, 0x68, 0x00, \
                                                     0x1F, 0x01, 0x00, 0x00, 0x00, 0x4D, 0x68, 0x08, \
                                                     0x1F, 0x01, 0x00, 0x00, 0x00, 0x4D, 0x6A, 0x08, \
                                                     0x1F, 0x01, 0x00, 0x00, 0x00, 0x4D, 0x68, 0x20, \
                                                     0x1F, 0x01, 0x00, 0x00, 0x00, 0x48, 0x14, 0x00, \
                                                     0x1F, 0x01, 0x00, 0x00, 0x00, 0x4D, 0x68, 0x10, \
                                                     0x1F, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x78, 0x00, \
                                                     0x1F, 0x01, 0x01, 0x00, 0x00, 0x4D, 0x78, 0x10, \
                                                     0x1F, 0x01, 0x00, 0x00, 0x00, 0x4D, 0x68, 0x38, \
                                                     0x0F, 0x01, 0x00, 0x00, 0x00, 0x4D, 0x69, 0x30, \
                                                     0x10, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x48, 0x14, \
                                                     0xA4, 0x08, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x4C, \
                                                     0x26, 0x78, 0x08, 0x1F, 0x01, 0x02, 0x00, 0x00, \
                                                     0x26, 0x78, 0x00, 0x0F, 0x1F, 0x00, 0x00, 0x00, \
                                                     0x00, 0x24, 0xAD, 0x00, 0x0F, 0x01, 0x01, 0x00, \
                                                     0x01, 0x88, 0x10, 0x08, 0x00, 0x0F, 0x00, 0x00, \
                                                     0x00, 0x01, 0x88, 0x10, 0x24, 0x31, 0x0A, 0x00, \
                                                     0x00, 0x80, 0x00, 0x00 \
                                                    } , 220);
    cs42l92_write_block(driver, 0x4108c, (uint8_t[]){0x49, 0x00, 0x40, 0x2F, 0x48, 0xA0, 0x48, 0x10, \
                                                     0x00, 0xAE, 0x00, 0xD0} , 12);
    cs42l92_write_reg(driver, 0x0808, 0x0002);
    cs42l92_write_reg(driver, 0x0808, 0x0000);
    return CS42L92_STATUS_OK;
}

static uint32_t cs42l92_otpid_8_patch(cs42l92_t *driver)
{
    cs42l92_write_reg(driver, 0x0030, 0x0055);
    cs42l92_write_reg(driver, 0x0030, 0x00aa);
    cs42l92_write_reg(driver, 0x0034, 0x0055);
    cs42l92_write_reg(driver, 0x0034, 0x00aa);
    cs42l92_write_reg(driver, 0x4d68, 0x1db10000);
    cs42l92_write_reg(driver, 0x4d70, 0x700249b8);
    cs42l92_write_reg(driver, 0x24ac, 0x10000);
    cs42l92_write_reg(driver, 0x24b4, 0x05ff);
    cs42l92_write_reg(driver, 0x2420, 0x4150415);
    cs42l92_write_reg(driver, 0x2424, 0x0415);
    cs42l92_write_reg(driver, 0x0030, 0x00cc);
    cs42l92_write_reg(driver, 0x0030, 0x0033);
    cs42l92_write_reg(driver, 0x0034, 0x00cc);
    cs42l92_write_reg(driver, 0x0034, 0x0033);

    return CS42L92_STATUS_OK;
}

#ifdef CS42L92_ADC_STANDARD_MODE
static uint32_t cs42l92_adc_support_patch(cs42l92_t *driver)
{
// * The following register writes follow the CS42L92 datasheet to program the device for Standard Mode ADC Support
    cs42l92_write_reg(driver, 0x0808, 0x0002);
    cs42l92_write_reg(driver, 0x0808, 0x0003);
    cs42l92_write_block(driver, 0x410ac, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8);
    cs42l92_write_block(driver, 0x4c800, (uint8_t[]){0x46, 0x84, 0x02, 0x0F, 0x0F, 0x05, 0x00, 0x00, \
                                                     0x00, 0x46, 0x20, 0x30, 0x30, 0x0F, 0x40, 0x00, \
                                                     0x00, 0x00, 0x46, 0x28, 0x30, 0x30, 0x0F, 0x40, \
                                                     0x40, 0x00, 0x00, 0x46, 0x46, 0x38, 0x30, 0x0F, \
                                                     0x0F, 0x40, 0x00, 0x00, 0x00, 0x26, 0x74, 0x03, \
                                                     0x01, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x46, 0xC4, \
                                                     0xCC, 0x04, 0x0F, 0x03, 0x09, 0x00, 0x00, 0x44, \
                                                     0x46, 0xC0, 0x01, 0x0F, 0x0F, 0x01, 0x00, 0x00, \
                                                     0x00, 0x46, 0xD8, 0x03, 0x02, 0x0E, 0x0E, 0x00, \
                                                     0x00, 0x00, 0x46, 0x84, 0x84, 0x02, 0x0F, 0x01, \
                                                     0x05, 0x00, 0x00, 0x46, 0x46, 0xC0, 0x01, 0x0F, \
                                                     0x0F, 0x00, 0x00, 0x00, 0x00, 0x44, 0xCC, 0x04, \
                                                     0x01, 0x0F, 0x1F, 0x00, 0x00, 0x00, 0x46, 0xC4, \
                                                     0x74, 0x03, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x26, \
                                                     0x46, 0x20, 0x30, 0x0F, 0x0F, 0x00, 0x00, 0x00, \
                                                     0x00, 0x46, 0x28, 0x30, 0x30, 0x0F, 0x00, 0x00, \
                                                     0x00, 0x00, 0x46, 0x30, 0x38, 0x30, 0x0F, 0x00, \
                                                     0x00, 0x00, 0x00, 0x46, 0x46, 0xD8, 0x03, 0x0F, \
                                                     0x0E, 0x00, 0x00, 0x00, 0x00, 0x46, 0x84, 0x02, \
                                                     0x00, 0x00, 0x04, 0x00}, 156);
    cs42l92_write_reg(driver, 0x41200, 0x4D480048);
    cs42l92_write_reg(driver, 0x0808, 0x0002);
    cs42l92_write_reg(driver, 0x0808, 0x0000);

    return CS42L92_STATUS_OK;
}
#endif

static uint32_t cs42l92_patch(cs42l92_t *driver)
{
    uint32_t ret;

    uint32_t otpid;
    ret = cs42l92_read_reg(driver, CS42L92_OTPID, &otpid);
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }
    switch (otpid)
    {
        case 0:
            break;
        case 0b1000:
            ret = cs42l92_otpid_8_patch(driver);
            if (ret == CS42L92_STATUS_FAIL)
            {
                return ret;
            }
            // Fallthrough
        default:
            ret = cs42l92_common_patch(driver);
            if (ret == CS42L92_STATUS_FAIL)
            {
                return ret;
            }
            break;
    }

    return CS42L92_STATUS_OK;
}

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/

/**
 * Initialize driver state/handle
 *
 */
uint32_t cs42l92_initialize(cs42l92_t *driver)
{
    uint32_t ret = CS42L92_STATUS_FAIL;

    if (NULL != driver)
    {
        /*
         * The memset() call sets all members to 0, including the following semantics:
         * - 'state' is set to UNCONFIGURED
         */
        memset(driver, 0, sizeof(cs42l92_t));

        ret = CS42L92_STATUS_OK;
    }

    return ret;
}

/**
 * Configures driver state/handle
 *
 */
uint32_t cs42l92_configure(cs42l92_t *driver, cs42l92_config_t *config)
{
    uint32_t ret = CS42L92_STATUS_FAIL;

    if ((NULL != driver) && \
        (NULL != config))
    {
        driver->config = *config;

        ret = bsp_driver_if_g->register_gpio_cb(driver->config.bsp_config.bsp_int_gpio_id,
                                                &cs42l92_irq_callback,
                                                driver);

        if (ret == BSP_STATUS_FAIL)
        {
            return ret;
        }

        // Configure DSP Core 1
        driver->dsp_info[0].dsp_core = 1;
        driver->dsp_info[0].base_addr = CS42L92_DSP_BASE_ADDR;
        driver->dsp_info[0].ram_banks = cs42l92_dsp1_ram_banks;
        driver->dsp_info[0].n_ram_banks = N_DSP1_RAM_BANKS;

        // Initialize FLLs
        ret = cs42l92_fll_init(driver, CS42L92_FLL1);
        if(ret == CS42L92_STATUS_FAIL)
        {
            return ret;
        }

        ret = cs42l92_fll_init(driver, CS42L92_FLL2);
        if(ret == CS42L92_STATUS_FAIL)
        {
            return ret;
        }

        // Advance driver to CONFIGURED state
        driver->state = CS42L92_STATE_CONFIGURED;
    }

    return ret;
}

/**
 * Processes driver events and notifications
 *
 */
uint32_t cs42l92_process(cs42l92_t *driver)
{
    // check for driver state
    if ((driver->state != CS42L92_STATE_UNCONFIGURED) && (driver->state != CS42L92_STATE_ERROR))
    {
        // check for driver mode
        if (driver->mode == CS42L92_MODE_HANDLING_EVENTS)
        {
            // Check for valid state to process events
            if ((driver->state == CS42L92_STATE_STANDBY))
            {
                // run through event handler
                if (CS42L92_STATUS_OK == cs42l92_event_handler(driver))
                {
                    driver->mode = CS42L92_MODE_HANDLING_CONTROLS;
                }
                else
                {
                    return CS42L92_STATUS_FAIL;
                }
            }
            // If in invalid state for handling events (i.e. BHM, Calibration), simply switch back to Handling Controls
            else
            {
                driver->mode = CS42L92_MODE_HANDLING_CONTROLS;
            }
        }

        if (driver->event_flags)
        {
            if (driver->config.bsp_config.notification_cb != NULL)
            {
                driver->config.bsp_config.notification_cb(driver->event_flags,
                                                          driver->config.bsp_config.notification_cb_arg);
            }

            driver->event_flags = 0;
        }
    }
    return CS42L92_STATUS_OK;
}

/**
 * Reset the CS42L92
 *
 */
uint32_t cs42l92_reset(cs42l92_t *driver)
{
    uint32_t temp_reg_val;
    uint32_t ret;
    uint32_t iter_timeout = 0;
    regmap_cp_config_t *cp = REGMAP_GET_CP(driver);

    // Drive RESET low
    bsp_driver_if_g->set_gpio(driver->config.bsp_config.bsp_reset_gpio_id, BSP_GPIO_LOW);
    bsp_driver_if_g->set_timer(2, NULL, NULL);

    // Enable DCVDD with RESET low
    bsp_driver_if_g->set_supply(driver->config.bsp_config.bsp_dcvdd_supply_id, BSP_SUPPLY_ENABLE);
    bsp_driver_if_g->set_timer(2, NULL, NULL);

    // Drive RESET high
    bsp_driver_if_g->set_gpio(driver->config.bsp_config.bsp_reset_gpio_id, BSP_GPIO_HIGH);

    // Wait for boot sequence to finish
    do
    {
        // Delay to allow boot before checking BOOT_DONE_EINT1
        bsp_driver_if_g->set_timer(10, NULL, NULL);

        // Read BOOT_DONE_EINT1
        ret = cs42l92_read_reg(driver, CS42L92_IRQ1_EINT_2, &temp_reg_val);
        if (ret != CS42L92_STATUS_OK)
        {
            return CS42L92_STATUS_FAIL;
        }
        iter_timeout++;
        if (iter_timeout > 20)
        {
            return CS42L92_STATUS_FAIL;
        }
    } while ((temp_reg_val & CS42L92_BOOT_DONE_EINT1_MASK) == 0);

    // Read device ID and revision ID
    ret = cs42l92_read_reg(driver, CS42L92_DEVID, &temp_reg_val);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }
    driver->devid = temp_reg_val;

    ret = cs42l92_read_reg(driver, CS42L92_REVID, &temp_reg_val);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }
    driver->revid = temp_reg_val;

    // Apply patch
    ret = cs42l92_patch(driver);
    if (ret != CS42L92_STATUS_OK)
    {
        return CS42L92_STATUS_FAIL;
    }

#ifdef CS42L92_ADC_STANDARD_MODE
    ret = cs42l92_adc_support_patch(driver);
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }
#endif

    // Write configuration data
    ret = regmap_write_array(cp, (uint32_t *) driver->config.syscfg_regs, driver->config.syscfg_regs_total);
    if (ret)
    {
        return CS42L92_STATUS_FAIL;
    }

    // Unmask interrupts
    // Omit first mask register, as BOOT_DONE_EINT1 is enabled by default
    for (uint32_t index = 1; index < N_IRQ_REGS; ++index)
    {
        ret = cs42l92_update_reg(driver,
                                 CS42L92_IRQ1_MASK_1 + cs42l92_event_data[index].irq_reg_offset,
                                 cs42l92_event_data[index].mask, 0);
        if (ret != CS42L92_STATUS_OK)
        {
            return CS42L92_STATUS_FAIL;
        }
    }

    driver->state = CS42L92_STATE_STANDBY;

    return CS42L92_STATUS_OK;
}

/**
 * Write block of data to the CS42L92 register file
 *
 */
uint32_t cs42l92_write_block(cs42l92_t *driver, uint32_t addr, uint8_t *data, uint32_t length)
{
    uint32_t ret;
    regmap_cp_config_t *cp = REGMAP_GET_CP(driver);

    if ((data == NULL) || (length == 0) || (length % 4 != 0))
    {
        return CS42L92_STATUS_FAIL;
    }

    ret = regmap_write_block(cp, addr, data, length);
    if (ret)
    {
        ret = CS42L92_STATUS_FAIL;
    }
    else
    {
        ret = CS42L92_STATUS_OK;
    }

    return ret;
}

/**
 * Read block of data from the CS42L92
 *
 */
uint32_t cs42l92_read_block(cs42l92_t *driver, uint32_t addr, uint8_t *data, uint32_t length)
{
    uint32_t ret;
    regmap_cp_config_t *cp = REGMAP_GET_CP(driver);

    if ((data == NULL) || (length == 0) || (length % 4 != 0))
    {
        return CS42L92_STATUS_FAIL;
    }

    ret = regmap_read_block(cp, addr, data, length);
    if (ret)
    {
        ret = CS42L92_STATUS_FAIL;
    }
    else
    {
        ret = CS42L92_STATUS_OK;
    }

    return ret;
}

/**
 * Wait for the provided number of miliseconds.
 *
 */
uint32_t cs42l92_wait(uint32_t time_in_ms)
{
    uint32_t bsp_status, ret = CS42L92_STATUS_OK;

    bsp_status = bsp_driver_if_g->set_timer(time_in_ms, NULL, NULL);
    if (bsp_status == BSP_STATUS_FAIL)
    {
        ret = CS42L92_STATUS_FAIL;
    }

    return ret;
}

/**
 * Finish booting the CS42L92
 *
 */
uint32_t cs42l92_boot(cs42l92_t *driver, uint32_t dsp_core, fw_img_info_t *fw_info)
{
    cs42l92_dsp_t *dsp_info;

    if (dsp_core > CS42L92_NUM_DSP || dsp_core == 0)
    {
        return CS42L92_STATUS_FAIL;
    }

    dsp_info = &driver->dsp_info[dsp_core - 1];
    dsp_info->fw_info = fw_info;

    return CS42L92_STATUS_OK;
}

/**
 * Change the power state
 *
 */
uint32_t cs42l92_power(cs42l92_t *driver, uint32_t dsp_core, uint32_t power_state)
{
    uint32_t ret = CS42L92_STATUS_OK;
    cs42l92_dsp_t *dsp_info;

    if (dsp_core > CS42L92_NUM_DSP || dsp_core == 0)
    {
        return CS42L92_STATUS_FAIL;
    }

    dsp_info = &driver->dsp_info[dsp_core - 1];

    switch (power_state)
    {
        case CS42L92_POWER_MEM_ENA:
            ret = cs42l92_power_mem_ena(driver, dsp_info);
            break;

        case CS42L92_POWER_UP:
            ret = cs42l92_power_up(driver, dsp_info);
            break;

        case CS42L92_POWER_DOWN:
            ret = cs42l92_power_down(driver, dsp_info);
            break;

        case CS42L92_POWER_MEM_DIS:
            ret = cs42l92_power_mem_dis(driver, dsp_info);
            break;

        default:
            ret = CS42L92_STATUS_FAIL;
            break;
    }

    return ret;
}

/**
 * Configure an FLL
 *
 */
uint32_t cs42l92_fll_config(cs42l92_t *driver,
                            uint32_t fll_id,
                            uint32_t src,
                            uint32_t freq_in,
                            uint32_t freq_out)
{
    uint32_t ret;
    bool osc_used;
    uint32_t current_src;


    if (fll_id >= CS42L92_NUM_FLL)
    {
        return CS42L92_STATUS_FAIL;
    }

    if (driver->fll[fll_id].ref_src == src &&
        driver->fll[fll_id].ref_freq == freq_in &&
        driver->fll[fll_id].fout == freq_out)
    {
        return CS42L92_STATUS_OK;
    }

    ret = cs42l92_fll_validate(&driver->fll[fll_id], freq_in , freq_out);
    if (ret != CS42L92_STATUS_OK)
    {
        return ret;
    }
    current_src = driver->fll[fll_id].ref_src;
    driver->fll[fll_id].ref_src = src;
    driver->fll[fll_id].ref_freq = freq_in;
    driver->fll[fll_id].fout = freq_out;

    if (driver->fll[fll_id].is_enabled && (current_src == CS42L92_FLL_SRC_INT_OSC))
    {
        driver->fll[fll_id].using_int_osc = false;
        osc_used = cs42l92_fll_int_osc_is_used(driver);
        if (!osc_used)
        {
            ret = cs42l92_write_reg(driver, CS42L92_RCO_CTRL1, 0);
            if (ret == CS42L92_STATUS_FAIL)
            {
                return ret;
            }
        }
    }

    ret = cs42l92_fll_apply_config(driver, &driver->fll[fll_id], driver->fll[fll_id].is_enabled);
    return ret;
}

/**
 * Enable an FLL
 *
 */
uint32_t cs42l92_fll_enable(cs42l92_t *driver, uint32_t fll_id)
{
    uint32_t ret;
    cs42l92_fll_t *fll;
    bool osc_used;

    if (fll_id >= CS42L92_NUM_FLL)
    {
        return CS42L92_STATUS_FAIL;
    }

    fll = &driver->fll[fll_id];

    if (fll->is_enabled)
    {
        return CS42L92_STATUS_OK;
    }

    if (fll->ref_src == CS42L92_FLL_SRC_INT_OSC)
    {
        osc_used = cs42l92_fll_int_osc_is_used(driver);
        if (!osc_used)
        {
            ret = cs42l92_write_reg(driver, CS42L92_RCO_CTRL1, CS42L92_RCO_EN);
            if (ret == CS42L92_STATUS_FAIL)
            {
                return ret;
            }
        }
        fll->using_int_osc = true;
    }

    // Set the refclk src
    ret = cs42l92_update_reg(driver,
                             fll->base + CS42L92_FLL_CONTROL2_OFFS,
                             CS42L92_FLL1_REFCLK_SRC_MASK,
                             fll->ref_src << CS42L92_FLL1_REFCLK_SRC_SHIFT);
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    // Set Enable bit
    ret = cs42l92_update_reg(driver,
                             fll->base + CS42L92_FLL_CONTROL1_OFFS,
                             CS42L92_FLL1_EN_MASK, CS42L92_FLL1_EN);
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    // set CTRL_UPD
    ret = cs42l92_update_reg(driver,
                             fll->base + CS42L92_FLL_CONTROL1_OFFS,
                             CS42L92_FLL1_CTRL_UPD_MASK, CS42L92_FLL1_CTRL_UPD);
    if (ret == CS42L92_STATUS_FAIL)
    {
        return ret;
    }

    fll->is_enabled = true;

    if (fll->is_hold == true)
    {
        // Clear FLLn_HOLD
        ret = cs42l92_update_reg(driver,
                                 fll->base + CS42L92_FLL_CONTROL1_OFFS,
                                 CS42L92_FLL1_HOLD_MASK, 0);
        if (ret == CS42L92_STATUS_FAIL)
        {
            return ret;
        }
        fll->is_hold = false;
    }

    return ret;
}

/**
 * Disable an FLL
 *
 */
uint32_t cs42l92_fll_disable(cs42l92_t *driver, uint32_t fll_id)
{
    uint32_t ret;
    bool int_osc_is_used;

    if (fll_id >= CS42L92_NUM_FLL)
    {
        return CS42L92_STATUS_FAIL;
    }

    if (driver->fll[fll_id].is_hold == false)
    {
        ret = cs42l92_update_reg(driver,
                                 driver->fll[fll_id].base + CS42L92_FLL_CONTROL1_OFFS,
                                 CS42L92_FLL1_HOLD_MASK,
                                 CS42L92_FLL1_HOLD);
        if (ret != CS42L92_STATUS_OK)
        {
            return ret;
        }
        driver->fll[fll_id].is_hold = true;
    }

    ret = cs42l92_update_reg(driver,
                             driver->fll[fll_id].base + CS42L92_FLL_CONTROL1_OFFS,
                             CS42L92_FLL1_EN_MASK, 0);
    if (ret != CS42L92_STATUS_OK)
    {
        return ret;
    }
    driver->fll[fll_id].is_enabled = false;

    driver->fll[fll_id].using_int_osc = false;
    // Disable the internal oscillator if neither FLL is using it
    int_osc_is_used = cs42l92_fll_int_osc_is_used(driver);
    if (driver->fll[fll_id].ref_src == CS42L92_FLL_SRC_INT_OSC && !int_osc_is_used)
    {
        ret = cs42l92_write_reg(driver, CS42L92_RCO_CTRL1, 0);
        if (ret == CS42L92_STATUS_FAIL)
        {
            return ret;
        }
    }
    if (ret != CS42L92_STATUS_OK)
    {
        return ret;
    }

    ret = cs42l92_update_reg(driver,
                             driver->fll[fll_id].base + CS42L92_FLL_CONTROL5_OFFS,
                             CS42L92_FLL1_FRC_INTEG_UPD_MASK,
                             CS42L92_FLL1_FRC_INTEG_UPD);
    if (ret != CS42L92_STATUS_OK)
    {
        return ret;
    }

    // Set refclk source to No Input
    ret = cs42l92_update_reg(driver,
                             driver->fll[fll_id].base + CS42L92_FLL_CONTROL2_OFFS,
                             CS42L92_FLL1_REFCLK_SRC_MASK,
                             CS42L92_FLL_SRC_NO_INPUT << CS42L92_FLL1_REFCLK_SRC_SHIFT);

    return ret;
}

/**
 * Wait a short period for FLL to achieve lock
 *
 */
uint32_t cs42l92_fll_wait_for_lock(cs42l92_t *driver, uint32_t fll_id)
{
    uint32_t ret, i, val=0;

    if (fll_id >= CS42L92_NUM_FLL)
    {
        return CS42L92_STATUS_FAIL;
    }

    for (i = 0; i < 30; i++)
    {
        ret = cs42l92_read_reg(driver, driver->fll[fll_id].sts_addr, &val);
        if (ret != CS42L92_STATUS_OK)
        {
            return ret;
        }

        if (val & driver->fll[fll_id].sts_mask)
        {
            return CS42L92_STATUS_OK;
        }
        bsp_driver_if_g->set_timer(10, NULL, NULL);
    }
    return CS42L92_STATUS_FAIL;
}

/*!
 * \mainpage Introduction
 *
 * This document outlines the driver source code included in the MCU Driver Software Package for the CS42L92 Driver.
 * This guide is primarily intended for those involved in end-system implementation, integration, and testing, who
 * will use the CS42L92 MCU Driver Software Package to integrate the CS42L92 driver source code into the end-system's
 * host MCU software.  After reviewing this guide, the reader will be able to begin software integration
 * of the CS42L92 MCU driver and then have the ability to initialize, reset, boot, configure, and service events from
 * the CS42L92.  This guide should be used along with the CS42L92 Datasheet.
 *
 *  In order to obtain any additional materials, and for any questions regarding this guide, the MCU Driver
 *  Software Package, or CS42L92 system integration, please contact your Cirrus Logic Representative.
 */
