/**
 * @file cs42l92.h
 *
 * @brief Functions and prototypes exported by the CS42L92 Driver module
 *
 * @copyright
 * Copyright (c) Cirrus Logic 2021, 2023 All Rights Reserved, http://www.cirrus.com/
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

#ifndef CS42L92_H
#define CS42L92_H

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
 * INCLUDES
 **********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "fw_img.h"
#include "cs42l92_sym.h"
#include "cs42l92_spec.h"
#include "cs42l92_syscfg_regs.h"
#include "sdk_version.h"
#include "regmap.h"

/***********************************************************************************************************************
 * LITERALS & CONSTANTS
 **********************************************************************************************************************/

/**
 * @defgroup CS42L92_STATUS_
 * @brief Return values for all public and most private API calls
 *
 * @{
 */
#define CS42L92_STATUS_OK                               (0)
#define CS42L92_STATUS_FAIL                             (1)
/** @} */

/**
 * @defgroup CS42L92_BUS_TYPE_
 * @brief Types of serial bus to control the CS42L92
 *
 * @see cs42l92_bsp_config_t member bus_type
 *
 * @{
 */
#define CS42L92_BUS_TYPE_I2C                            (0)
#define CS42L92_BUS_TYPE_SPI                            (1)
/** @} */

/**
 * @defgroup CS42L92_STATE_
 * @brief State of the driver
 *
 * @see cs42l92_t member state
 *
 * @{
 */
#define CS42L92_STATE_UNCONFIGURED                      (0)
#define CS42L92_STATE_CONFIGURED                        (1)
#define CS42L92_STATE_STANDBY                           (2)
#define CS42L92_STATE_ERROR                             (4)
 /** @} */

/**
 * @defgroup CS42L92_MODE_
 * @brief Mode of the driver
 *
 * @see cs42l92_t member mode
 *
 * @{
 */
#define CS42L92_MODE_HANDLING_CONTROLS                  (0)
#define CS42L92_MODE_HANDLING_EVENTS                    (1)
/** @} */

/**
 * @defgroup CS42L92_POWER_
 * @brief Power states passed on to power() API argument power_state
 *
 * @see cs42l92_power
 *
 * @{
 */
#define CS42L92_POWER_UP                                (0)
#define CS42L92_POWER_DOWN                              (1)
#define CS42L92_POWER_MEM_ENA                           (2)
#define CS42L92_POWER_MEM_DIS                           (3)
/** @} */

/**
 * @defgroup CS42L92_EVENT_FLAG_
 * @brief Flags passed to Notification Callback to notify BSP of specific driver events
 *
 * @see CS42L92_notification_callback_t argument event_flags
 *
 * @{
 */
#define CS42L92_EVENT_FLAG_BOOT_DONE                    (1 << 0)
#define CS42L92_EVENT_FLAG_SYSCLK_FAIL                  (1 << 1)
#define CS42L92_EVENT_FLAG_SYSCLK_ERR                   (1 << 2)
#define CS42L92_EVENT_FLAG_CTRLIF_ERR                   (1 << 3)
#define CS42L92_EVENT_FLAG_MPU_ERR                      (1 << 4)
#define CS42L92_EVENT_FLAG_DSP1_IRQ0                    (1 << 5)
#define CS42L92_EVENT_FLAG_WDT_EXPIRE                   (1 << 6)
#define CS42L92_EVENT_FLAG_AHB_SYS_ERR                  (1 << 7)
#define CS42L92_EVENT_FLAG_AHB_PACK_ERR                 (1 << 8)
/** @} */

#define CS42L92_NUM_DSP                                 (1)

/**
 * @brief FLL Ids. Identifies the two FLLs.
 *
 * @{
 */
#define CS42L92_NUM_FLL                 (2)
#define CS42L92_FLL1                    (0)
#define CS42L92_FLL2                    (1)
/** @} */

/**
 * @brief The source clock identifiers for the FLLs
 *
 * @{
 */
#define CS42L92_FLL_SRC_MCLK1           (0x0)
#define CS42L92_FLL_SRC_MCLK2           (0x1)
#define CS42L92_FLL_SRC_INT_OSC         (0x2)
#define CS42L92_FLL_SRC_NO_INPUT        (0x3)
#define CS42L92_FLL_SRC_ASP1_BCLK       (0x8)
#define CS42L92_FLL_SRC_ASP2_BCLK       (0x9)
/** @} */

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * ENUMS, STRUCTS, UNIONS, TYPEDEFS
 **********************************************************************************************************************/

/**
 * Function pointer to Notification Callback
 *
 * @details
 * This callback will be registered at driver configuration.  This callback is called whenever the driver has detected
 * a significant event has occurred, such as an over-temperature condition.
 *
 * @see cs42l92_configure
 *
 * @param [in] event_flags      Flags to indicate which events have occurred
 * @param [in] arg              Callback arg registered by upper layer
 *
 * @return none
 */
typedef void (*cs42l92_notification_callback_t)(uint32_t event_flags, void *arg);

/**
 * Data structure to describe a Control Request
 *
 * @see cs42l92_control
 */
typedef struct
{
    uint32_t id;    ///< Control ID
    void *arg;      ///< Argument for Control Request (nature depends on type of request)
} cs42l92_control_request_t;

/**
 * Data structure for ADSP2 Core DSP Firmware Revision
 *
 */
typedef struct
{
    union
    {
        uint32_t word;
        struct
        {
            uint32_t patch                      : 8;
            uint32_t minor                      : 8;
            uint32_t major                      : 8;
            uint32_t reserved                   : 8;
        };
    };
} cs42l92_fw_revision_t;

/**
 * Configuration parameters required for calls to BSP-Driver Interface
 */
typedef struct
{
    uint32_t bsp_reset_gpio_id;                         ///< Used to ID CS42L92 Reset pin in bsp_driver_if calls
    uint32_t bsp_dcvdd_supply_id;                       ///< Used to ID CS42L92 DCVDD Supply in bsp_driver_if calls
    uint32_t bsp_int_gpio_id;                           ///< Used to ID CS42L92 INT pin in bsp_driver_if calls
    cs42l92_notification_callback_t notification_cb;    ///< Notification callback registered for detected events
    void *notification_cb_arg;                          ///< Notification callback argument
    regmap_cp_config_t cp_config;
} cs42l92_bsp_config_t;

/**
 * Driver configuration data structure
 *
 * @see cs42l92_configure
 */
typedef struct
{
    cs42l92_bsp_config_t bsp_config;                ///< BSP Configuration
    const uint32_t *syscfg_regs;                ///< Pointer to system configuration table
    uint32_t syscfg_regs_total;                     ///< Total entries in system configuration table
} cs42l92_config_t;

/**
 * @defgroup CS42L92_DSP_RAM_BANK_
 * @brief A structure for describing a DSP ram bank's register offsets
 *
 * @{
 */
typedef struct
{
    uint32_t reg_start;
    uint32_t reg_end;
} cs42l92_dsp_ram_bank_t;
/** @} */

/**
 * DSP data structure
 */
typedef struct
{
    uint32_t dsp_core;                          ///< The DSP core number.  1-based
    uint32_t base_addr;                         ///< The base memory address for the DSP's config registers
    fw_img_info_t *fw_info;                     ///< Current ADSP2 FW/Coefficient boot configuration
    const cs42l92_dsp_ram_bank_t *ram_banks;    ///< Pointer to an array of ram_banks
    uint32_t n_ram_banks;                       ///< The number of ram_bank array entries
} cs42l92_dsp_t;

/**
 * Data structure for FLL
 */

typedef struct
{
    int32_t id;
    uint32_t base;

    uint32_t sts_addr;
    uint32_t sts_mask;

    uint32_t fout;

    uint32_t ref_src;
    uint32_t ref_freq;

    bool using_int_osc;

    bool is_enabled;
    bool is_hold;
} cs42l92_fll_t;

/**
 * Driver state data structure
 *
 * This is the type used for the handle to the driver for all driver public API calls.  This structure must be
 * instantiated outside the scope of the driver source and initialized by the cs42l92_initialize public API.
 */
typedef struct
{
    uint32_t state;                                      ///< General driver state - @see CS42L92_STATE_
    uint32_t mode;                                       ///< General driver mode - @see CS42L92_MODE_
    uint32_t devid;                                      ///< CS42L92 DEVID of current device
    uint32_t revid;                                      ///< CS42L92 REVID of current device
    /*
     * List of register address/value pairs to write on wake up from hibernate
     */
    cs42l92_config_t config;                             ///< Driver configuration fields - see cs42l92_config_t
    uint32_t event_flags;                                ///< Most recent event_flags reported to BSP Notification callback

    cs42l92_dsp_t dsp_info[CS42L92_NUM_DSP];             ///< Current ADSP2 FW/Coefficient boot configuration

    cs42l92_fll_t fll[CS42L92_NUM_FLL];                  ///< FLL configurations
} cs42l92_t;

/**
 * Data structure for interrupt information
 *
 * @see cs42l92_event_handler
 */

typedef struct
{
    uint32_t irq_reg_offset;
    uint32_t mask;
    uint32_t event_flag;
} irq_reg_t;

/***********************************************************************************************************************
 * GLOBAL VARIABLES
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/
/**
 * Initialize driver state/handle
 *
 * Sets all driver state members to 0
 *
 * @param [in] driver           Pointer to the driver state
 *
 * @return
 * - CS42L92_STATUS_FAIL        if pointer to driver is NULL
 * - CS42L92_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_initialize(cs42l92_t *driver);

/**
 * Configures driver state/handle
 *
 * Including the following:
 * - Applies all one-time configurations to the driver state
 * - Registers the IRQ Callback for INTb GPIO with the BSP
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] config           Pointer to driver configuration data structure
 *
 * @return
 * - CS42L92_STATUS_FAIL        if any pointers are NULL
 * - CS42L92_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_configure(cs42l92_t *driver, cs42l92_config_t *config);

/**
 * Processes driver events and notifications
 *
 * This implements Event Handling and BSP Notification
 *
 * @param [in] driver           Pointer to the driver state
 *
 * @return
 * - if in UNCONFIGURED or ERROR state, returns CS42L92_STATUS_OK
 * - else if in HANDLING_CONTROLS mode, returns CS42L92_STATUS_OK
 * - otherwise, returns status Event Handler
 *
 * @warning This MUST be placed either in baremetal or RTOS task while (1)
 *
 */
uint32_t cs42l92_process(cs42l92_t *driver);

/**
 * Reset the CS42L92
 *
 * This call performs all necessary reset of the CS42L92 from power-on-reset to being ready to be put into
 * its first use-case.
 * - toggling RESET line
 * - verifying boot_sequence completes
 * - read device id and revision
 * - apply any patches
 * - write out any initial configuration
 * - unmask all relevant interrupts
 *
 * @param [in] driver           Pointer to the driver state
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - any control port activity fails
 *      - any status bit polling times out
 *      - the part is not supported
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_reset(cs42l92_t *driver);

/*
 * Find if a symbol is in the symbol table and return its address if it is.
 *
 * This will search through the symbol table pointed to in the 'fw_info' member of the driver state and return
 * the control port register address to use for access.  The 'symbol_id' parameter must be from the group CS42L92_SYM_.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_core         Number of the DSP core to be searched.  1-based.  0 searches all cores.
 * @param [in] symbol_id        id of symbol to search for
 *
 * @return
 * - non-0 - symbol register address
 * - 0 - symbol not found.
 *
 */
uint32_t cs42l92_find_symbol(cs42l92_t *driver, uint32_t dsp_core, uint32_t symbol_id);

/*
 * Reads the contents of a single register/memory address
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             Address of the register to be read
 * @param [out] val             Pointer to where the read register value will be stored
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Control port activity fails
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_read_reg(cs42l92_t *driver, uint32_t addr, uint32_t *val);

/*
 * Writes the contents of a single register/memory address
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             Address of the register to be written
 * @param [in] val              Value to be written to the register
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Control port activity fails
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_write_reg(cs42l92_t *driver, uint32_t addr, uint32_t val);

/*
 * Reads, updates and writes (if there's a change) the contents of a single register/memory address
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             Address of the register to be written
 * @param [in] mask             Mask of the bits within the register to update
 * @param [in] val              Value to be written to the register (only bits matching the mask will be written)
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Control port activity fails
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_update_reg(cs42l92_t *driver, uint32_t addr, uint32_t mask, uint32_t val);

/*
 * Writes out val to a single register/memory address and then waits for the register to be returned to 0
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             Address of the register to be written
 * @param [in] val              Value to be written to the register
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Control port activity fails
 *      - The function times out waiting for the register to return to 0
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_write_acked_reg(cs42l92_t *driver, uint32_t addr, uint32_t val);

/*
 * Write block of data to the CS42L92 register file
 *
 * This call is used to load the HALO FW/COEFF files to HALO RAM.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             Starting address of loading destination
 * @param [in] data             Pointer to array of bytes to be written
 * @param [in] length           Length of array of bytes to be written
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - data pointer is NULL
 *      - length is not multiple of 4
 *      - Control port activity fails
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_write_block(cs42l92_t *driver, uint32_t addr, uint8_t *data, uint32_t length);

/*
 * Read block of data from the CS42L92 register file
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             Starting address of loading destination
 * @param [in] data             Pointer to array of bytes to store read data
 * @param [in] length           Length of array of bytes to be read
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - data pointer is NULL
 *      - length is not multiple of 4
 *      - Control port activity fails
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_read_block(cs42l92_t *driver, uint32_t addr, uint8_t *data, uint32_t length);


/*
 * Wait a set number of ms
 *
 * @param [in] time_in_ms       Time in ms before the function returns
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - the BSP set_timer function returns an error
 * - otherwise, returns CS42L92_STATUS_OK
 *
 */
uint32_t cs42l92_wait(uint32_t time_in_ms);

/**
 * Finish booting the CS42L92
 *
 * While cs42l92_write_block loads the actual FW/COEFF data into ADSP2 RAM, cs42l92_boot will finish the boot process
 * by:
 * - loading the fw_img_info_t fw_info member of the driver handle
 * - Performing any post-boot configuration writes
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_core         Number of the DSP core being booted.  1-based.
 * @param [in] fw_info          Pointer to FW information and FW Control Symbol Table
 *
 * @return
 * - CS42L92_STATUS_FAIL if:
 *      - Any pointers are null
 *      - Control port activity fails
 *      - Required FW Control symbols are not found in the symbol table
 * - CS42L92_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_boot(cs42l92_t *driver, uint32_t dsp_core, fw_img_info_t *fw_info);

/**
 * Change the power state
 *
 * Enable or disable a DSP's memory, or start or stop a DSP running.  The DSP's memory must be enabled before writes
 * can be made to it.
 *
 * @see CS42L92_POWER_
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_core         Number of the DSP core to be powered up or down.  1-based.
 * @param [in] power_state      New power state
 *
 * @return
 * - CS42L92_STATUS_FAIL        if requested power_state is invalid, or if the call to change power state fails
 * - CS42L92_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_power(cs42l92_t *driver, uint32_t dsp_core, uint32_t power_state);

/**
 * Configure one of the FLLs.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] fll_id           Id for the FLL clock subsystem to be configured
 * @param [in] src              The source clock identifier
 * @param [in] freq_in          The frequency of the source clock
 * @param [in] freq_out         The required FLL output frequency
 *
 * @return
 * - CS47L15_STATUS_FAIL        if FLL subsystem Id is invalid, or if the call to configure the FLL fails
 * - CS47L15_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_fll_config(cs42l92_t *driver,
                            uint32_t fll_id,
                            uint32_t src,
                            uint32_t freq_in,
                            uint32_t freq_out);
/**
 * Enable an FLL
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] fll_id           FLL Id to be enabled.
 *
 * @return
 * - CS47L15_STATUS_FAIL        if requested FLL is invalid, or if the call to enable the FLL fails
 * - CS47L15_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_fll_enable(cs42l92_t *driver, uint32_t fll_id);

/**
 * Disable an FLL
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] fll_id           FLL Id to be disabled.
 *
 * @return
 * - CS47L15_STATUS_FAIL        if requested FLL is invalid
 * - CS47L15_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_fll_disable(cs42l92_t *driver, uint32_t fll_id);

/**
 * Wait a short time for the FLL to reach a locked state
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] fll_id           FLL Id to achieve lock
 *
 * @return
 * - CS47L15_STATUS_FAIL        if requested FLL is invalid or a locked state is not reached
 * - CS47L15_STATUS_OK          otherwise
 *
 */
uint32_t cs42l92_fll_wait_for_lock(cs42l92_t *driver, uint32_t fll_id);

/**********************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif // CS42L92_H
