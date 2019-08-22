/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v5.0
processor: LPC54114J256
package_id: LPC54114J256BD64
mcu_data: ksdk2_0
processor_version: 5.0.1
board: LPCXpresso54114
pin_labels:
- {pin_num: '7', pin_signal: PIO1_16/PDM0_DATA/CTIMER0_MAT0/CTIMER0_CAP0/FC7_RTS_SCL_SSEL1, label: 'J1[19]/P1_16-CT32B0_MAT0-GYRO_INT1', identifier: LM0;L_Mn}
- {pin_num: '12', pin_signal: PIO0_30/FC1_TXD_SCL_MISO/SCT0_OUT3/CTIMER0_MAT2/CTIMER0_CAP2/ADC0_1, label: 'J9[2]/P0_30-ADC1', identifier: L_Mp}
- {pin_num: '13', pin_signal: PIO0_31/PDM0_CLK/FC2_CTS_SDA_SSEL0/CTIMER2_CAP2/CTIMER0_CAP3/CTIMER0_MAT3/ADC0_2, label: 'J2[17]/J3[2]/P1[7]/U3[4]/SW2/P0_31-PDM0_CLK-ISP0_EN',
  identifier: SW2;RM1;R_Mp}
- {pin_num: '18', pin_signal: PIO1_4/PDM1_CLK/FC7_RTS_SCL_SSEL1/SCT0_OUT7/FC3_TXD_SCL_MISO/CTIMER0_MAT1/ADC0_7, label: 'J2[18]/J9[10]/P1_4-ADC7-PDM1_CLK-FC7_RTS-FC3_TXD',
  identifier: RM0;R_Mn}
- {pin_num: '41', pin_signal: PIO0_7/FC6_SCK/SCT0_OUT0/CTIMER0_MAT2/CTIMER0_CAP2, label: 'J1[16]/P0_7-FC6_SCK', identifier: LM1;L_Mp}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm4, enableClock: 'true'}
- pin_list:
  - {pin_num: '31', peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI, pin_signal: PIO0_0/FC0_RXD_SDA_MOSI/FC3_CTS_SDA_SSEL0/CTIMER0_CAP0/SCT0_OUT3, mode: inactive, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: '32', peripheral: FLEXCOMM0, signal: TXD_SCL_MISO, pin_signal: PIO0_1/FC0_TXD_SCL_MISO/FC3_RTS_SCL_SSEL1/CTIMER0_CAP1/SCT0_OUT1, mode: inactive, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: '7', peripheral: CTIMER0, signal: 'MATCH, 0', pin_signal: PIO1_16/PDM0_DATA/CTIMER0_MAT0/CTIMER0_CAP0/FC7_RTS_SCL_SSEL1, identifier: L_Mn}
  - {pin_num: '18', peripheral: CTIMER0, signal: 'MATCH, 1', pin_signal: PIO1_4/PDM1_CLK/FC7_RTS_SCL_SSEL1/SCT0_OUT7/FC3_TXD_SCL_MISO/CTIMER0_MAT1/ADC0_7, identifier: R_Mn}
  - {pin_num: '13', peripheral: CTIMER0, signal: 'MATCH, 3', pin_signal: PIO0_31/PDM0_CLK/FC2_CTS_SDA_SSEL0/CTIMER2_CAP2/CTIMER0_CAP3/CTIMER0_MAT3/ADC0_2, identifier: R_Mp}
  - {pin_num: '42', peripheral: USB0, signal: USB_VBUS, pin_signal: PIO1_11/FC6_RTS_SCL_SSEL1/CTIMER1_CAP0/FC4_SCK/USB0_VBUS}
  - {pin_num: '41', peripheral: CTIMER0, signal: 'MATCH, 2', pin_signal: PIO0_7/FC6_SCK/SCT0_OUT0/CTIMER0_MAT2/CTIMER0_CAP2, identifier: L_Mp}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M4F */
void BOARD_InitPins(void)
{
    /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin0_config = (/* Pin is configured as FC0_RXD_SDA_MOSI */
                                        IOCON_PIO_FUNC1 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Input filter disabled */
                                        IOCON_PIO_INPFILT_OFF |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN0 (coords: 31) is configured as FC0_RXD_SDA_MOSI */
    IOCON_PinMuxSet(IOCON, 0U, 0U, port0_pin0_config);

    const uint32_t port0_pin1_config = (/* Pin is configured as FC0_TXD_SCL_MISO */
                                        IOCON_PIO_FUNC1 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Input filter disabled */
                                        IOCON_PIO_INPFILT_OFF |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN1 (coords: 32) is configured as FC0_TXD_SCL_MISO */
    IOCON_PinMuxSet(IOCON, 0U, 1U, port0_pin1_config);

    IOCON->PIO[0][31] = ((IOCON->PIO[0][31] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT031 (pin 13) is configured as CTIMER0_MAT3. */
                         | IOCON_PIO_FUNC(PIO031_FUNC_ALT6)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO031_DIGIMODE_DIGITAL));

    IOCON->PIO[0][7] = ((IOCON->PIO[0][7] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT07 (pin 41) is configured as CTIMER0_MAT2. */
                        | IOCON_PIO_FUNC(PIO07_FUNC_ALT3)

                        /* Select Analog/Digital mode.
                         * : Digital mode. */
                        | IOCON_PIO_DIGIMODE(PIO07_DIGIMODE_DIGITAL));

    IOCON->PIO[1][11] = ((IOCON->PIO[1][11] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT111 (pin 42) is configured as USB0_VBUS. */
                         | IOCON_PIO_FUNC(PIO111_FUNC_ALT7)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO111_DIGIMODE_DIGITAL));

    IOCON->PIO[1][16] = ((IOCON->PIO[1][16] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT116 (pin 7) is configured as CTIMER0_MAT0. */
                         | IOCON_PIO_FUNC(PIO116_FUNC_ALT2)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO116_DIGIMODE_DIGITAL));

    IOCON->PIO[1][4] = ((IOCON->PIO[1][4] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT14 (pin 18) is configured as CTIMER0_MAT1. */
                        | IOCON_PIO_FUNC(PIO14_FUNC_ALT6)

                        /* Select Analog/Digital mode.
                         * : Digital mode. */
                        | IOCON_PIO_DIGIMODE(PIO14_DIGIMODE_DIGITAL));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/