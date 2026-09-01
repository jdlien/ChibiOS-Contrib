/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    CT/hal_pwm_lld.c
 * @brief   SN32 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#    define PWM_CLK SN32_HCLK
/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD1 driver identifier.
 * @note    The driver PWMD1 allocates the complex timer CT16B1 when enabled.
 */
#    if SN32_PWM_USE_CT16B0 || defined(__DOXYGEN__)
PWMDriver PWMD0;
#    endif
#    if SN32_PWM_USE_CT16B1 || defined(__DOXYGEN__)
PWMDriver PWMD1;
#    endif
#    if SN32_PWM_USE_CT16B2 || defined(__DOXYGEN__)
PWMDriver PWMD2;
#    endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#    if SN32_PWM_USE_CT16B0 || defined(__DOXYGEN__)
#        if !defined(SN32_CT16B0_SUPPRESS_ISR)
#            if !defined(SN32_CT16B0_HANDLER)
#                error "SN32_CT16B0_HANDLER not defined"
#            endif
OSAL_IRQ_HANDLER(SN32_CT16B0_HANDLER) {
    OSAL_IRQ_PROLOGUE();
    pwm_lld_serve_interrupt(&PWMD0);
    OSAL_IRQ_EPILOGUE();
}
#        endif
#    endif     /* SN32_PWM_USE_CT16B0 */

#    if SN32_PWM_USE_CT16B1 || defined(__DOXYGEN__)
#        if !defined(SN32_CT16B1_SUPPRESS_ISR)
#            if !defined(SN32_CT16B1_HANDLER)
#                error "SN32_CT16B1_HANDLER not defined"
#            endif
/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    pwm_lld_serve_interrupt(&PWMD1);

    OSAL_IRQ_EPILOGUE();
}
#        endif /* !defined(SN32_CT16B1_SUPPRESS_ISR) */
#    endif     /* SN32_PWM_USE_CT16B1 */

#    if SN32_PWM_USE_CT16B2 || defined(__DOXYGEN__)
#        if !defined(SN32_CT16B2_SUPPRESS_ISR)
#            if !defined(SN32_CT16B2_HANDLER)
#                error "SN32_CT16B2_HANDLER not defined"
#            endif
OSAL_IRQ_HANDLER(SN32_CT16B2_HANDLER) {
    OSAL_IRQ_PROLOGUE();
    pwm_lld_serve_interrupt(&PWMD2);
    OSAL_IRQ_EPILOGUE();
}
#        endif
#    endif     /* SN32_PWM_USE_CT16B2 */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Whether a timer packs enable / IO-enable / mode into a single
 *          PWMCTRL register (protected by the 0x5A PWM key), rather than using
 *          the separate PWMENB / PWMIOENB registers.
 * @details On SN32F280/F290 only CT16B1 has PWMENB/PWMIOENB; CT16B0/B2/B5 do
 *          not, and configure everything through PWMCTRL. On other devices the
 *          classic PWMENB/PWMIOENB model applies to all timers.
 */
static inline bool pwm_lld_pwmctrl_only(const PWMDriver *pwmp) {
#    if (defined(SN32F280) || defined(SN32F290))
#        if SN32_PWM_USE_CT16B1
    return pwmp != &PWMD1;
#        else
    (void)pwmp;
    return true;
#        endif
#    else
    (void)pwmp;
    return false;
#    endif
}

/* PWMCTRL (n=0,2,5) bit layout: PWMnEN at bit n, PWMnMODE at bit (4 + 2n),
   PWMnIOEN at bit (20 + n), and the 0x5A write key in bits [31:24]. */
#    define CT16_PWMCTRLONLY_EN(n)    (1u << (n))
#    define CT16_PWMCTRLONLY_MODE(n, mode) ((uint32_t)(mode) << (4 + 2 * (n)))
#    define CT16_PWMCTRLONLY_IOEN(n)  (1u << (20 + (n)))
/* In the n=0,2,5 MCTRL register the PWM cycle-length register is MR9, whose
   reset-enable (MR9RST) is bit 22 -- not at the regular (n%10)*3 position. */
#    define CT16_PWMCTRLONLY_MR9RST   (1u << 22)

/**
 * @brief   Index of the match register that sets the PWM cycle length (period).
 * @details CT16B1 uses MR[channels] (MR12). CT16B0/B2/B5 have only MR0-3 for the
 *          duty outputs plus a dedicated MR9 for the cycle length.
 */
static inline uint8_t pwm_lld_period_mr(const PWMDriver *pwmp) {
    return pwm_lld_pwmctrl_only(pwmp) ? 9 : pwmp->channels;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {
    /* Driver initialization. Each timer records its own channel count so the rest
     * of the driver can use pwmp->channels instead of a single global. */
#    if SN32_PWM_USE_CT16B0
    pwmObjectInit(&PWMD0);
    PWMD0.channels = SN32_CT16B0_CHANNELS - 1;
#    endif
#    if SN32_PWM_USE_CT16B1
    pwmObjectInit(&PWMD1);
    PWMD1.channels = SN32_CT16B1_CHANNELS - 1;
#    endif
#    if SN32_PWM_USE_CT16B2
    pwmObjectInit(&PWMD2);
    PWMD2.channels = SN32_CT16B2_CHANNELS - 1;
#    endif
}

/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp) {
    uint32_t psc;
    uint32_t pwmctrl;
    uint32_t pwmctrl2;
    uint32_t pwmen;
    uint32_t pwmioen;

    if (pwmp->state == PWM_STOP) {
        /* Clock activation and timer reset.*/
#    if SN32_PWM_USE_CT16B0
        if (&PWMD0 == pwmp) {
            sys1EnableCT16B0();
            sys1ResetCT16B0();
            CT16B0_ResetTimer();
#        if !defined(SN32_CT16B0_SUPPRESS_ISR)
            nvicEnableVector(SN32_CT16B0_NUMBER, SN32_PWM_CT16B0_IRQ_PRIORITY);
#        endif
            pwmp->clock = PWM_CLK;
        }
#    endif
#    if SN32_PWM_USE_CT16B1
        if (&PWMD1 == pwmp) {
            sys1EnableCT16B1();
            sys1ResetCT16B1();
            CT16B1_ResetTimer();
#        if !defined(SN32_CT16B1_SUPPRESS_ISR)
            nvicEnableVector(SN32_CT16B1_NUMBER, SN32_PWM_CT16B1_IRQ_PRIORITY);
#        endif
            pwmp->clock = PWM_CLK;
        }
#    endif
#    if SN32_PWM_USE_CT16B2
        if (&PWMD2 == pwmp) {
            sys1EnableCT16B2();
            sys1ResetCT16B2();
            CT16B2_ResetTimer();
#        if !defined(SN32_CT16B2_SUPPRESS_ISR)
            nvicEnableVector(SN32_CT16B2_NUMBER, SN32_PWM_CT16B2_IRQ_PRIORITY);
#        endif
            pwmp->clock = PWM_CLK;
        }
#    endif

#    if (defined(SN32F240B) || defined(SN32F240C))
        /* PFPA - Map all PWM outputs to their PWM A pins */
        SN_PFPA->CT16B1 = 0x00000000;
        /* PFPA assignment for PWM B-pin mapping.*/
        for (uint8_t i = 0; i < pwmp->channels; i++) {
            if (pwmp->config->channels[i].pfpamsk != 0) {
                SN_PFPA->CT16B1 |= (1 << i);
            }
        }
#    endif

        /* Channel PWM mode selection and polarities setup.*/
        if (pwm_lld_pwmctrl_only(pwmp)) {
            /* CT16B0/B2/B5 (F280/F290): enable, IO-enable and mode all live in the
               single PWMCTRL register, and the write needs the 0x5A PWM key. */
            uint32_t pwmctrl_only = CT16_PWM_KEY;
            for (uint8_t i = 0; i < pwmp->channels; i++) {
                switch (pwmp->config->channels[i].mode & PWM_OUTPUT_MASK) {
                    case PWM_OUTPUT_ACTIVE_LOW:
                        pwmctrl_only |= CT16_PWMCTRLONLY_EN(i) | CT16_PWMCTRLONLY_IOEN(i) | CT16_PWMCTRLONLY_MODE(i, CT16_PWMnMODE_1);
                        break;
                    case PWM_OUTPUT_ACTIVE_HIGH:
                        pwmctrl_only |= CT16_PWMCTRLONLY_EN(i) | CT16_PWMCTRLONLY_IOEN(i) | CT16_PWMCTRLONLY_MODE(i, CT16_PWMnMODE_2);
                        break;
                }
            }
            SN32_CT_PWM_SET(pwmp, pwm.PWMCTRL, pwmctrl_only);
        } else {
            pwmctrl                                = 0;
            pwmctrl2                               = 0;
            pwmen                                  = 0;
            pwmioen                                = 0;
            volatile uint32_t *pwmctrl_registers[] = {&pwmctrl, &pwmctrl2};
            for (uint8_t i = 0; i < pwmp->channels; i++) {
                switch (pwmp->config->channels[i].mode & PWM_OUTPUT_MASK) {
                    case PWM_OUTPUT_ACTIVE_LOW:
                        *pwmctrl_registers[(i > 15) ? 1 : 0] |= mskCT16_PWMnMODE_1(i);
                        pwmen |= mskCT16_PWMnEN_EN(i);
                        pwmioen |= mskCT16_PWMnIOEN_EN(i);
                        break;
                    case PWM_OUTPUT_ACTIVE_HIGH:
                        *pwmctrl_registers[(i > 15) ? 1 : 0] |= mskCT16_PWMnMODE_2(i);
                        pwmen |= mskCT16_PWMnEN_EN(i);
                        pwmioen |= mskCT16_PWMnIOEN_EN(i);
                        break;
                }
            }
            SN32_CT_PWM_SET(pwmp, pwm.PWMCTRL, pwmctrl);
            SN32_CT_PWM_SET(pwmp, pwm.PWMCTRL2, pwmctrl2);
            SN32_CT_PWM_SET(pwmp, pwm.PWMENB, pwmen);
            SN32_CT_PWM_SET(pwmp, pwm.PWMIOENB, pwmioen);
        }
    } else {
        /* Driver re-configuration scenario, it must be stopped first.*/
        SN32_CT_PWM_SET(pwmp, config.TMRCTRL, CT16_CEN_DIS); /* Timer disabled.*/
        /* Counter reset to zero.*/
        SN32_CT_PWM_SET(pwmp, config.TMRCTRL, mskCT16_CRST); // Set CT16B1 as the up-counting mode.
        while (SN32_CT_PWM_GET(pwmp, config.TMRCTRL) & mskCT16_CRST)
            ; // Wait until timer reset done.
    }

    /* Timer configuration.*/
    psc = (pwmp->clock / pwmp->config->frequency) - 1;
    osalDbgAssert((psc <= SN32_CT16_PRE_LIMIT) && /* Prescaler calculation.*/
                      ((psc + 1) * pwmp->config->frequency) == pwmp->clock,
                  "invalid frequency");
    SN32_CT_PWM_SET(pwmp, config.PRE, psc);
    uint8_t period_mr = pwm_lld_period_mr(pwmp);
    SN32_CT_PWM_SET(pwmp, MR[period_mr], pwm_lld_mr_value(pwmp, pwmp->period - 1));

#    if SN32_PWM_USE_ONESHOT || defined(__DOXYGEN__)
    volatile uint32_t *mctrl_registers[] = {SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL2), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL3)};
    // Determine which MCTRL register to use
    volatile uint32_t *reg = mctrl_registers[pwmp->channels / 10];
    if (reg != NULL) {
        *reg |= mskCT16_MRnSTOP_EN(pwmp->channels);
    }
#    elif !defined(SN32_PWM_NO_RESET)
    if (pwm_lld_pwmctrl_only(pwmp)) {
        /* CT16B0/B2/B5: enable MR9 reset in MCTRL; the write needs the 0x5A key. */
        uint32_t mctrl = (SN32_CT_PWM_GET(pwmp, match.MCTRL) & 0x00FFFFFF) | CT16_PWM_KEY | CT16_PWMCTRLONLY_MR9RST;
        SN32_CT_PWM_SET(pwmp, match.MCTRL, mctrl);
    } else {
        volatile uint32_t *mctrl_registers[] = {SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL2), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL3)};
        // Determine which MCTRL register to use
        volatile uint32_t *reg = mctrl_registers[pwmp->channels / 10];
        if (reg != NULL) {
            *reg |= mskCT16_MRnRST_EN(pwmp->channels);
        }
    }
#    endif
    SN32_CT_PWM_AND(pwmp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/

    /* Timer configured and started.*/
    SN32_CT_PWM_OR(pwmp, config.TMRCTRL, mskCT16_CEN_EN);
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp) {
    /* If in ready state then disables the PWM clock.*/
    if (pwmp->state == PWM_READY) {
        SN32_CT_PWM_SET(pwmp, config.TMRCTRL, CT16_CEN_DIS);                     /* Timer disabled.    */
        SN32_CT_PWM_AND(pwmp, irq.IC, mskCT_IC_Clear(SN32_CT16B1_MAX_CHANNELS)); /* Clear pending IRQs.*/

#    if SN32_PWM_USE_CT16B0
        if (&PWMD0 == pwmp) {
#        if !defined(SN32_CT16B0_SUPPRESS_ISR)
            nvicDisableVector(SN32_CT16B0_NUMBER);
#        endif
            sys1DisableCT16B0();
        }
#    endif
#    if SN32_PWM_USE_CT16B1
        if (&PWMD1 == pwmp) {
#        if !defined(SN32_CT16B1_SUPPRESS_ISR)
            nvicDisableVector(SN32_CT16B1_NUMBER);
#        endif
            sys1DisableCT16B1();
        }
#    endif
#    if SN32_PWM_USE_CT16B2
        if (&PWMD2 == pwmp) {
#        if !defined(SN32_CT16B2_SUPPRESS_ISR)
            nvicDisableVector(SN32_CT16B2_NUMBER);
#        endif
            sys1DisableCT16B2();
        }
#    endif
    }
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    The function has effect at the next cycle start.
 * @note    Channel notification is not enabled.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp, pwmchannel_t channel, pwmcnt_t width) {
    if (channel < pwmp->channels) {
        /* Changing channel duty cycle on the fly.*/
        SN32_CT_PWM_SET(pwmp, MR[channel], pwm_lld_mr_value(pwmp, width));
        if (pwm_lld_pwmctrl_only(pwmp)) {
            /* IO-enable lives in PWMCTRL; re-supply the key so the write sticks. */
            SN32_CT_PWM_OR(pwmp, pwm.PWMCTRL, CT16_PWM_KEY | CT16_PWMCTRLONLY_IOEN(channel));
        } else {
            SN32_CT_PWM_OR(pwmp, pwm.PWMIOENB, mskCT16_PWMnIOEN_EN(channel));
        }
    }
}

/**
 * @brief   Disables a PWM channel and its notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {
    if (channel < pwmp->channels) {
        SN32_CT_PWM_OR(pwmp, irq.IC, mskCT16_MRnIC(channel));
        if (pwm_lld_pwmctrl_only(pwmp)) {
            /* Clear the IOEN bit in PWMCTRL while re-supplying the key (a plain
               AND would drop the key and be ignored). */
            uint32_t v = (SN32_CT_PWM_GET(pwmp, pwm.PWMCTRL) & ~CT16_PWMCTRLONLY_IOEN(channel)) | CT16_PWM_KEY;
            SN32_CT_PWM_SET(pwmp, pwm.PWMCTRL, v);
        } else {
            SN32_CT_PWM_AND(pwmp, pwm.PWMIOENB, ~mskCT16_PWMnIOEN_EN(channel));
        }
    }
}

/**
 * @brief   Enables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_enable_periodic_notification(PWMDriver *pwmp) {
    volatile uint32_t *mctrl_registers[] = {SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL2), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL3)};
    // Determine which MCTRL register to use
    volatile uint32_t *reg = mctrl_registers[pwmp->channels / 10];
    if (reg != NULL) {
        *reg |= mskCT16_MRnIE_EN(pwmp->channels);
    }
}

/**
 * @brief   Disables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_disable_periodic_notification(PWMDriver *pwmp) {
    volatile uint32_t *mctrl_registers[] = {SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL2), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL3)};
    // Determine which MCTRL register to use
    volatile uint32_t *reg = mctrl_registers[pwmp->channels / 10];
    if (reg != NULL) {
        *reg &= ~mskCT16_MRnIE_EN(pwmp->channels);
    }
}

/**
 * @brief   Enables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_enable_channel_notification(PWMDriver *pwmp, pwmchannel_t channel) {
    volatile uint32_t *mctrl_registers[] = {SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL2), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL3)};
    // Determine which MCTRL register to use
    volatile uint32_t *reg = mctrl_registers[pwmp->channels / 10];
    if (reg != NULL) {
        *reg |= mskCT16_MRnIE_EN(channel);
    }
}

/**
 * @brief   Disables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel_notification(PWMDriver *pwmp, pwmchannel_t channel) {
    volatile uint32_t *mctrl_registers[] = {SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL2), SN32_CT_PWM_GET_ADDR(pwmp, match.MCTRL3)};
    // Determine which MCTRL register to use
    volatile uint32_t *reg = mctrl_registers[pwmp->channels / 10];
    if (reg != NULL) {
        *reg &= ~mskCT16_MRnIE_EN(channel);
    }
}

/**
 * @brief   Common CT IRQ handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_serve_interrupt(PWMDriver *pwmp) {
    uint32_t ris;

    ris = SN32_CT_PWM_GET(pwmp, irq.RIS);
    SN32_CT_PWM_SET(pwmp, irq.IC, ris);
    for (int i = 0; i < pwmp->channels; i++) {
        if (((ris & mskCT16_MRnIF(i)) != 0) && (pwmp->config->channels[i].callback != NULL)) pwmp->config->channels[i].callback(pwmp);
    }
    if (((ris & mskCT16_MRnIF(pwmp->channels)) != 0) && (pwmp->config->callback != NULL)) pwmp->config->callback(pwmp);
}

#endif /* HAL_USE_PWM */

/** @} */
