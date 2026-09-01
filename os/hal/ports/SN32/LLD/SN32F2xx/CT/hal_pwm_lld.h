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
 * @file    CT/hal_pwm_lld.h
 * @brief   SN32 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef HAL_PWM_LLD_H
#define HAL_PWM_LLD_H

#if HAL_USE_PWM || defined(__DOXYGEN__)

#include "sn32_ct.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#define PWM_CHANNELS                 (SN32_CT16B1_CHANNELS - 1)
#define MCTRL_INDEX                  (PWM_CHANNELS / 10)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   PWMD1 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SN32_PWM_USE_CT16B1) || defined(__DOXYGEN__)
#define SN32_PWM_USE_CT16B1                  FALSE
#endif

/**
 * @brief   PWMD1 interrupt priority level setting.
 */
#if !defined(SN32_PWM_CT16B1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_PWM_CT16B1_IRQ_PRIORITY         2
#endif

/**
 * @brief   PWMD0 (CT16B0) / PWMD2 (CT16B2) enable switches + IRQ priorities.
 * @note    Small SN32 timers used to widen the PWM channel count beyond CT16B1.
 */
#if !defined(SN32_PWM_USE_CT16B0) || defined(__DOXYGEN__)
#define SN32_PWM_USE_CT16B0                  FALSE
#endif
#if !defined(SN32_PWM_CT16B0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_PWM_CT16B0_IRQ_PRIORITY         2
#endif
#if !defined(SN32_PWM_USE_CT16B2) || defined(__DOXYGEN__)
#define SN32_PWM_USE_CT16B2                  FALSE
#endif
#if !defined(SN32_PWM_CT16B2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_PWM_CT16B2_IRQ_PRIORITY         2
#endif
/** @} */

/*===========================================================================*/
/* Configuration checks.                                                     */
/*===========================================================================*/

#if SN32_PWM_USE_CT16B1 && !SN32_HAS_CT16B1
#error "CT16B1 not present in the selected device"
#endif

#if !SN32_PWM_USE_CT16B0 && !SN32_PWM_USE_CT16B1 && !SN32_PWM_USE_CT16B2
#error "PWM driver activated but no CT peripheral assigned"
#endif

/* Checks on allocation of CT units.*/
#if SN32_PWM_USE_CT16B1
#if defined(SN32_CT16B1_IS_USED)
#error "PWMD1 requires CT16B1 but the timer is already used"
#else
#define SN32_CT16B1_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if SN32_PWM_USE_CT16B1 && !defined(SN32_CT16B1_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(SN32_PWM_CT16B1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to CT16B1"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a PWM mode.
 */
typedef uint32_t pwmmode_t;

/**
 * @brief   Type of a PWM channel.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   Type of a channels mask.
 */
typedef uint32_t pwmchnmsk_t;

/**
 * @brief   Type of a PWM counter.
 */
typedef uint32_t pwmcnt_t;

/**
 * @brief   Type of a PWM driver channel configuration structure.
 */
typedef struct {
  /**
   * @brief Channel active logic level.
   */
  pwmmode_t                 mode;
  /**
   * @brief Channel callback pointer.
   * @note  This callback is invoked on the channel compare event. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /* End of the mandatory fields.*/
  /**
   * @brief CT16 PFPA register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */
  uint8_t                  pfpamsk;
} PWMChannelConfig;

/**
 * @brief   Type of a PWM driver configuration structure.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  uint32_t                  frequency;
  /**
   * @brief   PWM period in ticks.
   * @note    The low level can use assertions in order to catch invalid
   *          period specifications.
   */
  pwmcnt_t                  period;
  /**
   * @brief Periodic callback pointer.
   * @note  This callback is invoked on PWM counter reset. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /**
   * @brief Channels configurations.
   */
  PWMChannelConfig          channels[PWM_CHANNELS];
  /* End of the mandatory fields.*/
  /**
   * @brief CT16 CNTCTRL register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */
  uint32_t                  cntctrl;
} PWMConfig;

/**
 * @brief   Structure representing a PWM driver.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t                state;
  /**
   * @brief Current driver configuration data.
   */
  const PWMConfig           *config;
  /**
   * @brief   Current PWM period in ticks.
   */
  pwmcnt_t                  period;
  /**
   * @brief   Mask of the enabled channels.
   */
  pwmchnmsk_t               enabled;
  /**
   * @brief   Number of channels in this instance.
   */
  pwmchannel_t              channels;
#if defined(PWM_DRIVER_EXT_FIELDS)
  PWM_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Timer base clock.
   */
  uint32_t                  clock;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/
/* Per-timer accessor variants. The dispatchers below fan out to every enabled
 * timer; only the one matching `timer` acts (the C compiler still type-checks all
 * branches, which is fine because every SN32 CT16 struct shares the same
 * config/match/pwm/irq sub-structs). Multi-timer support (CT16B0/B1/B2) is required
 * for boards whose PWM axis exceeds one timer's channel count (e.g. SN32F290). */
#if SN32_PWM_USE_CT16B0
#define SN32_CT_PWM_SET_CT16B0(timer, field, value)       \
  do { if ((timer) == &PWMD0) (SN32_CT16B0)->field = (value); } while (0)
#else
#define SN32_CT_PWM_SET_CT16B0(timer, field, value)  do { } while (0)
#endif
#if SN32_PWM_USE_CT16B1
#define SN32_CT_PWM_SET_CT16B1(timer, field, value)       \
  do { if ((timer) == &PWMD1) (SN32_CT16B1)->field = (value); } while (0)
#else
#define SN32_CT_PWM_SET_CT16B1(timer, field, value)  do { } while (0)
#endif
#if SN32_PWM_USE_CT16B2
#define SN32_CT_PWM_SET_CT16B2(timer, field, value)       \
  do { if ((timer) == &PWMD2) (SN32_CT16B2)->field = (value); } while (0)
#else
#define SN32_CT_PWM_SET_CT16B2(timer, field, value)  do { } while (0)
#endif

#define SN32_CT_PWM_SET(timer, field, value)        \
  do {                                               \
    SN32_CT_PWM_SET_CT16B0(timer, field, value);     \
    SN32_CT_PWM_SET_CT16B1(timer, field, value);     \
    SN32_CT_PWM_SET_CT16B2(timer, field, value);     \
  } while (0)

#if SN32_PWM_USE_CT16B0
#define SN32_CT_PWM_OR_CT16B0(timer, field, value)        \
  do { if ((timer) == &PWMD0) (SN32_CT16B0)->field |= (value); } while (0)
#else
#define SN32_CT_PWM_OR_CT16B0(timer, field, value)  do { } while (0)
#endif
#if SN32_PWM_USE_CT16B1
#define SN32_CT_PWM_OR_CT16B1(timer, field, value)        \
  do { if ((timer) == &PWMD1) (SN32_CT16B1)->field |= (value); } while (0)
#else
#define SN32_CT_PWM_OR_CT16B1(timer, field, value)  do { } while (0)
#endif
#if SN32_PWM_USE_CT16B2
#define SN32_CT_PWM_OR_CT16B2(timer, field, value)        \
  do { if ((timer) == &PWMD2) (SN32_CT16B2)->field |= (value); } while (0)
#else
#define SN32_CT_PWM_OR_CT16B2(timer, field, value)  do { } while (0)
#endif

#define SN32_CT_PWM_OR(timer, field, value)         \
  do {                                              \
    SN32_CT_PWM_OR_CT16B0(timer, field, value);     \
    SN32_CT_PWM_OR_CT16B1(timer, field, value);     \
    SN32_CT_PWM_OR_CT16B2(timer, field, value);     \
  } while (0)

#if SN32_PWM_USE_CT16B0
#define SN32_CT_PWM_AND_CT16B0(timer, field, value)        \
  do { if ((timer) == &PWMD0) (SN32_CT16B0)->field &= (value); } while (0)
#else
#define SN32_CT_PWM_AND_CT16B0(timer, field, value)  do { } while (0)
#endif
#if SN32_PWM_USE_CT16B1
#define SN32_CT_PWM_AND_CT16B1(timer, field, value)        \
  do { if ((timer) == &PWMD1) (SN32_CT16B1)->field &= (value); } while (0)
#else
#define SN32_CT_PWM_AND_CT16B1(timer, field, value)  do { } while (0)
#endif
#if SN32_PWM_USE_CT16B2
#define SN32_CT_PWM_AND_CT16B2(timer, field, value)        \
  do { if ((timer) == &PWMD2) (SN32_CT16B2)->field &= (value); } while (0)
#else
#define SN32_CT_PWM_AND_CT16B2(timer, field, value)  do { } while (0)
#endif

#define SN32_CT_PWM_AND(timer, field, value)         \
  do {                                              \
    SN32_CT_PWM_AND_CT16B0(timer, field, value);     \
    SN32_CT_PWM_AND_CT16B1(timer, field, value);     \
    SN32_CT_PWM_AND_CT16B2(timer, field, value);     \
  } while (0)

#if SN32_PWM_USE_CT16B0
#define SN32_CT_PWM_GET_CT16B0(timer, cmd) ((timer) == &PWMD0 ? (SN32_CT16B0)->cmd : 0)
#else
#define SN32_CT_PWM_GET_CT16B0(timer, cmd) (0)
#endif
#if SN32_PWM_USE_CT16B1
#define SN32_CT_PWM_GET_CT16B1(timer, cmd) ((timer) == &PWMD1 ? (SN32_CT16B1)->cmd : 0)
#else
#define SN32_CT_PWM_GET_CT16B1(timer, cmd) (0)
#endif
#if SN32_PWM_USE_CT16B2
#define SN32_CT_PWM_GET_CT16B2(timer, cmd) ((timer) == &PWMD2 ? (SN32_CT16B2)->cmd : 0)
#else
#define SN32_CT_PWM_GET_CT16B2(timer, cmd) (0)
#endif

/* Only the matching timer contributes; the others yield 0. */
#define SN32_CT_PWM_GET(timer, cmd) \
  (SN32_CT_PWM_GET_CT16B0(timer, cmd) + SN32_CT_PWM_GET_CT16B1(timer, cmd) + SN32_CT_PWM_GET_CT16B2(timer, cmd))

#if SN32_PWM_USE_CT16B0
#define SN32_CT_PWM_GET_ADDR_CT16B0(timer, cmd) ((timer) == &PWMD0 ? &(SN32_CT16B0)->cmd : NULL)
#else
#define SN32_CT_PWM_GET_ADDR_CT16B0(timer, cmd) (NULL)
#endif
#if SN32_PWM_USE_CT16B1
#define SN32_CT_PWM_GET_ADDR_CT16B1(timer, cmd) ((timer) == &PWMD1 ? &(SN32_CT16B1)->cmd : NULL)
#else
#define SN32_CT_PWM_GET_ADDR_CT16B1(timer, cmd) (NULL)
#endif
#if SN32_PWM_USE_CT16B2
#define SN32_CT_PWM_GET_ADDR_CT16B2(timer, cmd) ((timer) == &PWMD2 ? &(SN32_CT16B2)->cmd : NULL)
#else
#define SN32_CT_PWM_GET_ADDR_CT16B2(timer, cmd) (NULL)
#endif

/* First non-NULL (i.e. the matching timer) wins. */
#define SN32_CT_PWM_GET_ADDR(timer, cmd)                                          \
  (SN32_CT_PWM_GET_ADDR_CT16B0(timer, cmd) != NULL ? SN32_CT_PWM_GET_ADDR_CT16B0(timer, cmd) : \
   SN32_CT_PWM_GET_ADDR_CT16B1(timer, cmd) != NULL ? SN32_CT_PWM_GET_ADDR_CT16B1(timer, cmd) : \
   SN32_CT_PWM_GET_ADDR_CT16B2(timer, cmd))

/**
 * @brief   Changes the period of the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
#define pwm_lld_change_period(pwmp, period)                                 \
  SN32_CT_PWM_SET((pwmp), MR[(pwmp)->channels], pwm_lld_mr_value((pwmp), ((period) - 1)))

/**
 * @brief   Changes the timer counter of the PWM peripheral.
 * @details This function changes the timer counter of a PWM unit that has
 *          already been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit timer counter is changed to the new value.
 * @note    The function overrides the TC value of the PWM peripheral
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] counter   new timer counter value in bits
 *
 * @notapi
 */
#define pwm_lld_change_counter(pwmp, counter)                                 \
  SN32_CT_PWM_SET((pwmp), config.TC, (counter))
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SN32_PWM_USE_CT16B0 && !defined(__DOXYGEN__)
extern PWMDriver PWMD0;
#endif
#if SN32_PWM_USE_CT16B1 && !defined(__DOXYGEN__)
extern PWMDriver PWMD1;
#endif
#if SN32_PWM_USE_CT16B2 && !defined(__DOXYGEN__)
extern PWMDriver PWMD2;
#endif

/**
 * @brief   Returns the value to write into a match (MR) register.
 * @details On SN32F280/F290 the match registers of every CT16 timer *except*
 *          CT16B1 are write-protected and require the PWM key (0x5A in the top
 *          byte) to accept a new value; CT16B1's MR registers are not protected.
 *          On other devices no key is needed. Writing the key on the small
 *          timers is what makes their period/duty updates actually take effect.
 */
static inline uint32_t pwm_lld_mr_value(const PWMDriver *pwmp, uint32_t value) {
#if (defined(SN32F280) || defined(SN32F290))
#if SN32_PWM_USE_CT16B1
  if (pwmp == &PWMD1) {
    return value;
  }
#endif
  return CT16_PWM_UNLOCK(value);
#else
  (void)pwmp;
  return value;
#endif
}

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmp);
  void pwm_lld_stop(PWMDriver *pwmp);
  void pwm_lld_enable_channel(PWMDriver *pwmp,
                              pwmchannel_t channel,
                              pwmcnt_t width);
  void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel);
  void pwm_lld_enable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_disable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                           pwmchannel_t channel);
  void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                            pwmchannel_t channel);
  void pwm_lld_serve_interrupt(PWMDriver *pwmp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM */

#endif /* HAL_PWM_LLD_H */

/** @} */
