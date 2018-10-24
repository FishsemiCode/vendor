/* -----------------------------------------------------------------------------
 * Copyright (c) 2018 Pinecone Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * -------------------------------------------------------------------------- */

#ifndef GPIO_H_
#define GPIO_H_

/****************************************************************************
 * Public MACRO definition
 ***************************************************************************/
#define DIR_GPIO_IN         0
#define DIR_GPIO_OUT        1

#define STATE_GPIO_LOW      0
#define STATE_GPIO_HIGH     1
#define STATE_GPIO_INVAL    -1

/****************************************************************************
 * Public MESSAGE definition
 ***************************************************************************/

#define MSG_GPIO_BASE       0x1000
#define MSG_GPIO_CHANGED    MSG_GPIO_BASE
#define MSG_GPIO_MAX        MSG_GPIO_CHANGED

typedef struct msg_gpio_changed {
  uint8_t                   pin;
  uint8_t                   state;
  uint32_t                  time;
} msg_gpio_changed_t;

typedef struct msg_gpio {
    uint16_t                id;
    union {
        msg_gpio_changed_t  changed;
    };
} msg_gpio_t;

typedef void *gpio_handle_t;

/****************************************************************************
 * Public API definition
 ***************************************************************************/

int gpio_get(gpio_handle_t handle, uint8_t pin);
int gpio_set(gpio_handle_t handle, uint8_t pin, uint8_t state);
int gpio_set_dir(gpio_handle_t handle, uint8_t pin, uint8_t dir);
#if (CONFIG_IOEXPANDER_NPINS > 32)
gpio_handle_t gpio_register(void *task, uint64_t in, uint64_t out);
#else
gpio_handle_t gpio_register(void *task, uint32_t in, uint32_t out);
#endif
int gpio_deregister(gpio_handle_t handle);

#endif /* GPIO_H_ */
