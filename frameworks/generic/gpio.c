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

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <mqueue.h>
#include <errno.h>
#include <message.h>
#include <gpio.h>

#include <nuttx/ioexpander/gpio.h>

#define GPIO_POLL       20

typedef struct gpio_client
{
  void                      *task;
#if (CONFIG_IOEXPANDER_NPINS > 32)
  uint64_t                  in;
  uint64_t                  out;
  struct gpio_changed_s     *changed[64];
#else
  uint32_t                  in;
  uint32_t                  out;
  struct gpio_changed_s     *changed[32];
#endif
} gpio_client_t;

#ifdef CONFIG_CAN_PASS_STRUCTS
static void gpio_notify(union sigval value)
#else
static void gpio_notify(FAR void *sival_ptr)
#endif
{
#ifdef CONFIG_CAN_PASS_STRUCTS
  struct gpio_changed_s *changed = (struct gpio_changed_s *)value.sival_ptr;
#else
  struct gpio_changed_s *changed = (struct gpio_changed_s *)sival_ptr;
#endif
  msg_gpio_t msg;

  msg.id = MSG_GPIO_CHANGED;
  msg.changed.pin = changed->gp_pin;
  msg.changed.state = changed->gp_state;
  msg.changed.time = changed->gp_time;

  /* notify client the change of the gpio */
  message_send(changed->gp_ptr, (message_t *)&msg,
          sizeof(msg_gpio_t));
}

#if (CONFIG_IOEXPANDER_NPINS > 32)
static int gpio_update(gpio_client_t *clt, uint64_t in, uint64_t out)
#else
static int gpio_update(gpio_client_t *clt, uint32_t in, uint32_t out)
#endif
{
#if (CONFIG_IOEXPANDER_NPINS > 32)
  uint64_t in_add = 0;
  uint64_t out_add = 0;
  uint64_t io_del = 0;
#else
  uint32_t in_add = 0;
  uint32_t out_add = 0;
  uint32_t io_del = 0;
#endif

  /* get updated ones */
  in_add = (in ^ clt->in) & in;
  out_add = (out ^ clt->out) & out;
  io_del = ((in | out) ^ (clt->in | clt->out))
      & (clt->in | clt->out);
  if (in_add || out_add || io_del)
    {
      int ret;
      uint32_t i;
      int fd;
      char devpath[12];
      struct sigevent notify;

      notify.sigev_notify            = SIGEV_THREAD;
      notify.sigev_signo             = GPIO_POLL;
      notify.sigev_notify_function   = gpio_notify;
      notify.sigev_notify_attributes = NULL;

      for (i = 0; i < CONFIG_IOEXPANDER_NPINS; i++)
        {
          /* update new input config */
          if (in_add & (1 << i))
            {
              sprintf(devpath, "/dev/gpin%d", i);

              fd = open(devpath, O_RDWR);
              if (fd < 0)
                {
                  fprintf(stderr, "ERROR: Failed to open\
                          %s: %d\n", devpath, errno);
                  continue;
                }
              ret = ioctl(fd, GPIOC_SETPINTYPE,
                      GPIO_INTERRUPT_PIN);
              if (ret < 0)
                {
                  fprintf(stderr, "ERROR: Failed to set pintype\
                          from %s: %d\n", devpath, errno);
                  close(fd);
                  continue;
                }

              if (clt->changed[i] == NULL)
                {
                  clt->changed[i] = malloc(sizeof(struct gpio_changed_s));
                  if (clt->changed[i] == NULL)
                    {
                      close(fd);
                      return -ENOMEM;
                    }
                }
              clt->changed[i]->gp_ptr = clt->task;
              notify.sigev_value.sival_ptr = clt->changed[i];

              ret = ioctl(fd, GPIOC_REGISTER, (unsigned long)&notify);
              if (ret < 0)
                {
                  fprintf(stderr, "ERROR: Failed to setup for\
                          signal from %s: %d\n",
                          devpath, errno);
                }

              close(fd);
            }

          /* update new output config */
          if (out_add & (1 << i))
            {
              sprintf(devpath, "/dev/gpin%d", i);

              fd = open(devpath, O_RDWR);
              if (fd < 0)
                {
                  fprintf(stderr, "ERROR: Failed to open\
                          %s: %d\n", devpath, errno);
                  continue;
                }

              if (clt->out & (1 << i))
                {
                  ret = ioctl(fd, GPIOC_UNREGISTER, (unsigned long)&notify);
                  if (ret < 0)
                    {
                      fprintf(stderr, "ERROR: Failed to setup for\
                              signal from %s: %d\n",
                              devpath, errno);
                      close(fd);
                      continue;
                    }
                }

              if (clt->changed[i] != NULL)
                {
                  free(clt->changed[i]);
                  clt->changed[i] = NULL;
                }

              ret = ioctl(fd, GPIOC_SETPINTYPE,
                      GPIO_OUTPUT_PIN);
              if (ret < 0)
                {
                  fprintf(stderr, "ERROR: Failed to set\
                          pintype from %s: %d\n",
                          devpath, errno);
                }
              close(fd);
            }

          /* remove old config */
          if (io_del & (1 << i))
            {
              sprintf(devpath, "/dev/gpin%d", i);

              fd = open(devpath, O_RDWR);
              if (fd < 0)
                {
                  fprintf(stderr, "ERROR: Failed to open\
                          %s: %d\n", devpath, errno);
                  continue;
                }

              if (clt->in & (1 << i))
                {
                  ret = ioctl(fd, GPIOC_UNREGISTER, (unsigned long)&notify);
                  if (ret < 0)
                    {
                      fprintf(stderr, "ERROR: Failed to setup for\
                              signal from %s: %d\n",
                              devpath, errno);
                      close(fd);
                      continue;
                    }
                  if (clt->changed[i] != NULL)
                    {
                      free(clt->changed[i]);
                      clt->changed[i] = NULL;
                    }
                }

              ret = ioctl(fd, GPIOC_SETPINTYPE, GPIO_INPUT_PIN);
              if (ret < 0)
                {
                  fprintf(stderr, "ERROR: Failed to set\
                          pintype from %s: %d\n",
                          devpath, errno);
                }

              close(fd);
            }
        }
    }
  clt->in = in;
  clt->out = out;

  return 0;
}

int gpio_get(gpio_handle_t handle, uint8_t pin)
{
  int ret;
  int fd;
  char devpath[12];
  uint8_t val = STATE_GPIO_INVAL;
  gpio_client_t *clt = (gpio_client_t *)handle;

  if (clt == NULL)
    return -EINVAL;

  /* check if pin configered to output or input */
  if (((clt->out & (1 << pin)) == 0) && ((clt->in & (1 << pin)) == 0))
    return -EINVAL;

  sprintf(devpath, "/dev/gpin%d", pin);
  fd = open(devpath, O_RDWR);
  if (fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n",
              devpath, errcode);
      return -errcode;
    }

  ret = ioctl(fd, GPIOC_READ, (unsigned long)(uintptr_t)&val);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to read from %s: %d\n",
              devpath, errcode);
      close(fd);
      return -errcode;
    }

  close(fd);

  return val;
}

int gpio_set(gpio_handle_t handle, uint8_t pin, uint8_t state)
{
  int ret;
  int fd;
  char devpath[12];
  gpio_client_t *clt = (gpio_client_t *)handle;

  if (clt == NULL)
    return -EINVAL;

  /* check if pin configered to output */
  if ((clt->out & (1 << pin)) == 0)
    return -EINVAL;

  sprintf(devpath, "/dev/gpin%d", pin);
  fd = open(devpath, O_RDWR);
  if (fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n",
              devpath, errcode);
      return -errcode;
    }

  ret = ioctl(fd, GPIOC_WRITE, state ? 1 : 0);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to write from %s: %d\n",
              devpath, errcode);
      close(fd);
      return -errcode;
    }

  close(fd);

  return 0;
}

int gpio_set_dir(gpio_handle_t handle, uint8_t pin, uint8_t dir)
{
#if (CONFIG_IOEXPANDER_NPINS > 32)
  uint64_t in;
  uint64_t out;
#else
  uint32_t in;
  uint32_t out;
#endif
  gpio_client_t *clt = (gpio_client_t *)handle;

  if (clt == NULL)
    return -EINVAL;

  in = clt->in;
  out = clt->out;

  /* return OK if no update */
  if ((dir == DIR_GPIO_IN) && (in & (1 << pin)))
    return 0;
  if ((dir == DIR_GPIO_OUT) && (out & (1 << pin)))
    return 0;

  /* update client config */
  if (dir == DIR_GPIO_IN)
    {
      in |= (1 << pin);
      out &= ~(1 << pin);
    }
  else if (dir == DIR_GPIO_OUT)
    {
      out |= (1 << pin);
      in &= ~(1 << pin);
    }

  /* update of client */
  gpio_update(clt, in, out);

  return 0;
}

#if (CONFIG_IOEXPANDER_NPINS > 32)
gpio_handle_t gpio_register(void *task, uint64_t in, uint64_t out)
#else
gpio_handle_t gpio_register(void *task, uint32_t in, uint32_t out)
#endif
{
  gpio_client_t *clt;

  if ((in & out) != 0)
    return NULL;

  /* create client instance */
  clt = malloc(sizeof(gpio_client_t));
  if (!clt)
    return NULL;
  memset(clt, 0, sizeof(gpio_client_t));

  clt->task = task;

  /* update of client */
  gpio_update(clt, in, out);

  return (gpio_handle_t)clt;
}

int gpio_deregister(gpio_handle_t handle)
{
  gpio_client_t *clt = (gpio_client_t *)handle;

  if (clt == NULL)
    return -EINVAL;

   /* update of client */
  gpio_update(clt, 0, 0);

  /* destroy client instance */
  free(clt);

  return 0;
}
