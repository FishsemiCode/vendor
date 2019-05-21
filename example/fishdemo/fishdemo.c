/****************************************************************************
 * apps/external/example/fishdemo/fishdemo.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/mount.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <sched.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <semaphore.h>
#include <mqueue.h>
#include <pthread.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>

#include "mount.h"

#include <nuttx/drivers/ramdisk.h>
#include <nuttx/timers/rtc.h>

#include "fsutils/mkfatfs.h"
#include "mount.h"


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * fishdemo_main
 ****************************************************************************/

#define DEMO_PRIO  100
#define STACK_SIZE 2048

static sem_t g_sem;

static int demo_task(int argc, FAR char *argv[])
{
  printf("This is demo task entry and pid is: %d!\n", getpid());
  usleep(1000 * 1000L);
  printf("This is demo task exit!\n");

  sem_post(&g_sem);

  return EXIT_SUCCESS;
}

static void task_test(void)
{
  int ret;

  sem_init(&g_sem, 0, 0);

  printf("Welcome to Fish Demo application!!\n");
  printf("Create a task !\n");
  ret = task_create("demo", DEMO_PRIO, STACK_SIZE, demo_task, NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("Failed to start demo task %d\n", errcode);
      return;
    }

  printf("Parent: Waiting for child pid = %d\n", ret);
  sem_wait(&g_sem);
  sem_destroy(&g_sem);
}

static mqd_t g_send_mqfd;
static mqd_t g_recv_mqfd;

static void *sender_thread(void *arg)
{
  char msg_buffer[24];
  struct mq_attr attr;
  int status = 0;
  int nerrors = 0;
  int i;

  printf("sender thread: Starting!\n");

  attr.mq_maxmsg = 20;
  attr.mq_msgsize = 24;
  attr.mq_flags = 0;

  g_send_mqfd = mq_open("mqueue", O_WRONLY|O_CREAT, 0666, &attr);
  if (g_send_mqfd == (mqd_t)-1)
    {
      printf("sender mq open failed!\n");
      pthread_exit((pthread_addr_t)1);
    }

  memcpy(msg_buffer, "Hi,Welcome to FishSemi!", 24);
  for (i = 0; i < 10; i++)
    {
      status = mq_send(g_send_mqfd, msg_buffer, 24, 42);
      if (status < 0)
        nerrors++;
      printf("send on msg: %d, status = %d\n", i, status);
    }

  mq_close(g_send_mqfd);
  g_send_mqfd = NULL;

  return (pthread_addr_t)((uintptr_t)nerrors);
}

static void *receiver_thread(void *arg)
{
  char msg_buffer[24];
  struct mq_attr attr;
  int nbytes;
  int nerrors = 0;
  int i;

  printf("receiver thread: Starting!\n");

  attr.mq_maxmsg = 20;
  attr.mq_msgsize = 24;
  attr.mq_flags = 0;

  g_recv_mqfd = mq_open("mqueue", O_RDONLY|O_CREAT, 0666, &attr);
  if (g_recv_mqfd == (mqd_t)-1)
    {
      printf("receiv mq open failed!\n");
      pthread_exit((pthread_addr_t)1);
    }

  for (i = 0; i < 10; i++)
    {
      memset(msg_buffer, 0xaa, 24);
      nbytes = mq_receive(g_recv_mqfd, msg_buffer, 24, 0);
      if (nbytes < 0)
        {
          if (errno != EINTR)
            {
              nerrors++;
            }
        }
      else if (nbytes != 24)
        {
          nerrors++;
        }
      else if (memcmp("Hi,Welcome to FishSemi!", msg_buffer, nbytes) != 0)
        {
          printf("Not equal!\n");
        }
      else
        {
          printf("receive successfully on %d!\n", i);
        }
    }

  mq_close(g_recv_mqfd);
  g_recv_mqfd = NULL;

  pthread_exit((pthread_addr_t)((uintptr_t)nerrors));
  return (pthread_addr_t)((uintptr_t)nerrors);

}

static void thread_mq_test(void)
{
  pthread_t sender;
  pthread_t receiver;
  struct sched_param sparam;
  pthread_attr_t attr;
  void *result;
  int status;

  printf("Starting receiver!\n");
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("pthread_attr_init failed, status = %d\n", status);
      return;
    }
  status = pthread_attr_setstacksize(&attr, 1024);
  if (status != 0)
    {
      printf("pthread_attr_setstacksize failed, status = %d\n", status);
      return;
    }

  sparam.sched_priority = 100;
  status = pthread_attr_setschedparam(&attr, &sparam);
  if (status != OK)
    {
      printf("pthread_attr_setschedparam failed, status = %d\n", status);
      return;
    }
  status = pthread_create(&receiver, &attr, receiver_thread, NULL);
  if (status != 0)
    {
      printf("pthread_create receiver failed, status = %d\n", status);
    }

  printf("Starting sender!\n");
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("pthread_attr_init failed, status = %d\n", status);
      return;
    }
  status = pthread_attr_setstacksize(&attr, 1024);
  if (status != 0)
    {
      printf("pthread_attr_setstacksize failed, status = %d\n", status);
      return;
    }

  sparam.sched_priority = 100;
  status = pthread_attr_setschedparam(&attr, &sparam);
  if (status != OK)
    {
      printf("pthread_attr_setschedparam failed, status = %d\n", status);
      return;
    }
  status = pthread_create(&sender, &attr, sender_thread, NULL);
  if (status != 0)
    {
      printf("pthread_create sender failed, status = %d\n", status);
    }

  printf("Waiting for sender to complete!\n");
  pthread_join(sender, &result);
  pthread_join(receiver, &result);

  mq_close(g_recv_mqfd);
  mq_close(g_send_mqfd);

  pthread_attr_destroy(&attr);
}

#define MY_TIMER_SIGNAL 17
#define WAKEUP_SIGNAL 17
#define SIGVALUE_INT 42

static sem_t g_thread_sem;
#ifdef CONFIG_CAN_PASS_STRUCTS
static void thread_callback(union sigval value)
#else
static void thread_callback(FAR void *ptr)
#endif
{
  printf("timer callback!\n");
  sem_post(&g_thread_sem);
}

static void clocks_test(void)
{
  struct timeval tv;
  struct timespec ts0, ts1;
  struct sigevent notify;
  struct itimerspec timer;
  timer_t timerid;
  struct tm tm;
  int ret;

  memset(&tm, 0, sizeof(tm));
  memset(&ts0, 0, sizeof(ts0));
  memset(&ts0, 0, sizeof(ts1));

  tm.tm_year = 2019 - 1900;
  tm.tm_mon = 4;
  tm.tm_mday = 15;
  tm.tm_hour = 12;
  tm.tm_min = 0;
  tm.tm_sec = 0;
  ts0.tv_sec = mktime(&tm);
  ts0.tv_nsec = 1000;

  ret = clock_settime(CLOCK_REALTIME, &ts0);
  if (ret < 0)
    {
      printf("Failed to call clock_settime!\n");
    }

  ret = clock_gettime(CLOCK_REALTIME, &ts1);
  if (ret < 0)
    {
      printf("Failed to call clock_gettime!\n");
    }

  printf("get time sec: %lu, nsec: %04lu\n", ts1.tv_sec, ts1.tv_nsec);

  ret = gettimeofday(&tv, NULL);
  if (ret != 0)
    {
      printf("Failed to call gettimeofday!\n");
    }

  printf("gettimeofday sec: %lu, usec: %04lu\n", tv.tv_sec, tv.tv_usec);

  printf("Create timer!\n");

  sem_init(&g_thread_sem, 0, 0);

  notify.sigev_notify            = SIGEV_THREAD;
  notify.sigev_signo             = MY_TIMER_SIGNAL;
  notify.sigev_value.sival_int   = SIGVALUE_INT;
  notify.sigev_notify_function   = thread_callback;
  notify.sigev_notify_attributes = NULL;

  ret = timer_create(CLOCK_REALTIME, &notify, &timerid);
  if (ret != 0)
    {
      printf("Failed to create timer!\n");
    }

  printf("Start timer!\n");
  timer.it_value.tv_sec     = 2;
  timer.it_value.tv_nsec    = 0;
  timer.it_interval.tv_sec  = 2;
  timer.it_interval.tv_nsec = 0;
  ret = timer_settime(timerid, 0, &timer, NULL);
  if (ret != 0)
    {
      printf("Failed to start timer!\n");
    }

  sem_wait(&g_thread_sem);
  sem_destroy(&g_thread_sem);

  ret = timer_delete(timerid);
  if (ret != 0)
    {
      printf("Failed to delete timer!\n");
    }
}

static sem_t sem;
static bool sigreceived = false;
static bool threadexited = false;

static void death_of_child(int signo, siginfo_t *info, void *ucontext)
{
  if (info)
    {
      printf("death_of_child: PID %d received signal=%d code=%d "
          "errno=%d pid=%d status=%d\n",
          getpid(), signo, info->si_code, info->si_errno,
          info->si_pid, info->si_status);
    }
  else
    {
      printf("death_of_child: PID %d received signal=%d (no info?)\n",
          getpid(), signo);
    }
}

static void wakeup_action(int signo, siginfo_t *info, void *ucontext)
{
  sigset_t oldset;
  sigset_t allsigs;
  int status;

  printf("Wake up Received signal %d\n", signo);
  sigreceived = true;

  /* Check signo */

  if (signo != WAKEUP_SIGNAL)
    {
      printf("Wake up: ERROR expected signo=%d\n" , WAKEUP_SIGNAL);
    }

  /* Check siginfo */

  if (info->si_value.sival_int != SIGVALUE_INT)
    {
      printf("Wake up: ERROR sival_int=%d expected %d\n",
          info->si_value.sival_int, SIGVALUE_INT);
    }
  else
    {
      printf("Wake up: sival_int=%d\n" , info->si_value.sival_int);
    }

  if (info->si_signo != WAKEUP_SIGNAL)
    {
      printf("Wake up: ERROR expected si_signo=%d, got=%d\n",
          WAKEUP_SIGNAL, info->si_signo);
    }

  printf("Wake up: si_code=%d\n" , info->si_code);

  /* Check ucontext_t */

  printf("Wake up: ucontext=%p\n" , ucontext);

  /* Check sigprocmask */

  (void)sigfillset(&allsigs);
  status = sigprocmask(SIG_SETMASK, NULL, &oldset);
  if (status != OK)
    {
      printf("Wake up: ERROR sigprocmask failed, status=%d\n",
          status);
    }

  if (oldset != allsigs)
    {
      printf("Wake up: ERROR sigprocmask=%x expected=%x\n",
          oldset, allsigs);
    }
}

static int waiter_main(int argc, char *argv[])
{
  sigset_t set;
  struct sigaction act;
  struct sigaction oact;
  int status;

  printf("Waiter task started!\n");
  printf("Unmasking signal %d\n", WAKEUP_SIGNAL);
  (void)sigemptyset(&set);
  (void)sigaddset(&set, WAKEUP_SIGNAL);
  status = sigprocmask(SIG_UNBLOCK, &set, NULL);
  if (status != OK)
    {
      printf("waiter called sigprocmask failed!\n");
    }

  printf("Waiter register signal handler!\n");
  act.sa_sigaction = wakeup_action;
  act.sa_flags  = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, WAKEUP_SIGNAL);

  status = sigaction(WAKEUP_SIGNAL, &act, &oact);
  if (status != OK)
    {
      printf("Waiter sigaction failed!\n");
    }

  printf("Waiting on semaphore!\n");

  status = sem_wait(&sem);
  if (status != 0)
    {
      int error = errno;
      if (error == EINTR)
        {
          printf("Waiter: sem_wait() successfully interrupted by signal\n" );
        }
      else
        {
          printf("Waiter: ERROR sem_wait failed, errno=%d\n" , error);
        }
    }
  else
    {
      printf("Waiter: ERROR awakened with no error!\n" );
    }

  act.sa_handler = SIG_DFL;
  (void)sigaction(WAKEUP_SIGNAL, &act, &oact);

  printf("waiter_main: done\n" );

  threadexited = true;
  return 0;
}

static void signal_test(void)
{
  int status;
  sigset_t set;
  struct sigaction act;
  struct sigaction oact;
  struct sched_param param;
  union sigval sigvalue;
  pid_t waiterpid;

  printf("Unmasking SIGCHLD!\n");
  sem_init(&sem, 0, 0);

  (void)sigemptyset(&set);
  (void)sigaddset(&set, SIGCHLD);
  status = sigprocmask(SIG_UNBLOCK, &set, NULL);
  if (status != OK)
    {
      printf("Failed to call sigprocmask!\n");
    }

  printf("Register SIGCHLD handler!\n");
  act.sa_sigaction = death_of_child;
  act.sa_flags = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, SIGCHLD);

  status = sigaction(SIGCHLD, &act, &oact);
  if (status != OK)
    {
      printf("Failed to call sigaction!\n");
    }

  printf("Start waiter task!\n");
  status = sched_getparam(0, &param);
  if (status != OK)
    {
      printf("Failed to call sched_getparam\n");
      param.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }

  waiterpid = task_create("waiter", param.sched_priority, STACK_SIZE, waiter_main, NULL);
  if (waiterpid == ERROR)
    {
      printf("Failed to create task!\n");
    }
  else
    {
      printf("Create waiter task pid = %d\n", waiterpid);
    }

  sleep(2);

  printf("signaling: pid = %d, signo = %d, sigvalue = %d\n", waiterpid, WAKEUP_SIGNAL, SIGVALUE_INT);
  sigvalue.sival_int = SIGVALUE_INT;
#ifdef CONFIG_CAN_PASS_STRUCTS
  status = sigqueue(waiterpid, WAKEUP_SIGNAL, sigvalue);
#else
  status = sigqueue(waiterpid, WAKEUP_SIGNAL, sigvalue.sival_ptr);
#endif
  if (status != OK)
    {
      printf("sigqueue called failed!\n");
      task_delete(waiterpid);
    }

  sleep(2);

  if (!threadexited)
    {
      printf("waiter task did not exit!\n");
    }
  if (!sigreceived)
    {
      printf("signal handler did not run!\n");
    }

  act.sa_handler = SIG_DFL;
  (void)sigaction(SIGCHLD, &act, &oact);

  printf("signal test done!\n");
}

#define NLOOPS 32
static pthread_mutex_t mut;
static volatile int my_mutex = 0;
static unsigned long nloops[2] = {0, 0};
static unsigned long nerrors[2] = {0, 0};

static void *thread_func(FAR void *parameter)
{
  int id = (int)((intptr_t)parameter);
  int ndx = id - 1;
  int i;

  for (nloops[ndx] = 0; nloops[ndx] < NLOOPS; nloops[ndx]++)
    {
      int status = pthread_mutex_lock(&mut);
      if (status != 0)
        {
          printf("pthread mutex lock failed!\n");
        }
      if (my_mutex == 1)
        {
          printf("error thread = %d, my_mutex should be 0, but is: %d\n", id, my_mutex);
          nerrors[ndx]++;
        }
      my_mutex = 1;
      for (i = 0; i < 10; i++)
        {
          pthread_yield();
        }
      my_mutex = 0;
      status = pthread_mutex_unlock(&mut);
      if (status != 0)
        {
          printf("pthread mutex unlock failed!\n");
        }
    }
  pthread_exit(NULL);
  return NULL;
}

static void mutex_test(void)
{
  pthread_t thread1;
  pthread_t thread2;
  int status;

  printf("Initializing mutex!\n");
  pthread_mutex_init(&mut, NULL);

  printf("Start thread 1\n");
  status = pthread_create(&thread1, NULL, thread_func, (pthread_addr_t)1);
  if (status != 0)
    {
      printf("pthread create thread1 failed!\n");
    }

  printf("Start thread 2\n");
  status = pthread_create(&thread2, NULL, thread_func, (pthread_addr_t)2);
  if (status != 0)
    {
      printf("pthread create thread2 failed!\n");
    }

  pthread_join(thread1, NULL);
  pthread_join(thread2, NULL);

  printf("Mutex test finished!\n");
}

static pthread_mutex_t mutex;
static pthread_cond_t cond;
static volatile int data_available = 0;

static void *thread_waiter(void *parameter)
{
  int status;

  printf("Waiter thread: started!\n");

  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
      printf("waiter: Failed to call mutex lock!\n");
    }

  printf("Before wait!\n");
  status = pthread_cond_wait(&cond, &mutex);
  if (status != 0)
    {
      printf("waiter: Failed to call cond wait!\n");
    }

  if (data_available != 1)
    {
      printf("waiter: data_available should be 1 but is 0!\n");
    }
  else
    {
      printf("waiter: data_available is correct!\n");
    }

  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("waiter: Failed to call mutex unlock!\n");
    }

  return NULL;
}

static void *thread_signaler(void *parameter)
{
  int status;

  usleep(1000000);
  printf("Signaler thread: started!\n");

  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
      printf("signaler: Failed to call mutex lock!\n");
    }

  data_available = 1;
  printf("Send cond!\n");
  status = pthread_cond_signal(&cond);
  if (status != 0)
    {
      printf("signaler: Failed to call cond signal!\n");
    }

  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("signaler: Failed to call mutex unlock!\n");
    }

  return NULL;
}

static void cond_test(void)
{
  pthread_t waiter;
  pthread_t signaler;
  pthread_attr_t attr;
  struct sched_param sparam;
  int status;

  printf("Initializing mutex and cond!\n");
  status = pthread_mutex_init(&mutex, NULL);
  if (status != 0)
    {
      printf("pthread mutex init failed!\n");
    }

  status = pthread_cond_init(&cond, NULL);
  if (status != 0)
    {
      printf("pthread cond init failed!\n");
    }

  printf("Start waiter!\n");
  status = pthread_attr_init(&attr);
  if (status != OK)
    {
      printf("waiter pthread attr init failed!\n");
    }
  sparam.sched_priority = 100;
  status = pthread_attr_setschedparam(&attr,&sparam);
  if (status != OK)
    {
      printf("waiter pthread attr setschedparam failed!\n");
    }

  status = pthread_create(&waiter, &attr, thread_waiter, NULL);
  if (status != 0)
    {
      printf("pthread waiter create failed!\n");
    }

  printf("Start signaler!\n");
  status = pthread_attr_init(&attr);
  if (status != OK)
    {
      printf("signaler pthread attr init failed!\n");
    }

  sparam.sched_priority = 50;
  status = pthread_attr_setschedparam(&attr,&sparam);
  if (status != OK)
    {
      printf("signaler pthread attr setschedparam failed!\n");
    }

  status = pthread_create(&signaler, &attr, thread_signaler, NULL);
  if (status != 0)
    {
      printf("pthread signaler create failed!\n");
    }

  pthread_join(signaler, NULL);
  pthread_join(waiter, NULL);
}

static const char g_target[]         = "/mnt/fs";
const char g_source[] = MOUNT_DEVNAME;
static const char g_filesystemtype[] = "vfat";
static const char g_testfile[]      = "/mnt/fs/TestFile.txt";
static const char g_testmsg[]        = "This is a write test";

static void write_test_file(const char *path)
{
  int fd;

  /* Write a test file into a pre-existing file on the test file system */

  printf("write_test_file: opening %s for writing\n", path);

  fd = open(path, O_WRONLY|O_CREAT|O_TRUNC, 0644);
  if (fd < 0)
    {
      printf("write_test_file: ERROR failed to open %s for writing, errno=%d\n",
          path, errno);
    }
  else
    {
      int nbytes = write(fd, g_testmsg, strlen(g_testmsg));
      if (nbytes < 0)
        {
          printf("write_test_file: ERROR failed to write to %s, errno=%d\n",
              path, errno);
        }
      else
        {
          printf("write_test_file: wrote %d bytes to %s\n", nbytes, path);
        }
      close(fd);
    }
}

static void read_test_file(const char *path)
{
  char buffer[128];
  int  nbytes;
  int  fd;

  /* Read a test file that is already on the test file system image */

  printf("read_test_file: opening %s for reading\n", path);

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("read_test_file: ERROR failed to open %s, errno=%d\n",
          path, errno);
    }
  else
    {
      memset(buffer, 0, 128);
      nbytes = read(fd, buffer, 128);
      if (nbytes < 0)
        {
          printf("read_test_file: ERROR failed to read from %s, errno=%d\n",
              path, errno);
        }
      else
        {
          buffer[127]='\0';
          printf("read_test_file: Read \"%s\" from %s\n", buffer, path);
        }
      close(fd);
    }
}

static bool is_file_exist(const char *path)
{
  int fd;
  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      return false;
    }
  else
    {
      return true;
    }
}

#define BUFFER_SIZE (64 * 512)
static struct fat_format_s g_fmt = FAT_FORMAT_INITIALIZER;

static int create_dummy_ramdisk(void)
{
  FAR char *pbuffer;
  int ret;

  /* Allocate a buffer to hold the file system image. */

  pbuffer = (char*)malloc(BUFFER_SIZE);
  if (!pbuffer)
    {
      printf("create_ramdisk: Failed to allocate ramdisk of size %d\n",
             BUFFER_SIZE);
      return -ENOMEM;
    }

  /* Register a RAMDISK device to manage this RAM image */

  ret = ramdisk_register(1,
                         (FAR uint8_t *)pbuffer,
                         64,
                         512,
                         RDFLAG_WRENABLED | RDFLAG_FUNLINK);
  if (ret < 0)
    {
      printf("create_ramdisk: Failed to register ramdisk at %s: %d\n",
             g_source, -ret);
      free(pbuffer);
      return ret;
    }

  /* Create a FAT filesystem on the ramdisk */

  ret = mkfatfs(g_source, &g_fmt);
  if (ret < 0)
    {
      printf("create_ramdisk: Failed to create FAT filesystem on ramdisk at %s: %d\n",
             g_source, ret);
      /* free(pbuffer); -- RAM disk is registered */
      return ret;
    }

  return 0;
}

static void fs_test(void)
{
  int ret = 0;

  if (!is_file_exist(g_testfile))
    {
      ret = create_dummy_ramdisk();
      if (ret < 0)
        {
          printf("Failed to create dummy ramdisk!\n");
          return;
        }

      printf("Successfully create ramdisk!\n");

      printf("mount %s filesystem at target=%s with source=%s\n",
          g_filesystemtype, g_target, g_source);

      ret = mount(g_source, g_target, g_filesystemtype, 0, NULL);
      printf("mount returned %d\n", ret);
    }
  if (ret == 0)
    {
      write_test_file(g_testfile);
      read_test_file(g_testfile);
    }
}

static void rtc_test(void)
{
  struct rtc_time curtime;
  time_t nxtime;
  int fd;

  struct rtc_setalarm_s alarminfo =
  {
    .id  = 0,
    .pid = 0,
    .event =
    {
      .sigev_notify            = SIGEV_SIGNAL,
      .sigev_signo             = SIGALRM,
      .sigev_value.sival_int   = 0,
#ifdef CONFIG_SIG_EVTHREAD
      .sigev_notify_function   = NULL,
      .sigev_notify_attributes = NULL,
#endif
    }
  };

  fd = open("/dev/rtc0", 0);
  if (fd < 0)
  {
    printf("%s, error, line %d\n", __func__, __LINE__);
    return;
  }

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  nxtime = mktime((FAR struct tm *)&curtime) + 2;
  gmtime_r(&nxtime, (FAR struct tm *)&alarminfo.time);
  ioctl(fd, RTC_SET_ALARM, (unsigned long)&alarminfo);
  printf("Curtime: [%02d:%02d:%02d], should wakeup at [%02d:%02d:%02d]\n",
          curtime.tm_hour, curtime.tm_min, curtime.tm_sec,
          alarminfo.time.tm_hour, alarminfo.time.tm_min, alarminfo.time.tm_sec);

  sleep(10);

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  printf("Curtime: [%02d:%02d:%02d], ACT\n", curtime.tm_hour, curtime.tm_min, curtime.tm_sec);

  close(fd);
}

/****************************************************************************
 * Public function
 ****************************************************************************/
#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int fishdemo_main(int argc, char *argv[])
#endif
{

  printf("1. task test  \n######################################\n");
  task_test();
  printf("############################\n2. mqueue test\n######################################\n");
  thread_mq_test();
  printf("############################\n3. clock test \n######################################\n");
  clocks_test();
  printf("############################\n4. signal test\n######################################\n");
  signal_test();
  printf("############################\n5. mutex test \n######################################\n");
  mutex_test();
  printf("############################\n6. cond test  \n######################################\n");
  cond_test();
  printf("############################\n7. fs test    \n######################################\n");
  fs_test();
  printf("############################\n8. rtc test   \n######################################\n");
  rtc_test();

  return EXIT_SUCCESS;
}
