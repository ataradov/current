// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#ifdef OS_LINUX
#include <pthread.h>
#endif
#ifdef OS_WINDOWS
#include <windows.h>
#endif
#include "os_thread.h"

/*- Definitions -------------------------------------------------------------*/
#define NS_PER_S          1000000000
#define NS_PER_MS         1000000
#define MS_PER_S          1000

/*- Types -------------------------------------------------------------------*/
#ifdef OS_LINUX
typedef struct OsThread
{
  pthread_t handle;
} OsThread;

typedef struct OsMutex
{
  pthread_mutex_t mutex;
} OsMutex;

typedef struct OsCond
{
  pthread_cond_t var;
} OsCond;
#endif

#ifdef OS_WINDOWS
typedef struct OsThread
{
  HANDLE handle;
} OsThread;

typedef struct OsMutex
{
  CRITICAL_SECTION *cs;
} OsMutex;

typedef struct OsCond
{
  CONDITION_VARIABLE *var;
} OsCond;
#endif

/*- Implementations ---------------------------------------------------------*/

#ifdef OS_LINUX
//-----------------------------------------------------------------------------
OsThread *os_thread_create(OsThreadHandler handler, void *data)
{
  OsThread *thread = os_alloc(sizeof(OsThread));
  pthread_create(&thread->handle, NULL, handler, data);
  return thread;
}

//-----------------------------------------------------------------------------
void os_thread_free(OsThread *thread)
{
  pthread_detach(thread->handle);
  os_free(thread);
}

//-----------------------------------------------------------------------------
void os_thread_join(OsThread *thread)
{
  pthread_join(thread->handle, NULL);
}

//-----------------------------------------------------------------------------
OsMutex *os_mutex_create(void)
{
  OsMutex *mutex = os_alloc(sizeof(OsMutex));
  pthread_mutex_init(&mutex->mutex, NULL);
  return mutex;
}

//-----------------------------------------------------------------------------
void os_mutex_free(OsMutex *mutex)
{
  pthread_mutex_destroy(&mutex->mutex);
  os_free(mutex);
}

//-----------------------------------------------------------------------------
void os_mutex_lock(OsMutex *mutex)
{
  pthread_mutex_lock(&mutex->mutex);
}

//-----------------------------------------------------------------------------
void os_mutex_unlock(OsMutex *mutex)
{
  pthread_mutex_unlock(&mutex->mutex);
}

//-----------------------------------------------------------------------------
OsCond *os_cond_create(void)
{
  OsCond *cond = os_alloc(sizeof(OsCond));
  pthread_cond_init(&cond->var, NULL);
  return cond;
}

//-----------------------------------------------------------------------------
void os_cond_free(OsCond *cond)
{
  pthread_cond_destroy(&cond->var);
  os_free(cond);
}

//-----------------------------------------------------------------------------
void os_cond_signal(OsCond *cond)
{
  pthread_cond_signal(&cond->var);
}

//-----------------------------------------------------------------------------
void os_cond_broadcast(OsCond *cond)
{
  pthread_cond_broadcast(&cond->var);
}

//-----------------------------------------------------------------------------
void os_cond_wait(OsCond *cond, OsMutex *mutex)
{
  pthread_cond_wait(&cond->var, &mutex->mutex);
}

//-----------------------------------------------------------------------------
void os_cond_wait_timed(OsCond *cond, OsMutex *mutex, int timeout)
{
  struct timespec ts;

  clock_gettime(CLOCK_REALTIME, &ts);

  ts.tv_sec  += (timeout / MS_PER_S);
  ts.tv_nsec += (timeout % MS_PER_S)* NS_PER_MS;
  ts.tv_sec  += ts.tv_nsec / NS_PER_S;
  ts.tv_nsec %= NS_PER_S;

  pthread_cond_timedwait(&cond->var, &mutex->mutex, &ts);
}
#endif // OS_LINUX

#ifdef OS_WINDOWS
//-----------------------------------------------------------------------------
OsThread *os_thread_create(OsThreadHandler handler, void *data)
{
  OsThread *thread = os_alloc(sizeof(OsThread));
  thread->handle = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)(void *)handler, data, 0, NULL);
  return thread;
}

//-----------------------------------------------------------------------------
void os_thread_free(OsThread *thread)
{
  CloseHandle(thread->handle);
  os_free(thread);
}

//-----------------------------------------------------------------------------
void os_thread_join(OsThread *thread)
{
  WaitForSingleObject(thread->handle, INFINITE);
}

//-----------------------------------------------------------------------------
OsMutex *os_mutex_create(void)
{
  OsMutex *mutex = os_alloc(sizeof(OsMutex));
  mutex->cs = os_alloc(sizeof(CRITICAL_SECTION));
  InitializeCriticalSection(mutex->cs);
  return mutex;
}

//-----------------------------------------------------------------------------
void os_mutex_free(OsMutex *mutex)
{
  DeleteCriticalSection(mutex->cs);
  os_free(mutex->cs);
  os_free(mutex);
}

//-----------------------------------------------------------------------------
void os_mutex_lock(OsMutex *mutex)
{
  EnterCriticalSection(mutex->cs);
}

//-----------------------------------------------------------------------------
void os_mutex_unlock(OsMutex *mutex)
{
  LeaveCriticalSection(mutex->cs);
}

//-----------------------------------------------------------------------------
OsCond *os_cond_create(void)
{
  OsCond *cond = os_alloc(sizeof(OsCond));
  cond->var = os_alloc(sizeof(CONDITION_VARIABLE));
  InitializeConditionVariable(cond->var);
  return cond;
}

//-----------------------------------------------------------------------------
void os_cond_free(OsCond *cond)
{
  os_free(cond->var);
  os_free(cond);
}

//-----------------------------------------------------------------------------
void os_cond_signal(OsCond *cond)
{
  WakeConditionVariable(cond->var);
}

//-----------------------------------------------------------------------------
void os_cond_broadcast(OsCond *cond)
{
  WakeAllConditionVariable(cond->var);
}

//-----------------------------------------------------------------------------
void os_cond_wait(OsCond *cond, OsMutex *mutex)
{
  SleepConditionVariableCS(cond->var, mutex->cs, INFINITE);
}

//-----------------------------------------------------------------------------
void os_cond_wait_timed(OsCond *cond, OsMutex *mutex, int timeout)
{
  SleepConditionVariableCS(cond->var, mutex->cs, timeout);
}
#endif // OS_WINDOWS

