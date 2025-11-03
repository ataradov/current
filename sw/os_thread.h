// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _OS_THREAD_H_
#define _OS_THREAD_H_

/*- Includes ----------------------------------------------------------------*/
#include "os.h"

/*- Types -------------------------------------------------------------------*/
typedef struct OsThread OsThread;
typedef struct OsMutex OsMutex;
typedef struct OsCond OsCond;
typedef void * (*OsThreadHandler)(void *data);

/*- Prototypes --------------------------------------------------------------*/
OsThread *os_thread_create(OsThreadHandler handler, void *data);
void os_thread_free(OsThread *thread);
void os_thread_join(OsThread *thread);

OsMutex *os_mutex_create(void);
void os_mutex_free(OsMutex *mutex);
void os_mutex_lock(OsMutex *mutex);
void os_mutex_unlock(OsMutex *mutex);

OsCond *os_cond_create(void);
void os_cond_free(OsCond *cond);
void os_cond_signal(OsCond *cond);
void os_cond_broadcast(OsCond *cond);
void os_cond_wait(OsCond *cond, OsMutex *mutex);
void os_cond_wait_timed(OsCond *cond, OsMutex *mutex, int timeout);

#endif // _OS_THREAD_H_

