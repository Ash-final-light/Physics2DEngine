// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"
#include "box2d/types.h"

typedef struct b2Scheduler b2Scheduler;

b2Scheduler* b2CreateScheduler( int workerCount );
void b2DestroyScheduler( b2Scheduler* scheduler );
void b2ResetScheduler( b2Scheduler* scheduler );

// 与 b2EnqueueTaskCallbackFcn、b2FinishTaskCallbackFcn 相同的签名。
void* b2SchedulerEnqueueTask( b2TaskCallbackFcn* task, void* taskContext, void* userContext );
void b2SchedulerFinishTask( void* userTask, void* userContext );
