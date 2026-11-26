// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"
#include "box2d/types.h"

struct b2StepContext;

// 重置当前批次；上一批任务成为下一批必须等待的接合任务。
void b2ResetTaskBatch( b2StepContext* stepContext );

// 将一个任务加入当前批次。
void b2AddTask( b2StepContext* stepContext, b2TaskType type, b2TaskCallbackFcn* fcn, void* context );

// 生成当前批次的只读描述，包含本批任务与待接合的前批任务。
b2TaskBatch b2MakeTaskBatch( const b2StepContext* stepContext );

// 用外部 enqueue/finish 回调执行一个批次：除最后一个任务在本地执行外，
// 其余任务先入队，再按句柄等待完成。空句柄表示入队回调已同步执行任务。
void b2RunTaskBatch( b2EnqueueTaskCallbackFcn* enqueue, b2FinishTaskCallbackFcn* finish, void* userContext,
					 b2TaskBatch batch );
