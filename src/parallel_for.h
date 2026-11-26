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
