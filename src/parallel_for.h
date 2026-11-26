// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"
#include "box2d/types.h"

struct b2StepContext;
struct b2World;

// 分阶段 parallel-for 调用此回调来处理一段元素区间。每个工作线程可能被调用
// 多次：工作被分成若干块，工作线程以原子方式领取，因此提前完成的工作线程会
// 领取下一块尚未认领的工作，而不是空闲等待。workerIndex 是工作线程身份，
// 同一工作线程的所有调用中保持稳定，因此可安全用作每线程状态的索引
// （例如 world->taskContexts.data + workerIndex）。
typedef void b2ParallelForCallback( int startIndex, int endIndex, int workerIndex, void* context );

typedef struct b2ParallelForShared
{
	b2AtomicInt nextBlock;
	int blockCount;
	int blockSize;
	int itemCount;
	b2ParallelForCallback* callback;
	void* context;
} b2ParallelForShared;

typedef struct b2ParallelForTask
{
	b2ParallelForShared* shared;
	int workerIndex;
} b2ParallelForTask;

void b2StageParallelFor( b2World* world, b2TaskType type, b2ParallelForCallback* callback, int itemCount, int minRange, void* context );

// 重置当前批次；上一批任务成为下一批必须等待的接合任务。
void b2ResetTaskBatch( b2StepContext* stepContext );

// 将一个任务加入当前批次。
void b2AddTask( b2StepContext* stepContext, b2TaskType type, b2TaskCallbackFcn* fcn, void* context );

// 将长时间运行的任务放入保留槽位，等待调用方在其他阶段接合。
void b2StageDeferredTask( b2StepContext* stepContext, b2TaskType type, b2TaskCallbackFcn* fcn, void* context );

// 生成当前批次的只读描述，包含本批任务与待接合的前批任务。
b2TaskBatch b2MakeTaskBatch( const b2StepContext* stepContext );

// 用外部 enqueue/finish 回调执行一个批次：除最后一个任务在本地执行外，
// 其余任务先入队，再按句柄等待完成。空句柄表示入队回调已同步执行任务。
void b2RunTaskBatch( b2EnqueueTaskCallbackFcn* enqueue, b2FinishTaskCallbackFcn* finish, void* userContext,
					 b2TaskBatch batch );

// 与 b2RunTaskBatch 相同，但把延迟任务的句柄交回调用方以便后续接合。
void b2RunTaskBatchWithTree( b2EnqueueTaskCallbackFcn* enqueue, b2FinishTaskCallbackFcn* finish, void* userContext,
							 b2TaskBatch batch, void** treeHandle );
