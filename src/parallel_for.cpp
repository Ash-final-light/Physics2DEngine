// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "parallel_for.h"

#include "platform.h"
#include "solver.h"

static void b2RunStepTask( void* taskContext )
{
	b2StepTask* task = static_cast<b2StepTask*>( taskContext );
	b2StepContext* stepContext = task->stepContext;
	task->fcn( task->context );

	if ( task->deferred )
	{
		if ( b2AtomicFetchAddInt( &stepContext->treePending, -1 ) == 1 && stepContext->signalSemaphores &&
			 stepContext->treeSemaphore != NULL )
		{
			b2SignalSemaphore( stepContext->treeSemaphore );
		}
	}
	else if ( b2AtomicFetchAddInt( &stepContext->pendingCount, -1 ) == 1 && stepContext->signalSemaphores &&
			  stepContext->batchSemaphore != NULL )
	{
		b2SignalSemaphore( stepContext->batchSemaphore );
	}
}

void b2ResetTaskBatch( b2StepContext* stepContext )
{
	// 上一批的普通任务与延迟任务都保留为下一批的依赖，且必须复制出来以免被新任务覆盖。
	int base = stepContext->treeStaged ? 0 : stepContext->batchTaskBase;
	int count = stepContext->batchTaskCount + ( stepContext->treeStaged ? 1 : 0 );
	for ( int i = 0; i < count; ++i )
	{
		stepContext->joinStepTasks[i] = stepContext->stepTasks[base + i];
		stepContext->joinTasks[i] = stepContext->tasks[base + i];
		stepContext->joinTasks[i].context = stepContext->joinStepTasks + i;
	}
	stepContext->joinTaskBase = 0;
	stepContext->joinTaskCount = count;
	stepContext->batchTaskBase = 1;
	stepContext->batchTaskCount = 0;
	stepContext->treeStaged = false;
}

void b2AddTask( b2StepContext* stepContext, b2TaskType type, b2TaskCallbackFcn* fcn, void* context )
{
	int index = stepContext->batchTaskBase + stepContext->batchTaskCount;
	B2_ASSERT( 1 <= index && index < B2_STEP_TASK_CAPACITY );

	b2StepTask* stepTask = stepContext->stepTasks + index;
	stepTask->fcn = fcn;
	stepTask->context = context;
	stepTask->stepContext = stepContext;
	stepTask->deferred = false;

	stepContext->tasks[index] = b2Task{ b2RunStepTask, stepTask, type, false };
	stepContext->batchTaskCount += 1;
	stepContext->taskCount += 1;
}

void b2StageDeferredTask( b2StepContext* stepContext, b2TaskType type, b2TaskCallbackFcn* fcn, void* context )
{
	B2_ASSERT( stepContext->treeStaged == false );

	b2StepTask* stepTask = stepContext->stepTasks + 0;
	stepTask->fcn = fcn;
	stepTask->context = context;
	stepTask->stepContext = stepContext;
	stepTask->deferred = true;

	stepContext->tasks[0] = b2Task{ b2RunStepTask, stepTask, type, true };
	stepContext->treeStaged = true;
	stepContext->taskCount += 1;
	b2AtomicStoreInt( &stepContext->treePending, 1 );
}

b2TaskBatch b2MakeTaskBatch( const b2StepContext* stepContext )
{
	int base = stepContext->treeStaged ? 0 : stepContext->batchTaskBase;
	int count = stepContext->batchTaskCount + ( stepContext->treeStaged ? 1 : 0 );
	return b2TaskBatch{
		stepContext->tasks + base,
		count,
		stepContext->joinTasks + stepContext->joinTaskBase,
		stepContext->joinTaskCount,
	};
}

void b2RunTaskBatchWithTree( b2EnqueueTaskCallbackFcn* enqueue, b2FinishTaskCallbackFcn* finish, void* userContext,
							 b2TaskBatch batch, void** treeHandle )
{
	if ( batch.count <= 0 )
	{
		return;
	}

	void* handles[B2_STEP_TASK_CAPACITY] = {};
	void* tree = NULL;
	int lastIndex = batch.count - 1;
	B2_ASSERT( lastIndex < B2_STEP_TASK_CAPACITY );

	for ( int i = 0; i < lastIndex; ++i )
	{
		const b2Task* task = batch.tasks + i;
		void* handle = enqueue( task->fcn, task->context, userContext );
		if ( task->deferred )
		{
			tree = handle;
		}
		else
		{
			handles[i] = handle;
		}
	}

	// 最后一个任务在当前线程执行，避免尾巴等待。
	const b2Task* lastTask = batch.tasks + lastIndex;
	lastTask->fcn( lastTask->context );

	if ( finish != NULL )
	{
		for ( int i = 0; i < lastIndex; ++i )
		{
			if ( handles[i] != NULL )
			{
				finish( handles[i], userContext );
			}
		}
	}

	if ( treeHandle != NULL )
	{
		*treeHandle = tree;
	}
	else if ( tree != NULL && finish != NULL )
	{
		finish( tree, userContext );
	}
}

void b2RunTaskBatch( b2EnqueueTaskCallbackFcn* enqueue, b2FinishTaskCallbackFcn* finish, void* userContext,
					 b2TaskBatch batch )
{
	b2RunTaskBatchWithTree( enqueue, finish, userContext, batch, NULL );
}
