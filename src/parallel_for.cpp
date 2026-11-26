// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "parallel_for.h"

#include "solver.h"

static void b2RunStepTask( void* taskContext )
{
	b2StepTask* task = static_cast<b2StepTask*>( taskContext );
	task->fcn( task->context );
}

void b2ResetTaskBatch( b2StepContext* stepContext )
{
	// 上一批任务保留为下一批的依赖，必须复制出来以免被新任务覆盖。
	for ( int i = 0; i < stepContext->batchTaskCount; ++i )
	{
		stepContext->joinStepTasks[i] = stepContext->stepTasks[stepContext->batchTaskBase + i];
		stepContext->joinTasks[i] = stepContext->tasks[stepContext->batchTaskBase + i];
		stepContext->joinTasks[i].context = stepContext->joinStepTasks + i;
	}
	stepContext->joinTaskBase = 0;
	stepContext->joinTaskCount = stepContext->batchTaskCount;
	stepContext->batchTaskBase = 1;
	stepContext->batchTaskCount = 0;
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

b2TaskBatch b2MakeTaskBatch( const b2StepContext* stepContext )
{
	return b2TaskBatch{
		stepContext->tasks + stepContext->batchTaskBase,
		stepContext->batchTaskCount,
		stepContext->joinTasks + stepContext->joinTaskBase,
		stepContext->joinTaskCount,
	};
}
