#include "toolchain_c.h"

#include <stdlib.h>

int toolchain_c17_allocate_and_free(int count)
{
	int* values;
	int index;
	int sum;

	if (count <= 0)
	{
		return 0;
	}

	values = (int*)malloc((size_t)count * sizeof(int));
	if (values == NULL)
	{
		return -1;
	}

	sum = 0;
	for (index = 0; index < count; ++index)
	{
		values[index] = index + 1;
		sum += values[index];
	}

	free(values);
	return sum;
}
