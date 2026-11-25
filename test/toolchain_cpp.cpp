#include "toolchain_c.h"

#include <cstdio>
#include <span>

namespace
{
int SumSpan(std::span<const int> values)
{
	int sum = 0;
	for (int value : values)
	{
		sum += value;
	}
	return sum;
}

struct ProbeBuffer
{
	int* data;
	explicit ProbeBuffer(int count) : data(new int[count]()) {}
	~ProbeBuffer()
	{
		delete[] data;
	}
	ProbeBuffer(const ProbeBuffer&) = delete;
	ProbeBuffer& operator=(const ProbeBuffer&) = delete;
};
} // namespace

int main()
{
	constexpr int values[] = { 1, 2, 3, 4 };
	const int cppSum = SumSpan(std::span<const int>(values));
	const int cSum = toolchain_c17_allocate_and_free(4);
	ProbeBuffer probe(4);

	if (probe.data == nullptr || cppSum != 10 || cSum != 10)
	{
		std::fputs("toolchain check failed\n", stderr);
		return 1;
	}

	std::puts("C17 and C++20 toolchain check passed");
	return 0;
}
