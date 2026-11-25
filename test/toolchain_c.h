#ifndef TOOLCHAIN_C_H
#define TOOLCHAIN_C_H

// C17 工具链检查：分配一段整数，求和后释放。count <= 0 时不分配并返回 0。
#ifdef __cplusplus
extern "C"
{
#endif

int toolchain_c17_allocate_and_free(int count);

#ifdef __cplusplus
}
#endif

#endif
