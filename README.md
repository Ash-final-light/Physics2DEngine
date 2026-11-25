# Box2D

Box2D 是用于游戏的二维物理引擎。

版权归 Erin Catto 所有，许可证见 [LICENSE](LICENSE)。上游项目主页：<https://box2d.org>。

## 当前能力

当前仓库只包含 C17 与 C++20 工具链检查。程序会用 C17 分配并释放一小段内存，再用 C++20 的 `std::span` 对固定数组求和。它不提供物理模拟。

## 构建

需要 CMake 3.22 或更高版本，以及支持 C17 和 C++20 的编译器。

```powershell
cmake -S . -B build-core -G "Visual Studio 17 2022" -A x64
cmake --build build-core --config RelWithDebInfo --parallel 8
.\build-core\bin\RelWithDebInfo\toolchain_check.exe
```

通过时程序打印 `C17 and C++20 toolchain check passed` 并返回 0。
