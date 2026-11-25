# Box2D

Box2D 是用于游戏的二维物理引擎。

版权归 Erin Catto 所有，许可证见 [LICENSE](LICENSE)。上游项目主页：<https://box2d.org>。

## 当前能力

当前可以检查 C17 与 C++20 工具链，并用 CTest 运行测试。程序可以查询库版本 3.2.0 和双精度设置，也可以安装、触发并恢复断言回调，使用默认输出或用户日志回调记录消息，设置每米长度单位，以及按 64 字节对齐分配和释放内存。长度单位只接受有限正数，非法值保持原值。分配统计会在释放后回到原值，也可以换成自定义分配器。扩容时保留已有内容。失败断言的回调返回 0 时不会进入调试中断。它不提供物理模拟。

## 构建

需要 CMake 3.22 或更高版本，以及支持 C17 和 C++20 的编译器。

```powershell
cmake -S . -B build-core -G "Visual Studio 17 2022" -A x64
cmake --build build-core --config RelWithDebInfo --parallel 8
.\build-core\bin\RelWithDebInfo\toolchain_check.exe
ctest --test-dir build-core -C RelWithDebInfo --output-on-failure --no-tests=error
```

工具链程序通过时打印 `C17 and C++20 toolchain check passed` 并返回 0。CTest 会运行 `unit_tests`。
