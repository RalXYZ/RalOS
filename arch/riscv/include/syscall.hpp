#pragma once

using uint64_t = unsigned long;
using uint32_t = unsigned int;

constexpr auto SYS_WRITE = 64;
auto sys_write(uint32_t fd, const char *buf, uint64_t count) -> uint64_t;

constexpr auto SYS_GETPID = 172;
auto sys_getpid() -> uint64_t;
