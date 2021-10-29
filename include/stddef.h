#pragma once

typedef __PTRDIFF_TYPE__ ptrdiff_t;
typedef __SIZE_TYPE__ size_t;

#ifndef __cplusplus
typedef __WCHAR_TYPE__ wchar_t;
#endif

#define NULL 0L
#define offsetof(type, member) __builtin_offsetof(type, member)

typedef __builtin_va_list va_list;

#define va_start(v,l)   __builtin_va_start(v,l)
#define va_end(v)       __builtin_va_end(v)
#define va_arg(v,l)     __builtin_va_arg(v,l)
#define va_copy(d,s)    __builtin_va_copy(d,s)
