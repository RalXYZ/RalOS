#pragma once

[[nodiscard]]
auto construct_u_mode_pgtbl() -> unsigned long*;

auto va_to_pa(const unsigned long va) -> unsigned long;
