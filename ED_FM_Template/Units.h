#pragma once

inline long double operator"" _deg(long double x)
{
	return x * 0.01745329252;
}

inline long double operator"" _nauticalMile(long double x)
{
	return x * 1852.0;
}
