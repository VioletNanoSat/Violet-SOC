/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <asf.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>

void convolve(const double Signal[/* SignalLen */], size_t SignalLen, const double Kernel[/* KernelLen */], size_t KernelLen, double Result[/* SignalLen + KernelLen - 1 */])
{
	size_t n;

	for (n = 0; n < SignalLen + KernelLen - 1; n++)
	{
		size_t kmin, kmax, k;

		Result[n] = 0;

		kmin = (n >= KernelLen - 1) ? n - (KernelLen - 1) : 0;
		kmax = (n < SignalLen - 1) ? n : SignalLen - 1;

		for (k = kmin; k <= kmax; k++)
		{
			Result[n] += Signal[k] * Kernel[n - k];
		}
	}
}

void printSignal(const char* Name, double Signal[/* SignalLen */], size_t SignalLen)
{
	size_t i;

	for (i = 0; i < SignalLen; i++)
	{
		printf("%s[%zu] = %f\n", Name, i, Signal[i]);
	}
	printf("\n");
}
#define ELEMENT_COUNT(X) (sizeof(X) / sizeof((X)[0]))
int main(void)
{
	double signal[] = { 1, 1, 1, 1, 1 };
	double kernel[] = { 1, 1, 1, 1, 1 };
	double result[ELEMENT_COUNT(signal) + ELEMENT_COUNT(kernel) - 1];

	convolve(signal, ELEMENT_COUNT(signal),
	kernel, ELEMENT_COUNT(kernel),
	result);
	while(1);
	return 0;
}
