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

#define ELEMENT_COUNT(X) (sizeof(X) / sizeof((X)[0]))

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

double delta_function(int i){
	if(i==0)
		return 1.0;
	else
		return 0.0;
}

void algorithm_1(const double sampled_i[], const double time[], size_t n, double f_norm[], double i_norm[]){
	for(int i=0;i<n;i++){
		f_norm[i] = delta_function(i)/sampled_i[0];
		i_norm[i] = sampled_i[i]/sampled_i[0];		
	}
	for(int i=1;i<n;i++){
		for(int j=n-1;j>=i;j--){
			f_norm[j]=f_norm[j]-f_norm[j-i]*i_norm[i];
			i_norm[j]=i_norm[j]-i_norm[j-i]*i_norm[i];
		}
	}
}


int main(void)
{
	double signal[] = { .05, .06, .07 };
	double kernel[] = { .27, .54, .37};
	double result[ELEMENT_COUNT(signal) + ELEMENT_COUNT(kernel) - 1];

	convolve(signal, ELEMENT_COUNT(signal), kernel, ELEMENT_COUNT(kernel), result);
	double samp_i[] = {.05, .01, .03, -.01};
	double time[] = {0, 1, 2, 3};
	double f[ELEMENT_COUNT(time)];
	double i[ELEMENT_COUNT(time)];
	algorithm_1(samp_i,time, ELEMENT_COUNT(time),f,i);
	
	double result2[ELEMENT_COUNT(f) + ELEMENT_COUNT(i) -1];
	convolve(f, ELEMENT_COUNT(f), samp_i, ELEMENT_COUNT(i), result2);
	while(1);
	return 0;
}
