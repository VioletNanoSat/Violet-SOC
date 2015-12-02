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
#define min(a, b) (((a) < (b)) ? (a) : (b)) 
#define max(a, b) (((a) > (b)) ? (a) : (b)) 

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

void maximum(const double sig[], size_t n, double* max_, int* max_ind){
	for(int i=0; i<n;i++){
		if(*max_ >= 0){
			if(sig[i] > *max_){
				*max_ = sig[i];
				*max_ind = i;
			}
			else if(-1*sig[i] > *max_){
				*max_ = sig[i];
				*max_ind = i;
			}
		}
		else{
			if(sig[i] > -1 * (*max_)){
				*max_ = sig[i];
				*max_ind = i;
			}
			else if(sig[i] < *max_){
				*max_ = sig[i];
				*max_ind = i;
			}
		}
	}
}

void compute_h(const double vf[], double ocv, const double uf[], size_t n, double h[]){
	for (int i=0;i<n;i++){
		h[i]=vf[i]-ocv*uf[i];
	}
}

void compute_ocv(const double vf[], const double uf[], int max_ind, double* ocv){
	double max_vf = vf[max_ind];
	double max_uf = uf[max_ind];
	*ocv = max_vf/max_uf;
}
int main(void)
{
	double signal[] = { .05, .06, .07 };
	double kernel[] = { .27, .54, .37};
	double result[ELEMENT_COUNT(signal) + ELEMENT_COUNT(kernel) - 1];

	convolve(signal, ELEMENT_COUNT(signal), kernel, ELEMENT_COUNT(kernel), result);
	double samp_i[] = {-.01,-.01,-.01,-.01,-.01,-.01,-.01,-.01};
	double samp_v[] = {12.0, 11.995, 11.990, 11.985,11.980,11.975, 11.970, 11.965};
	double time[] = {0, 1, 2, 3, 4, 5, 6, 7};
	double f[ELEMENT_COUNT(time)];
	double i[ELEMENT_COUNT(time)];
	algorithm_1(samp_i,time, ELEMENT_COUNT(time),f,i);
	
	double result2[ELEMENT_COUNT(f) + ELEMENT_COUNT(i) - 1];
	double vf[ELEMENT_COUNT(f) + ELEMENT_COUNT(samp_v) - 1];
	double uf[ELEMENT_COUNT(f) + ELEMENT_COUNT(f) - 1];
	double u[] = {1,0,0,0,0,0,0,0};
	convolve(f, ELEMENT_COUNT(f), samp_i, ELEMENT_COUNT(samp_i), result2);
	convolve(f, ELEMENT_COUNT(f), samp_v, ELEMENT_COUNT(samp_v), vf);
	convolve(f, ELEMENT_COUNT(f), u, ELEMENT_COUNT(u), uf);
	
	double max_ = 0;
	int max_ind = 0;
	maximum(f, ELEMENT_COUNT(f), &max_, &max_ind);
	double ocv;
	compute_ocv(vf, uf, max_ind, &ocv);	
	double h[ELEMENT_COUNT(vf)];
	compute_h(vf,ocv,uf,ELEMENT_COUNT(vf),h);
	while(1);
	return 0;	
}

void isr_to_get_c_v(){
	// somewhere you want ...
	//   volatile float voltage_sample[t]
	//   volatile float current_sample[t]
	// ADC current sample --> current_sample[t]
	// ADC voltage sample --> voltage_sample[t]
	// t = t + 1;
	// if t = max, make sure we call calc_OCV(method)
	
}

void                                                                                                                                                                                                                                                                   