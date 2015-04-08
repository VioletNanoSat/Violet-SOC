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
#include <stdio.h>
#include <stddef.h>

void convolve(const double Signal[/* SignalLen */], size_t SignalLen,
const double Kernel[/* KernelLen */], size_t KernelLen,
double Result[/* SignalLen + KernelLen - 1 */])
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

void printSignal(const char* Name,
double Signal[/* SignalLen */], size_t SignalLen)
{
	size_t i;

	for (i = 0; i < SignalLen; i++)
	{
		printf("%s[%zu] = %f\n", Name, i, Signal[i]);
	}
	printf("\n");
}
#define ELEMENT_COUNT(X) (sizeof(X) / sizeof((X)[0]))


int delta_function(int i){
	if(i==0){
		return 1;
	}
	else{
		return 0;
	}
}

int step_function(int i){
	if(i>=0){
		return 1;
	}
	else{
		return 0;
	}
}

void algorithm_1(double sampled_i[], double f[]){
	int n=ELEMENT_COUNT(sampled_i);
	double f_norm[n];
	double i_norm[n];
	for(int i=0;i<n;i++){
		f_norm[i]=(double) delta_function(i)/sampled_i[0];
		i_norm[i]=(double) sampled_i[i]/sampled_i[0];
	
	for(int i=1;i<n;i++){
		for(int j=n;j>i;j--){
			f[j]=f[j]-f_norm[j-i]*i_norm[i];
			i_norm[j]=i_norm[j]-i_norm[j-i]*i_norm[i];
			}
		}
	}
}

void compute_uf(double f[], double uf[]){
	int size=ELEMENT_COUNT(f);
	double u[size];
	for(int i=0;i<size;i++){
		u[i]=step_function(i);
	}
	convolve(f,ELEMENT_COUNT(f),u,ELEMENT_COUNT(u),uf);
}

void compute_vf(double f[], double sampled_v[], double vf[]){
	convolve(f,ELEMENT_COUNT(f),sampled_v,ELEMENT_COUNT(sampled_v),vf);
}

double max_uf(double uf[], int max_index){
	double max=0;
	for(int i=0;i<ELEMENT_COUNT(uf);i++){
		if(uf[i]>max){
			max=uf[i];
			max_index=i;
		}
	}
	return max;
}

double compute_OCV(double vf[],double max, int max_index){
	return (double) vf[max_index]/max;
	
	
	/*int size=ELEMENT_COUNT(vf);
	for(int i=0;i<size;i++){
		OCV[i]=(double)vf[i]/max;
	}*/
}

int main (void)
{
	board_init();

	// Insert application code here, after the board has been initialized.
	
	double sampled_i[1200];
	double sampled_v[1200];
	double f[1200];
	double vf[1200];
	double uf[1200];
	algorithm_1(sampled_i,f);
	compute_vf(f,sampled_v,vf);
	compute_uf(f,uf);
	int max_index=0;
	double max=max_uf(uf,max_index);
	double OCV=compute_OCV(vf,max,max_index);
		

	return 0;
}
