#include<stdio.h>
int main(){
	 float acumulador = 1,fatorial,controle,n;
	printf("fatorial ");
	scanf("%f",&fatorial);
  
		if(fatorial == 0){
		fatorial = 1;
	}
while(fatorial>0){
		acumulador = acumulador * fatorial;
	
		fatorial--;
	}
	
	
	printf("\t\t%f",acumulador);
	
	
	
	return 0;
}
