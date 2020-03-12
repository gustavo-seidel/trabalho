#include<stdio.h>
int main(){
	float numero;
	float fatorial;
	printf("fatorial\t");
	scanf("%f",&numero);
	if(numero==0){
		fatorial=1;
	}
	if(numero<0){
		printf("invalido");
	}else{
	
	for(fatorial=1;numero>0;numero--){
	
	fatorial=fatorial*numero;
}
printf("\n%f",fatorial);
}
	return 0;
}
