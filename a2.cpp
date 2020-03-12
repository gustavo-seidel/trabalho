#include<stdio.h>
int main (){
	float fat = 1,ent ;
	float acumulador = 1;
	printf("fatorial:\t");
	scanf("%f",&fat);
	if(fat==0){
		acumulador=1;
	}else{
	
	while(fat>0){
		acumulador=acumulador*fat;
		fat--;
	}
}
	printf("%f",acumulador);
	
	return 0;
}
//gcc acumulador.c -o "acc"
//./acc
