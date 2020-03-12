#include<stdio.h>

 typedef struct{
 	float inteiro;
 }lista;
int main(){
 float comp;
 int i,quant;
 int cont;
lista list[quant];	 
	 
	 printf("quantidade");
	 scanf("\n%i",&quant);
	 printf("lista:\n");
	 for(i=0;i<quant;i++){
	 	scanf("%f",&list[i].inteiro);
	 	
		 }
	scanf("%f",&comp);
	for(i=0;i<quant;i++){
		if(comp==list[i].inteiro){
			printf("S");
			cont=1;
		}
	}
	if(cont==0){
	printf("N");
}
	return 0;
}


