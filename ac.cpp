#include<stdio.h>

typedef struct{
	float inteiro;
}lista;

int main (){
	float comp;
	int quant;
	int i,controle=0;
	lista list[quant];
	
	printf("tamanho\n");
	scanf("%i",&quant);
	printf("\nlista\n");
	for(i=0;i<quant;i++){
		scanf("%f",&list[i].inteiro);
		}
	printf("comparar\t");
	scanf("%f",&comp);
	for(i=0;i<quant;i++){
		if(comp==list[i].inteiro){
			printf("S");
			controle=1;
		}
	}
	if(controle==0){
		printf("N");
	}
	
	return 0;
}

//gcc inteiro.c -o "int"
//./int
