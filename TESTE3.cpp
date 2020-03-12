#include<stdio.h>
typedef struct{
	float inteiro;
}lista;

int main(){
	float comparar;
	int i,controle=0;
	int quantidade;
	lista list[quantidade];

printf("tamanho da lista\n");
scanf("%i",&quantidade);
printf("lista:\n");
for(i=0;i<quantidade;i++){

	scanf("%f",&list[i].inteiro);
}
printf("o numero:");
scanf("%f",&comparar);
	for(i=0;i<quantidade;i++){
		if(comparar==list[i].inteiro){
			printf("S");
			controle=1;
		}
	}
	if(controle==0){
		printf("N");
	}
	
	return 0;
}
