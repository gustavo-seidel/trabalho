#include<stdio.h>
 
 	unsigned int quantidade;
 	typedef struct // Cria uma STRUCT para armazenar os dados da lista
{
    int inteiro; //cria a variavel para a lista
} numero;
 //-----------------------------------------------------------------------------------
 int main(){
 	 int quantidade;//parametro para saber quantos numeros foram escritos
     int i,controle;//variaveis de controle
     int comparar;//variavel para comparar o valor incerido	
    numero lista[quantidade]; //cria o ventor lista com tamanho igual a "quantidade	
    
     while(1){
     	for(i=0;i<quantidade;i++){
		 
     	lista[i].inteiro=0;
      }
	 
	 	 
	 printf("\ninsira o tamanho da lista de inteiros\n");
	 scanf("%i",&quantidade);  //quantidade de numeros inteiros
      i=quantidade;            
         printf("lista:\n");
     for(;i>0;i--){  //lop ultilizado para alocar cada inteiro em uma posi��o do vetor
     	
	   scanf("%i",&lista[i].inteiro);//inserir os numeros da lista
	
    }
      printf("adivinhe o n�mero\n");
      scanf("%i",&comparar);     //contante para a compara��o
      for(controle = 0;i<quantidade;i++){ //varre o vetor de i at� a quantidade inserida posi��o por posi��o.
        if(comparar==lista[i].inteiro){ //condi��o para acertar,compara se o numero inserido � igual ao do vetor.
        	printf("S");
        	controle=1;
		}
		
		 	
	  }
	  if(controle == 0){
	   printf("N");
  }
}
 	
 	return 0;
 }
