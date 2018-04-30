// Resolve caixeiro viajante usando busca em profundidade limitada por custo
// Falta: implementar outras heurística para acelerar o tempo de busca

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <float.h>
#include <time.h>
#include <conio.h>
#include <string.h>

#define PRIMEIRO_VERTICE 0 // mudando-se este valor, pode ficar mais rápido!

#define R 6378.388
#define TO_RAD (3.1415926536 / 180)

int verticePartida=0;

typedef struct no { int destino; double custo; struct no *prox; } *Lista;
typedef struct grafo { int tam; 
					   Lista *item;
					   double *x;
					   double *y;
                     } *Grafo;

double Matriz[25] = {0, 9, 8,30, 7,
             	9, 0, 6, 5, 1,
             	8, 6, 0,40, 9,
             	30,5,40, 0, 2,
             	7, 1, 9, 2, 0};

double M[36] = {0, 4, 100,100, 5,100,
             	4, 0, 1,  3,   2,100,
             100 , 1, 0,  5, 100, 2,
             100 ,3 ,5 ,  0, 100,100,
             	5, 2,100,100, 0  ,1,
			100  ,100, 2,100,  1  , 0};

double M2[36]={0,6,1,5,100,100,
			  6,0,2,100,5,100,  		
 			  1,2,0,2,6,4,
			  5,100,2,0,100,4,
	          100,5,6,100,0,3,
	  		  100,100,4,4,3,0};
// Gera matriz de custos, cada um deles entre 1 e 1000

double *gera_matriz(int total) {
   int n = sqrt(total);
   double *g = malloc(total*sizeof(double));
   for(int i=0; i<n; i++)
      for(int j=i; j<n; j++)
         g[i*n+j] = g[j*n+i] = ( i==j ) ? 0 : 1+rand()%999;
   return g;
}                                                                                                                                                                                                                                           

Lista no(int d, double c, Lista p) {
   Lista n = malloc(sizeof(struct no));
   n->destino = d;
   n->custo = c;
   n->prox = p;
   return n;
}

void insere(int d, double c, Lista *L) {
   if( *L==NULL || c<=(*L)->custo ) *L = no(d,c,*L);
   else insere(d,c,&(*L)->prox);
}

void exibe_vizinhos(Lista L) {
   printf("[");
   while( L ) { 
      printf("(%d,%lf)",L->destino+1,L->custo); 
      if( L->prox ) printf(",");
      L = L->prox; 
   }
   printf("]");
}

void exibe_inversa(Lista L) {
   if( L==NULL ) return;
   exibe_inversa(L->prox);
   if( L->prox ) printf(",");
   printf("%d",L->destino+1);
}

double custo_ultima_aresta(Lista L, Grafo G) {
   int ultimo_vertice = L->destino;
   for(Lista p=G->item[ultimo_vertice]; p; p = p->prox)
      if( p->destino == verticePartida) 
         return p->custo;
   return -1; // este comando nunca deve ser executado
}


void exibeCaminho(Lista L, Grafo G) {
   printf("\n[");
   exibe_inversa(L);
   printf("] : %lf",L->custo+custo_ultima_aresta(L,G));
}

//Apartir de um vertice dado, verifica o custo usando o algortimo do vizinho mais proximo
double custoCaminho(int vertice_inicio, int vertice_anterior, int vertice_partida, int *vertices, Grafo g){
	double custo=0;
	int continua=1;
	for(Lista p = g->item[vertice_partida]; p; p = p->prox)
		if(p->destino == vertice_anterior) custo+=p->custo;
	while(continua){
		for(Lista p = g->item[vertice_partida]; p; p = p->prox){
			if(!vertices[p->destino]){
				custo+=p->custo;
				vertice_partida=p->destino;
				vertices[p->destino]=1;
				continua=1;
				break;
			}else{
				continua=0;
			}
		}
	}
	for(Lista p = g->item[vertice_partida]; p; p = p->prox) 
		if(p->destino == vertice_inicio) custo+=p->custo;
	return custo;
}

//Procura melhor vertice para começar a busca;
//Dado um grafo procura o menor caminho a partir de cada vertices, produzindo um unico caminho por vertice
int buscaMelhorVertice(Grafo G){
	int melhorVertice=0;
	double melhorCaminho =DBL_MAX;
	int *vertices = malloc(G->tam*sizeof(int));
	//for(Lista p = c; p; p = p->prox) vertices[p->destino]=1;
	for(int i=0;i<G->tam;i++){
		for(int i=0;i<G->tam;i++) vertices[i]=0;
		vertices[i]=1;
		double novoCaminho = custoCaminho(i,-1,i,vertices,G);
		if(novoCaminho<melhorCaminho){ 
			melhorVertice=i;
			melhorCaminho = novoCaminho;
		}
	}
	free(vertices);
	return melhorVertice;
}

//Busca a arvores geradora minima utilizando o algortimo de Kruskal
//Orig passar qualquer vertice que esteja dentro da MST
double algKruskal(Grafo g, int *pai){
	int i,j, orig, dest, NV=g->tam;
	double total=0;
	int *arv = (int *) malloc(NV*sizeof(int));
	for(i=0;i<NV;i++){
		arv[i] = i;// Cada vertice sem pai está na sua propria arvore
		if(pai[i]!=-2) orig=i;
	}
	pai[orig] = orig;
	while(1){
		double menorPeso = DBL_MAX;
		for(i=0;i<NV;i++){//percorre os vertices
			if(pai[i] == -2) continue; // Pula os vertices que não participam na MST
			for(Lista p = g->item[i]; p; p = p->prox){//arestas
				//printf("\nValor de i: %d, valor da arvore de i:%d,valor da arvore de destino: %d, valor do pai de destino:%d\n",i,arv[i],arv[p->destino],pai[p->destino]);
				if(arv[i] != arv[p->destino] && menorPeso > p->custo && pai[p->destino]!= -2){
					menorPeso = p->custo;
					orig = i;
					dest=p->destino;
					break;// Como as arestas já estão organizadas em ordem crescente, a primeira vez que essa condição é executada, tem-se certeza que é o minimo custo
				}
			}
		}
		if(menorPeso == DBL_MAX) break;
		if(pai[orig] == -1) pai[orig]=dest;
		else pai[dest]=orig;
		for(i=0;i<NV;i++){
			if(arv[i] == arv[dest]){
				arv[i] = arv[orig];
			}
		}
	}
	//Calcula o custo da MST
	for(int v=0;v<NV;v++){
		if(pai[v]==-2) continue;
		for(Lista p = g->item[v]; p; p = p->prox){
			if(p->destino == pai[v] && pai[pai[v]]!=v)total+=p->custo;
			else if(p->destino == pai[v] && v<pai[v])total+=p->custo;//Verificação para não somar arestas já somadas
		}
	}
	free(arv);
	for(i=0;i<NV;i++)if(pai[i]!=-2)pai[i]=-1;
	return total;
}

//Dado a coordernada de dois pontos que formam segmento e de um ponto isolado, retorna se o ponto está a esquerda do segmento.
int esquerda(double x1,double y1, double x2, double y2, double x3, double y3){
	return (((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))>=0);
}

//Dado as coordenadas dos pontos que formam dois segmento, retorna se os segmentos se cruzam.
int InterceptaSegmentos(double xA,double yA,double xB, double yB,double xC, double yC,double xD, double yD){
	return (esquerda(xC,yC,xD,yD,xA,yA) ^ esquerda(xC,yC,xD,yD,xB,yB)) & (esquerda(xA,yA,xB,yB,xC,yC) ^ esquerda(xA,yA,xB,yB,xD,yD)); 
}

//Dado um caminho percorrido, verifica se o vertice analisado forma um segmento que sobrepõe uma aresta
int verificaInterceptacao(Lista Caminho,Grafo g,int vert, int comprimento){
	int ultimo_Vertice = Caminho->destino;
	Lista c = Caminho->prox;
	double xA = g->x[ultimo_Vertice];
	double yA = g->y[ultimo_Vertice];
	double xB = g->x[vert];
	double yB = g->y[vert];
	if(comprimento<3) return 0;
	for(Lista p = c; p && p->prox ; p = p->prox){
		double xC = g->x[p->destino];
		double yC = g->y[p->destino];
		double xD = g->x[p->prox->destino];
		double yD = g->y[p->prox->destino];
		if(InterceptaSegmentos(xA,yA,xB,yB,xC,yC,xD,yD)) return 1;
	}
	return 0;
} 

//Calculo para coordenadas Geograficas - Usando a Documentação de calculo do TSPLIB
double geo(double th1, double ph1, double th2, double ph2)
{

	double PI = 3.141592;
	int deg = th1;
	double min = th1 - deg;
	double latitude_x1 = PI * (deg + 5.0 * min / 3.0 ) / 180.0;
	deg = ph1;
	min = ph1 - deg;
	double longitude_y1 = PI * (deg + 5.0 * min / 3.0 ) / 180.0;

	deg = th2;
	min = th2 - deg;
	double latitude_x2 = PI * (deg + 5.0 * min / 3.0 ) / 180.0;
	deg = ph2;
	min = ph2 - deg;
	double longitude_y2 = PI * (deg + 5.0 * min / 3.0 ) / 180.0;

	double RRR = 6378.388;
	double q1 = cos( longitude_y1 - longitude_y2 );
	double q2 = cos( latitude_x1 - latitude_x2 );
	double q3 = cos( latitude_x1 + latitude_x2 );
	return RRR * acos( 0.5*((1.0+q1)*q2 - (1.0-q1)*q3) ) + 1.0;
}

int distanciaEuclideana(double x1, double y1, double x2, double y2){
	double xd = x1 - x2;
	double yd = y1 - y2;
	return sqrt( xd*xd + yd*yd);
}
 

// O parâmetro n é a ordem da matriz (isto é, o número de vértices no grafo)
// Se g é NULL, gera um grafo G a partir de uma matriz aleatória de custos g
// Senão, gera um grafo G a partir dos valores que estão na matriz de custos dados g

Grafo grafo(double *g, int n) {
   int aleatorio = 0;
   if( g==NULL ) { 
      g = gera_matriz(n*n);
      aleatorio = 1;
   }
   Grafo G = malloc(sizeof(struct grafo));
   G->tam = n;
   G->item = malloc(n*sizeof(Lista));
   for(int i=0; i<n; i++) {
      G->item[i] = NULL;
      for(int j=0; j<n; j++)
         if( i!=j ) 
            insere(j,g[i*n+j],&G->item[i]);
   }
   if( aleatorio ) free(g);
   return G;
}

int pertence(int d,Lista L) {
   if( L==NULL ) return 0;
   if( d == L->destino ) return 1;
   return pertence(d,L->prox);
}

void exibeg(Grafo G) {
   for(int i=0; i<G->tam; i++) {
      printf("\n%d: ",i+1); 
      exibe_vizinhos(G->item[i]);
   }
}

void busca(Lista Caminho, int comprimento, Grafo G, int *pai) {
   static double custo_minimo;
   static int quantidade=0;
   int *paiCopia = (int *) malloc(G->tam*sizeof(int));
   for(int i=0;i<G->tam;i++)paiCopia[i]=pai[i];
   if( comprimento == 1 ) custo_minimo = DBL_MAX;
   if( comprimento == G->tam ) {
	  quantidade++;
	  double custo_atual = Caminho->custo+custo_ultima_aresta(Caminho,G); 
      if( custo_atual<custo_minimo ) {    
		 exibeCaminho(Caminho,G);
		 printf("\nTotal de caminhos percorridos para encontrar a resposta: %d",quantidade);
		 printf("\nHorario: %s\n",__TIME__);
         custo_minimo = custo_atual;
         return; 
      }
   }
   int corrente = Caminho->destino;
   for(Lista p = G->item[corrente]; p; p = p->prox){
      if( !pertence(p->destino,Caminho)
		  && p->custo+Caminho->custo < custo_minimo
		  && algKruskal(G,paiCopia)+Caminho->custo < custo_minimo
		  && !verificaInterceptacao(Caminho,G,p->destino,comprimento)) {
         Lista c = no(p->destino,p->custo+Caminho->custo,Caminho);
		 paiCopia[p->destino] =-2;
         busca(c,comprimento+1,G,paiCopia);
         free(c);
      }
	}
	free(paiCopia);
}

int main(void) {
   	Grafo G;
   	clock_t inicio;
	srand(time(NULL)); // inicia gerador de números aleatórios

 	FILE *fd;
    
	char arquivo[20];
    char buf[20];
    char comment[50];
    char ewformat[20];
    char ewtype[20];
    int i, j, N;
    double aux,x,y;	
    
    /* Parser do arquivo de entrada.
     * Interpreta dois tipos de arquivo: EXPLICIT/UPPER_ROW e arquivo de coordenadas euclideanas EUC_2D
     * Considera invalido se o arquivo nao iniciar com a palavra NAME */
	printf("Digite o nome do arquivo:");
	gets(arquivo);
    fd=fopen(arquivo,"r");
    fscanf(fd,"%s",buf);
    if(strcmp(buf,"NAME")!=0&&strcmp(buf,"NAME:")!=0){
        printf("O arquivo de entrada nao eh um arquivo TSP valido\n");
        exit(0);
    }

    /* Parser
     * O laco while termina quando acabar de ler o cabecalho */
    while(1){
		fscanf(fd,"%s",buf);
		if(!strcmp(buf,"COMMENT:"))
	 	   fgets(comment, 100, fd);
		else if(!strcmp(buf,"DIMENSION:")){
	 	   fscanf(fd," %d",&N);
		}else if(!strcmp(buf,"COMMENT")){
	 	   fscanf(fd," : ");
	  	  fgets(comment, 100, fd);
		}
		else if(!strcmp(buf,"DIMENSION")){
	 	   fscanf(fd," : %d",&N);
		}
		else if(!strcmp(buf,"EDGE_WEIGHT_FORMAT:")){
	 	   fscanf(fd," %s",ewformat);
		}
		else if(!strcmp(buf,"EDGE_WEIGHT_FORMAT")){
	 	   fscanf(fd," : %s",ewformat);
		}
		else if(!strcmp(buf,"EDGE_WEIGHT_TYPE:")){
	 	   fscanf(fd," %s",ewtype);
		}
		else if(!strcmp(buf,"EDGE_WEIGHT_SECTION"))
	 	   break;
		else if(!strcmp(buf,"NODE_COORD_SECTION"))
	 	   break;
		else if(!strcmp(buf,"EOF")){
	 	  printf("Erro no reconhecimento do arquivo\n");
	  	  printf("O campo EDGE_WEIGHT_FORMAT ou NODE_COORD_SECTION nao existe\n");
	      exit(0);
		}	    
    }
	
    /* Nao aceita arquivos que nao sao nem de coordenadas euclideanas EUC_2D e nem 
     * matriz diagonal superior UPPER_ROW. Tambem nao aceita o arquivo se for UPPER_ROW 
     * mas nao for tipo EXPLICIT */
    /*if(strcmp(ewformat,"FUNCTION") && strcmp(ewformat,"FULL_MATRIX") && strcmp(ewformat,"COORD_DISPLAY")){
		printf("Arquivo TSP nao suportado\n");
    	exit(0);
    }*/
    
    /* Imprime a linha de comentario extraida do arquivo de entrada 
     * A expressao simbolica serve para eliminar um espaco em branco deixado pelo parser no inicio */
    printf("\n%s",(comment[0]==' ')?comment+1:comment);
	printf("\n%s",ewtype);
    
	//Aloca matriz de custos
	double *m = malloc((N*N)*sizeof(double));

    /* Aloca memoria para os vetores e matrizes */
    double *xVert = malloc(sizeof(double) * N);
	double *yVert = malloc(sizeof(double) * N);
    if(!strcmp(ewtype,"GEO")){
		/* Arquivo de entrada tipo GEOGRAPHIC
	 	* obtem as distancias utilizando a latitute e longitude fornecidas */
		for(i=0;i<N;i++)
	    	fscanf(fd,"%d %lf %lf\n",&aux,&(xVert[i]),&(yVert[i]));

		for(int i=0; i<N; i++){
      		for(int j=i; j<N; j++){
				if(i==j){
					m[i*N+j] = m[j*N+i]=0;
				}else{
					double valor = geo(xVert[i],yVert[i],xVert[j],yVert[j]);
					m[i*N+j] = m[j*N+i] = floor(valor);
				}
			}
		}
    }else if(!strcmp(ewtype,"EUC_2D")){
		/* Arquivo de entrada tipo EUC_2D
	 	* obtem as distancias utilizando a x e y fornecidas */
		for(i=0;i<N;i++)
	    	fscanf(fd,"%d %lf %lf\n",&aux,&(xVert[i]),&(yVert[i]));

		for(int i=0; i<N; i++){
      		for(int j=i; j<N; j++){
				if(i==j){
					m[i*N+j] = m[j*N+i]=0;
				}else{
					double valor = distanciaEuclideana(xVert[i],yVert[i],xVert[j],yVert[j]);
					m[i*N+j] = m[j*N+i] = floor(valor);
				}
			}
		}
    }
    printf("Distancias lidas com sucesso!\n");

/*
	G=grafo(M2,6);
	exibeg(G);
	int pai[6];
	for(i=0;i<=6;i++)pai[i]=-1;
	printf("O resultado %lf foi retornado",algKruskal(G,pai));
	getchar();
*/

/*	printf("\nGrafo gerado a partir de matriz dada:\n");
   	G = grafo(Matriz,5);
   	exibeg(G);
	inicio = clock();
	int pais2[5];
	for(i=0;i<=5;i++)pais2[i]=-1;
	for(int i=0;i<5;i++)printf("\n%d %d",i,pais2[i]);
   	printf("\n\nCaminhos encontrados (o ultimo tem custo minimo):\n");
   	verticePartida = buscaMelhorVertice(G);
   	busca(no(verticePartida,0,NULL),1,G,pais2);
	printf("\n\nTempo de busca: %.1f\n", 1.0*(clock()-inicio)/CLOCKS_PER_SEC);
   	printf("\n\nPressione <enter>");
   	getchar();
   	_clrscr();
*/
	G = grafo(m,N);
	G->x=xVert; G->y=yVert;
	exibeg(G);
	inicio = clock();
	verticePartida = buscaMelhorVertice(G);
	int pais[N];
	for(i=0;i<=N;i++)pais[i]=-1;
	printf("\n\nCaminhos encontrados (o ultimo tem custo minimo):\n");
   	busca(no(verticePartida,0,NULL),1,G,pais);
	printf("\n\nTempo de busca: %.1f\n", 1.0*(clock()-inicio)/CLOCKS_PER_SEC);
	printf("\n\nPressione <enter>");
   	getchar();
   	_clrscr();
    
   	printf("Grafo com 3 vertices (gerado a partir de matriz aleatoria):\n");
   	G = grafo(NULL,3);
   	exibeg(G);
   	printf("\n\nCaminhos encontrados (o ultimo tem custo minimo):\n");
   	inicio = clock();
   	verticePartida = buscaMelhorVertice(G);
   	//busca(no(verticePartida,0,NULL),1,G);
   	printf("\n\nTempo de busca: %.1f\n", 1.0*(clock()-inicio)/CLOCKS_PER_SEC);
   	getchar();
   	_clrscr();

   	int n = 20;
   	printf("Grafo com %d vertices (gerado a partir de matriz aleatoria):",n);
   	G = grafo(NULL,n);
   	//exibeg(G);
   	printf("\n\nCaminhos encontrados (o ultimo tem custo minimo):\n");
   	inicio = clock();
   	verticePartida = buscaMelhorVertice(G);
   	//busca(no(verticePartida,0,NULL),1,G);
   	printf("\n\nTempo de busca: %.1f\n", 1.0*(clock()-inicio)/CLOCKS_PER_SEC);

   	return 0;
}
