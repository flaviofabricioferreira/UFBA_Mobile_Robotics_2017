//Codigo Trabalho UFBA Robotica Movel 20171
//Flavio Fabricio
//Jessica Motta
//Takaaki Oda Junior

//This is a ROS version of the standard Helloworld program.

#include <ros/ros.h> //This header defines the stantard ROS classes
#include <nav_msgs/GetMap.h>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <string>
#include "gnuplot-iostream.h"
#include "obstaculo.h"


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)


// Defines para setar cores na impressao do mapa
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

/*  if (!robot_model.compare("burger"))
  {
    turning_radius_ = 0.08;
    rotate_angle_ = 50.0 * DEG2RAD;
    front_distance_limit_ = 0.7;
    side_distance_limit_  = 0.4;
  }
  else if (!robot_model.compare("waffle"))
  {
    turning_radius_ = 0.1435;
    rotate_angle_ = 40.0 * DEG2RAD;
    front_distance_limit_ = 0.7;
   side_distance_limit_  = 0.6;
 }
*/



using namespace std; // uso de strings


//Declaração das variaveis que serão utilizadas com Mapa Importado do arquivo YAML
int rows;
int cols;
double mapResolution;
vector<vector<bool> > grid; // array de arrays de booleanos , ou seja, um array de duas dimensoes pra guardar o mapa
//vector<vector<int> > gridObstaculos; //
//vector<Obstaculo> listadeobstaculos;

vector< pair<int,int> > ListaCordenadasObstaculos; //  
vector< pair<int,int> > ListaCordenadasObstaculosNasAproximacoes;
vector< pair<double,double> > TrajetoriaCalculadaDouble;
vector< pair<int,int> > TrajetoriaCalculadaInt;
vector< pair<int,int> > TrajetoriaCalculadaIntValoresUnicos;

vector< pair<int,int> > PontoInicialRoboParaImpressao;
vector< pair<int,int> > PontoFinalRoboParaImpressao;

//Declaração das variaveis que serão utilizadas com Mapa Criado na mao para teste
int tamanho;
int linhas;// usado no mapa de teste
int colunas;// usado no mapa de teste
vector<vector<bool> > gridTeste;
double xIncialRobo;
double yIncialRobo;
pair<int,int> PosInicialRobo;
pair<int,int> PosAtualRoboInt;
pair<double,double> PosAtualRoboDouble;
pair<int,int> PosFinalRobo;
double xAtualRobo;
double yAtualRobo;
double xProximoPonto;
double yProximoPonto;
double xFinalRobo;
double yFinalRobo;
double LBurguer; //Distancia entre eixos = 160mm= 16cm = 0,16m
double RBurguer; //Raio das rodas = 66mm= 6,6cm = 0,066m
vector<vector<double> > trajetoria;
double N; //Parametro de Atracao
double B; //Parametro de Repulsão
double Dmax; // Distancia Maxima obstculo
double tS; //amostragem


//Declaração das funções que lidarão com o Mapa Importado do arquivo YAML
bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printGrid();

//Declaração das funções que lidarão com Mapa Criado na mao para teste
void inicializaGridTest(int size);
void printGridTest(bool comtrajetoria);
bool estaMenosdaMetadedeLdaReferencia();
void inicializaObstaculos();
void MostrarListaObstaculos();
void MostrarDistanciaObstaculosaRobo();
//void CalcularTrajetoria(double n,double B,double Dmax,double tS,int xd,int yd,int xRef,int yRef);
void CalcularTrajetoria(double xdkatual,double ydkatual,double n,double B,double Dmax,double tS,double xRef,double yRef);
double CalcularK(double Dmax,double xatualrobo,double yatualrobo);
double CalcularKx(double Dmax,double xatualrobo,double yatualrobo);
double CalcularKy(double Dmax,double xatualrobo,double yatualrobo);
double CompararPosAtualComFinal(double xa,double ya,double xf,double yf);
void imprimirTrajetoriaCalculada();
void detectarObstaculosGridTeste();
double CalcularDistanciaEntrePontos(double xi,double yi,double xf,double yf);
void converterTrajetoriaParaInteiro();
void removerValoresDuplicados();
void plotarMapaeTrajetoria();
void calcularObstaculosNasProximidades(double DistanciaProximidade,double xAtual,double yAtual);
void cliqueAlgumaTeclaPraContinuar();
void rodarInteracoesdeCalculodaTrajetoria(double N, double B ,double Dmax,double tS,double RangeDaRegiaoProxima,double DistanciaAtualproPontoReferenciaFinal, int LimiteRrepeticoesInteracao);
int checkQtdadeRepeticoesDistancia(vector<double> lista);
	
int main(int argc, char** argv){

	//Initialize the ROS System
	ros::init(argc, argv, "UFBA_ROS_20171_FJT");
	//Establish this program as a ROS node.
	ros::NodeHandle nh;


	PosInicialRobo = make_pair(100,220);
	PontoInicialRoboParaImpressao.push_back(PosInicialRobo);
	PosAtualRoboInt = PosInicialRobo;
        PosAtualRoboDouble = make_pair(PosInicialRobo.first,PosInicialRobo.second);
	//printf("\nPONTO ATUAL ROBO : {%f,%f} ",PosAtualRoboDouble.first,PosAtualRoboDouble.second);
	//cliqueAlgumaTeclaPraContinuar();
	PosFinalRobo = make_pair(200,225);
	PontoFinalRoboParaImpressao;
	PontoFinalRoboParaImpressao.push_back(PosFinalRobo);


	if(!requestMap(nh)) // Se a Função RequestMap retornar falso por algum problema, então sai do programa
	exit(-1);
//	printf("\naaaaaaaaaaa3");
	// Imprime na Tela o Mapa
	printGrid();
//	printf("\naaaaaaaaaaa4");



//	inicializaGridTest(tamanho);
	//printf("\ninicializa Grid Testes");
	//cliqueAlgumaTeclaPraContinuar();
	detectarObstaculosGridTeste();
	printf("\nObstaculos Detectados");
	//cliqueAlgumaTeclaPraContinuar();

	//inicializaObstaculos();
//	printGridTest(false);

	double N = 0.5; //Parametro de Atracao
	double B = 2; //Parametro de Repulsão
	double Dmax = 8; // Distancia Maxima obstculo
	double tS = 0.1; //amostragem
	double AreaRangeSensorRobo = 400;
	double DeltadeDistanciaPontoReferencia = 5;
	double LimiteRepeticoesPontoMinimoLocal = 10;

	//double RangeDaRegiaoProxima,double DistanciaAtualproPontoReferenciaFinal,int LimiteRrepeticoesInteracao
	rodarInteracoesdeCalculodaTrajetoria(N,B,Dmax,tS,AreaRangeSensorRobo,
					     DeltadeDistanciaPontoReferencia,LimiteRepeticoesPontoMinimoLocal);

	//xIncialRobo = 1;
	//yIncialRobo = 1;
	//xAtualRobo = xIncialRobo;
	//yAtualRobo = yIncialRobo;
	//xFinalRobo = 14;
	//yFinalRobo = 14;
	//LBurguer  = 1;
	//RBurguer = 1;
//	printf("\naaaaaaaaaaa1");
//	trajetoria.resize(2);
//	trajetoria[0].push_back(xIncialRobo);
//	trajetoria[1].push_back(yIncialRobo);
//	printf("\naaaaaaaaaaa2");
	// Faz o request do Mapa a partir do Serviço

/*
//	MostrarListaObstaculos();

	//printf("\n");
//	MostrarDistanciaObstaculosaRobo();
	
*/

	//cliqueAlgumaTeclaPraContinuar();
//	imprimirTrajetoriaCalculada();

	printf("\nConverter Trajetoria para Inteiro");
//	cliqueAlgumaTeclaPraContinuar();
//	converterTrajetoriaParaInteiro();

	printf("\nPlotar Função Seno");
	cliqueAlgumaTeclaPraContinuar();
	plotarMapaeTrajetoria();
//	removerValoresDuplicados();
//	printGridTest(true);


	

	
	//Send some output as a log message
	//ROS_INFO_STREAM("MAPA GERADO");

	//Fecha o Programa
	return 0;
}

void rodarInteracoesdeCalculodaTrajetoria(double N, double B ,double Dmax,double tS,double RangeDaRegiaoProxima,double DistanciaAtualproPontoReferenciaFinal, int LimiteRrepeticoesInteracao){

	//Calculo Trajetoria até atingir uma distancia minima do ponto de referencia
	int i=0;
	bool flagDistancia = false;
	bool flagRepeticao = false;
	vector<double> ultimasdistancias;
	do{
		//double xinicial;
		//double yinicial;
		//double xfinal;
		printf("\n\nInteração %d : \nPosicaoAtual {%.2f;%.2f} -->  ",i,
		       PosAtualRoboDouble.first,PosAtualRoboDouble.second);		

		//Varredura uma area proxima ao robo (tamanho 3) para analisar quais obstaculos estao nesta regiao
		calcularObstaculosNasProximidades(RangeDaRegiaoProxima,
						  PosAtualRoboDouble.first,PosAtualRoboDouble.second);
		
//		printf("\nCalculou os Obstaculos nas Proximades -> Ira Calcular Trajetoria");
//		cliqueAlgumaTeclaPraContinuar();		

		//printf("\n\n\n\nParametros de entrada N=%f,B=%f,Dmax=%f,tS=%f\n\n\n\n",N,B,Dmax,tS);
		//Calculo do proximo ponto da trajetoria considerando os obstaculos na proximidade
		CalcularTrajetoria(PosAtualRoboDouble.first,
				   PosAtualRoboDouble.second,
				   N,B,Dmax,tS,
				   PosFinalRobo.first,
				   PosFinalRobo.second);

		printf("\nProximaPosicao {%.2f;%.2f}" "||Posição Final Esperada {%d;%d}",
		       PosAtualRoboDouble.first,PosAtualRoboDouble.second,
		       PosFinalRobo.first,PosFinalRobo.second);

//		printf("\nCalculou o Proximo Ponto -> Fara o loop Novamente");
//		cliqueAlgumaTeclaPraContinuar();		
/*		if(i%100 ==0){
			cliqueAlgumaTeclaPraContinuar();
		}*/
 		
		double distancia = CompararPosAtualComFinal(PosAtualRoboDouble.first,
					   PosAtualRoboDouble.second,
					   PosFinalRobo.first,
				           PosFinalRobo.second);

		printf("\nDistancia = %f",distancia);
		// Testa se atingiu delta de distancia do ponto
		if(distancia < DistanciaAtualproPontoReferenciaFinal){
			flagDistancia = true;
			printf("\nParou as iterações de calculo porque foi atingido um ponto da trajetoria com Distancia de %f < %f !", 					distancia,DistanciaAtualproPontoReferenciaFinal);
		} 


		// Testa a distancia repetiu 
		ultimasdistancias.push_back(distancia);

		int repeticoes = checkQtdadeRepeticoesDistancia(ultimasdistancias);

		if( repeticoes > LimiteRrepeticoesInteracao ){
			flagRepeticao = true;			
			printf("\n\nParou o WHile pq Distancia se repetiu %d vezes ",repeticoes);
		}


	
		i++;
		 //Continua calculando a trajetoria até que a distancia do ponto atual ao ponto de referencia seja menor que 			um determinado valor estipulado  	
	} while (!(flagDistancia||flagRepeticao));

	printf("\nParou Realmente o WHile"); 	

}


int checkQtdadeRepeticoesDistancia(vector<double> lista){

	int contador = 0;
	for(int i=0;i<lista.size()-1;i++){
		if(lista[i+1] == lista[i]){
			contador = contador + 1;
		}
	}
	
	return contador;
	
}

void imprimirTrajetoriaCalculada(){
	printf("\n\n\n------Trajetoria Calculada----");
//	for(int i=0;i<trajetoria[0].size();i++){

//		printf("\nPonto %d = {%.2f,%.2f}",i,trajetoria[0][i],trajetoria[1][i]);
//	}

	for(int i=0;i<TrajetoriaCalculadaDouble.size();i++){

		printf("\nPonto  Trajetoria Calculadora %d = {%.2f,%.2f}",i,
		       TrajetoriaCalculadaDouble[i].first,
		       TrajetoriaCalculadaDouble[i].second);
	}



}

void calcularObstaculosNasProximidades(double DistanciaProximidade,double xAtual,double yAtual){

/*	for(int j=0;j<ListaCordenadasObstaculosNasAproximacoes.size();j++){
 
		ListaCordenadasObstaculosNasAproximacoes.erase(j);
	}
*/

	// Limpa a lista de obstaculos nas proximidades e testa se deu certo
	ListaCordenadasObstaculosNasAproximacoes.clear();

//	printf("\n-----LiMpando de Obstaculos nas Proximidades Vazia");
	for(int j=0;j<ListaCordenadasObstaculosNasAproximacoes.size();j++){
		printf("\nNAO LIMPOUUUUUUUU");
		printf("\nNAO LIMPOUUUUUUUU");
		printf("\nNAO LIMPOUUUUUUUU");
		printf("\nPonto Nao limpou%d = {%d,%d}",j,
			ListaCordenadasObstaculosNasAproximacoes[j].first,
			ListaCordenadasObstaculosNasAproximacoes[j].second);
	}

	
	// Analisa toda a lista de obstaculos para identificar quais estao dentro de um raio de proximidade
	for(int i=0;i<ListaCordenadasObstaculos.size();i++){
		double xobs = ListaCordenadasObstaculos[i].first;
		double yobs = ListaCordenadasObstaculos[i].second;
		double dist = CalcularDistanciaEntrePontos(xAtual,yAtual,xobs,yobs);
//		printf("DistanciaProximidade = %f",DistanciaProximidade)
		if ( dist <= DistanciaProximidade){
		   // printf("\nAdicionou {%f,%f} na lista de Obst. Aproximados por ter distancia %f < %f do valor atual do robo {%f,%f}",xobs,yobs,dist,DistanciaProximidade,xAtual,yAtual); 
		    ListaCordenadasObstaculosNasAproximacoes.push_back(make_pair(xobs,yobs));	    	
		}
	}


	// Imprime a lista de obstaculos identificados dentro de um raio de proximidade nas proximidades do robô
	//printf("\n-----Quantidade de Obstaculos nas Proximidades = :%d",ListaCordenadasObstaculosNasAproximacoes.size());

/*	for(int j=0;j<ListaCordenadasObstaculosNasAproximacoes.size();j++){
		printf("\nPonto Proximidade Obstaculo %d = {%d,%d}",j,
			ListaCordenadasObstaculosNasAproximacoes[j].first,
			ListaCordenadasObstaculosNasAproximacoes[j].second);
	}
*/	
}

double CalcularDistanciaEntrePontos(double xi,double yi,double xf,double yf){

	
	double fatorx = (xf-xi);
	//printf("\n\nFator1 xf - xi = %f - %f = %f",xf,xi,fatorx);
	double fatorx2 = fatorx*fatorx;
	//printf("\nFator2 (xf - xi)² = (%f - %f)² = %f",xf,xi,fatorx2);
	double fatory = (yf-yi);
	//printf("\nFator3 yf - yi = %f - %f = %f",yf,yi,fatory);
	double fatory2 = fatory*fatory;
	//printf("\nFator4 (yf - yi)² = (%f - %f)² = %f",yf,yi,fatory2);
	double distanciadospontos = sqrt(fatorx2 + fatory2);
	//printf("\nFator5 sqrt[(%f - %f)² + (%f - %f)²] = %f",xf,xi,yf,yi,distanciadospontos);

		return distanciadospontos;
}


double CompararPosAtualComFinal(double xi,double yi,double xf,double yf){

	double diferencaX = (xf-xi)*(xf-xi);
	double diferencaY = (yf-yi)*(yf-yi);
	double distanciadospontos = sqrt(diferencaX + diferencaX);
	
/*	printf("\nDiferença de %f entre os pontos atual da trejetoria {%.2f;%.2f}"
	" e o Final {%.2f;%.2f} esperado\n", distanciadospontos,xi,yi,xf,yf);*/
		
	return distanciadospontos;

/*	if(distanciadospontos < Distancia){
		return true;
	}else {
		return false;
	}
*/
}

void CalcularTrajetoria(double xdkatual,double ydkatual,double n,double B,double Dmax,double tS,double xRef,double yRef){
	double xdKmais1;
	double ydKmais1;
	double K = 0;
        double Kx = 0;
	double Ky = 0;
	K = CalcularK(Dmax,xdkatual,ydkatual);
	Kx = CalcularKx(Dmax,xdkatual,ydkatual);
	Ky = CalcularKy(Dmax,xdkatual,ydkatual);
	//printf("\n(K=%f / Kx=%f / Ky=%f) ",K,Kx,Ky);
	//printf("\n-------------------------------------------------");
	//cliqueAlgumaTeclaPraContinuar();

	//             fator1          fator2                   fator3           fator 4
	//xdKmais1 = (n*(tS/100)*K - B*(tS/100) + 1)*xdkatual + B*(tS/100)*xRef - n*(tS/100)*Kx;
	//ydKmais1 = (n*(tS/100)*K - B*(tS/100) + 1)*ydkatual + B*(tS/100)*yRef - n*(tS/100)*Ky;

	double fator1 = n*(tS/100)*K;
	//printf("\n(Fator 1 : n*(tS/100)*K = %f ",fator1);
 	double fator2 = B*(tS/100);
	//printf("\n(Fator 2 : B*(tS/100) = %f onde B=%f,tS=%f,",fator2,B,tS);
	double fator1menosfator2 = fator1 - fator2 + 1;
	//printf("\n(Fator 1 - 2 : (n*(tS/100)*K - B*(tS/100) + 1) = %f ",fator1menosfator2);
	
	//printf("\nxdkatual = %f ",xdkatual);
	double fator1menosfator2Xatual = fator1menosfator2*xdkatual;
	//printf("\n(**(Fator 1- 2)*xdatual : (n*(tS/100)*K - B*(tS/100) + 1)*xdkatual = %f ",fator1menosfator2Xatual);

	//printf("\nydkatual = %f ",ydkatual);
	double fator1menosfator2Yatual = fator1menosfator2*ydkatual;
	//printf("\n(**(Fator 1- 2)*ydatual : (n*(tS/100)*K - B*(tS/100) + 1)*ydkatual = %f ",fator1menosfator2Yatual);
	
	double fator3x = B*(tS/100)*xRef;
	//printf("\n(Fator 3x: B*(tS/100)*xRef = %f ",fator3x);
	double fator3y = B*(tS/100)*yRef;
	//printf("\n(Fator 3y: B*(tS/100)*yRef = %f ",fator3y);

	double fator4x = n*(tS/100)*Kx;
	//printf("\n(Fator 4x: n*(tS/100)*Kx = %f ",fator4x);
	double fator4y = n*(tS/100)*Ky;
	//printf("\n(Fator 4y: n*(tS/100)*Ky = %f ",fator4y);

	double fator3xmenosfator4x = fator3x - fator4x;
	//printf("\n(**(Fator 3-4)*x: B*(tS/100)*xRef - n*(tS/100)*Kx = %f ",fator3xmenosfator4x);
	double fator3ymenosfator4y = fator3y - fator4y;
	//printf("\n(**(Fator 3-4)*y: B*(tS/100)*yRef - n*(tS/100)*Ky = %f ",fator3ymenosfator4y);

	xdKmais1 = fator1menosfator2Xatual + fator3xmenosfator4x;
	ydKmais1 = fator1menosfator2Yatual + fator3ymenosfator4y;

	//printf("\nProximo ponto da trajetoria é {%f,%f}",xdKmais1,ydKmais1);
	//cliqueAlgumaTeclaPraContinuar();

//	trajetoria[0].push_back(xdKmais1);
//	trajetoria[1].push_back(ydKmais1);

	TrajetoriaCalculadaDouble.push_back(make_pair(xdKmais1,ydKmais1));

	PosAtualRoboDouble.first = xdKmais1;
	PosAtualRoboDouble.second = ydKmais1;
	//xAtualRobo = 	xdKmais1;
	//yAtualRobo = 	ydKmais1;
	
}

double CalcularK(double Dmax,double xatualrobo,double yatualrobo){

	double somaK =0;
	double dbi;
	vector<double> valores;

	//printf("\nNumero de Obstaculos proximos para calculo de K é de %d",ListaCordenadasObstaculosNasAproximacoes.size());


	for(int i=0;i<ListaCordenadasObstaculosNasAproximacoes.size();i++){
		//printf("\nCalcularK esta considerando valores atuais do robo como {%f,%f}",xatualrobo,yatualrobo);
		dbi = CalcularDistanciaEntrePontos(ListaCordenadasObstaculosNasAproximacoes[i].first,
			                           ListaCordenadasObstaculosNasAproximacoes[i].second,
				                   xatualrobo,
				                   yatualrobo);

	///	printf("\n dbi do obstaculo %d é %f",i,dbi);
		//cliqueAlgumaTeclaPraContinuar();		

		if(dbi>0 && dbi<Dmax){
			//Por enquanto nao foi considerado dbi > Dmax
			//printf("\n VALORES {dbi,xbi,ybbi} ={%f,%f,%f} ",valores[0],valores[1],valores[2]);
			//printf("\n dbi do obstaculo %d é %f",i,dbi);
			double parte1 = (1/Dmax);
			//printf("\n 1/Dmax = %f",parte1);
			double parte2 = (1/dbi);
			//printf("\n 1/dbi = %f",parte2);
			double parte1Menosparte2 = parte1 - parte2;
			//printf("\n (1/Dmax) - (1/dbi) = %f\n",parte1Menosparte2);
			double parte3 = parte1Menosparte2*parte1Menosparte2;
			//printf("\n ((1/Dmax) - (1/dbi))² = %f\n",parte3);
			somaK = somaK + parte3;	
			//printf("\n SOMA = %f\n",soma);
			//soma = soma + ((1/Dmax) - (1/dbi))*((1/Dmax) - (1/dbi));
		} else {
			somaK  = somaK + 0;	;
		}		
	
	}
	//printf("\nO valor de K =%f",somaK);
	return somaK;
}
/*
double CalcularK(double Dmax,double xatualrobo,double yatualrobo){

	double soma =0;
	double dbi;
	vector<double> valores;

	for(int i=0;i<listadeobstaculos.size();i++){
		//printf("\nCalcularK esta considerando valores atuais do robo como {%d,%d}",xatualrobo,yatualrobo);
		valores = listadeobstaculos[i].calculaDistanciasRoboAoObstaculo(xatualrobo,yatualrobo,false);		
		//dbi = listadeobstaculos[i].getMenordistancia();
		dbi = valores[0];
		//Por enquanto nao foi considerado dbi > Dmax
		//printf("\n VALORES {dbi,xbi,ybbi} ={%f,%f,%f} ",valores[0],valores[1],valores[2]);
		//printf("\n dbi do obstaculo %d é %f",i,dbi);
		double parte1 = (1/Dmax);
		//printf("\n 1/Dmax = %f",parte1);
		double parte2 = (1/dbi);
		//printf("\n 1/dbi = %f",parte2);
		double parte1Menosparte2 = parte1 - parte2;
		//printf("\n (1/Dmax) - (1/dbi) = %f\n",parte1Menosparte2);
		double parte3 = parte1Menosparte2*parte1Menosparte2;
		//printf("\n ((1/Dmax) - (1/dbi))² = %f\n",parte3);
		soma = soma + parte3;	
		//printf("\n SOMA = %f\n",soma);
//		soma = soma + ((1/Dmax) - (1/dbi))*((1/Dmax) - (1/dbi));
	}

	return soma;
}*/



//pagina 37 dissertacao
double CalcularKx(double Dmax,double xatualrobo,double yatualrobo){

	double somaKx =0;
	double xbi;	
	double dbi;
	vector<double> valores;

	//printf("\nNumero de Obstaculos proximos para calculo de Kx é de %d",ListaCordenadasObstaculosNasAproximacoes.size());

	for(int i=0;i<ListaCordenadasObstaculosNasAproximacoes.size();i++){
//		printf("\nCalcularKx esta considerand valores atuais do robo como {%f,%f}",xatualrobo,yatualrobo);
//		valores = listadeobstaculos[i].calculaDistanciasRoboAoObstaculo(xatualrobo,yatualrobo,false);		
		//xbi = listadeobstaculos[i].getXbiMenorDistancia();
		//dbi = listadeobstaculos[i].getMenordistancia();
		xbi = ListaCordenadasObstaculosNasAproximacoes[i].first;
		dbi = CalcularDistanciaEntrePontos(ListaCordenadasObstaculosNasAproximacoes[i].first,
			                           ListaCordenadasObstaculosNasAproximacoes[i].second,
		      				   xatualrobo,
			                	   yatualrobo);

		//printf("\n dbi do obstaculo %d é %f",i,dbi);
		//cliqueAlgumaTeclaPraContinuar();		

		if(dbi>0 && dbi<Dmax){
			//dbi = valores[0];
			//printf("\n VALORES {dbi,xbi,ybbi} ={%f,%f,%f} ",valores[0],valores[1],valores[2]);
			//printf("\n VALORES dbi = %f e xbi = %f ",dbi,xbi);
			//Por enquanto nao foi considerado dbi > Dmax
			//printf("\n xbi do obstaculo %d é %f",i,xbi);
			somaKx = somaKx + (xbi*((1/Dmax) - (1/dbi)))*(xbi*((1/Dmax) - (1/dbi)));
		} else {
			somaKx  = somaKx + 0;
		}		

	}	

	//printf("\nO valor de Kx =%f",somaKx);
	return somaKx;

}

/*
double CalcularKx(double Dmax,double xatualrobo,double yatualrobo){

	double soma =0;
	double xbi;	
	double dbi;
	vector<double> valores;

	for(int i=0;i<listadeobstaculos.size();i++){
		//printf("\nCalcularKx esta considerand valores atuais do robo como {%d,%d}",xatualrobo,yatualrobo);
		valores = listadeobstaculos[i].calculaDistanciasRoboAoObstaculo(xatualrobo,yatualrobo,false);		
		//xbi = listadeobstaculos[i].getXbiMenorDistancia();
		//dbi = listadeobstaculos[i].getMenordistancia();
		xbi = valores[1];
		dbi = valores[0];
		//printf("\n VALORES {dbi,xbi,ybbi} ={%f,%f,%f} ",valores[0],valores[1],valores[2]);
		//printf("\n VALORES dbi = %f e xbi = %f ",dbi,xbi);
		//Por enquanto nao foi considerado dbi > Dmax
		//printf("\n xbi do obstaculo %d é %f",i,xbi);
		soma = soma + (xbi*((1/Dmax) - (1/dbi)))*(xbi*((1/Dmax) - (1/dbi)));
	}	

	return soma;

}*/

double CalcularKy(double Dmax,double xatualrobo,double yatualrobo){

	double somaKy =0;
	double ybi;	
	double dbi;
	vector<double> valores;

	//printf("\nNumero de Obstaculos proximos para calculo de Ky é de %d",ListaCordenadasObstaculosNasAproximacoes.size());

	for(int i=0;i<ListaCordenadasObstaculosNasAproximacoes.size();i++){
//		printf("\nCacularKy esta considerand valores atuais do robo como {%f,%f}",xatualrobo,yatualrobo);
		//valores = listadeobstaculos[i].calculaDistanciasRoboAoObstaculo(xatualrobo,yatualrobo,false);		
		//ybi = listadeobstaculos[i].getYbiMenorDistancia();
		//dbi = listadeobstaculos[i].getMenordistancia();
		ybi = ListaCordenadasObstaculosNasAproximacoes[i].second;
		
		dbi = CalcularDistanciaEntrePontos(ListaCordenadasObstaculosNasAproximacoes[i].first,
		      ListaCordenadasObstaculosNasAproximacoes[i].second,
		      xatualrobo,
		      yatualrobo);

//		printf("\n dbi do obstaculo %d é %f",i,dbi);
		//cliqueAlgumaTeclaPraContinuar();		



		if(dbi>0 && dbi<Dmax){
			//printf("\n ybi do obstaculo %d é %f",i,ybi);
//			printf("\n Interecao %d : 0 < DBI = (%f) < (%f)DBMAX .  ybi = %f",i,dbi,Dmax,ybi);
			somaKy = somaKy + (ybi*((1/Dmax) - (1/dbi)))*(ybi*((1/Dmax) - (1/dbi)));
		} else {
			somaKy  = somaKy + 0;
//			printf("\n Interecao %d : DBI = (%f) < (%f)DBMAX é %f",i,dbi,Dmax);
		}		


	}

	//printf("\nO valor de Ky =%f",somaKy);
	return somaKy;
}


/*
double CalcularKy(double Dmax,double xatualrobo,double yatualrobo){

	double soma =0;
	double ybi;	
	double dbi;
	vector<double> valores;

	for(int i=0;i<listadeobstaculos.size();i++){
		//printf("\nCacularKy esta considerand valores atuais do robo como {%d,%d}",xatualrobo,yatualrobo);
		valores = listadeobstaculos[i].calculaDistanciasRoboAoObstaculo(xatualrobo,yatualrobo,false);		
		//ybi = listadeobstaculos[i].getYbiMenorDistancia();
		//dbi = listadeobstaculos[i].getMenordistancia();
		ybi = valores[2];
		dbi = valores[0];
		//printf("\n VALORES {dbi,xbi,ybbi} ={%f,%f,%f} ",valores[0],valores[1],valores[2]);
		//printf("\n VALORES dbi = %f e ybi = %f ",dbi,ybi);
	
		//printf("\n ybi do obstaculo %d é %f",i,ybi);
		soma = soma + (ybi*((1/Dmax) - (1/dbi)))*(ybi*((1/Dmax) - (1/dbi)));
	}

	return soma;
}*/

//Implementação das Funções Propriamente Ditas

bool requestMap(ros::NodeHandle &nh)
{
// Requisitor(Request) do Servico
	nav_msgs::GetMap::Request req; 
//Resposta(Response) do Serviço
	nav_msgs::GetMap::Response res;

// Imprime que está esperando serviço ficar dispinovel enquanto não receber resposta do serviço
	while(!ros::service::waitForService("static_map",ros::Duration(3.0))){
	ROS_INFO("Waiting for Service static_map to become available");
	}

	ROS_INFO("Requesting the map...");

	//Requisita o Serviço staticMap
	ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
// nav_msgs é o nome do pacote
//static_map é o nome do serviço

	if(mapClient.call(req,res)){
		readMap(res.map);
		return true;
	}
	else{
		ROS_ERROR("Failed to call map service");
		return false;	
	}
}


void readMap(const nav_msgs::OccupancyGrid& map)
{
	ROS_INFO("Received a %d X %d map @ %.3f m/px\n",
		map.info.width,
		map.info.height,
		map.info.resolution);

	rows = map.info.height;
	cols = map.info.width;
	mapResolution = map.info.resolution;

// Redimensionamento dinamico da mapa(grid),ou seja, cria um array do tamanho do mapa importado

	gridTeste.resize(rows);
	for(int i=0;i<rows;i++){
		gridTeste[i].resize(cols);	
	}

	int currCell = 0;

	for(int i=0;i<rows;i++){
		for(int j=0;j<cols;j++){
			if(map.data[currCell]==0) // Celula Livre
				gridTeste[i][j] = false;
			else // Celula Ocupada = 100 ou indefinido (-1)
				gridTeste[i][j] = true;
			currCell++;
		}
	}


}

void inicializaGridTest(int tamanho){


        linhas = 20;
	colunas = 20;
	
	gridTeste.resize(linhas);
	for(int i=0;i<linhas;i++){
		gridTeste[i].resize(colunas);	
	}

	for(int i=0;i<linhas;i++){
		for(int j=0;j<colunas;j++){
			gridTeste[i][j] = false;
			if(i==0 || j ==0 || i==19 || j==19 ){
				gridTeste[i][j] = true;
			} else{
				gridTeste[i][j] = false;
			}
		}
	}



/*	//Criar obstaculo 1
	for(int j=(50);j<(350);j++){		
		gridTeste[j][200] = true;// Celula Ocupada = 100 ou indefinido (-1)
	}*/
/*
	//Criar obstaculo 2
	for(int j=5;j<15;j++){		
		gridTeste[15][j] = true;// Celula Ocupada = 100 ou indefinido (-1)
	}

	//Criar obstaculo 3
	for(int k=9;k>5;k--){		
		gridTeste[k][k] = true;// Celula Ocupada = 100 ou indefinido (-1)
	}
*/
	//Criar obstaculo 4
/*
		gridTeste[5][5] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][6] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][7] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][8] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][9] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[6][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[7][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[8][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[9][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[10][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[11][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[12][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[13][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[14][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][9] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][8] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][7] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][6] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][5] = true;// Celula Ocupada = 100 ou indefinido (-1)
*/

/*
		gridTeste[5][5] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][6] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][7] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][8] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][9] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[5][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[6][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[7][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
	//	gridTeste[8][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
	//	gridTeste[9][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
	//	gridTeste[10][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
	//	gridTeste[11][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
	//	gridTeste[12][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
	//	gridTeste[13][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[14][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][9] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][8] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][7] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][6] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][5] = true;// Celula Ocupada = 100 ou indefinido (-1)
*/


		//gridTeste[5][5] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[5][6] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[5][7] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[5][8] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[5][9] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[5][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[6][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[7][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[8][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[9][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[10][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[11][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[12][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[13][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[14][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		gridTeste[15][10] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[15][9] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[15][8] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[15][7] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[15][6] = true;// Celula Ocupada = 100 ou indefinido (-1)
		//gridTeste[15][5] = true;// Celula Ocupada = 100 ou indefinido (-1)




}

void detectarObstaculosGridTeste(){
	
//	printf("\na1");
	int linhas = gridTeste.size();
//	printf("\na2  Linhas = %d ",linhas);
	int colunas = gridTeste[0].size();		
//	printf("\na3 Colunas  = %d ",colunas);
	for(int x=0;x<linhas;x++){

//		printf("\na5 ");

		for(int y=0;y<colunas;y++){
			//printf("\n grid[x][y] %B=",grid[x][y]);
			if(gridTeste[x][y]){
//					printf("\nDeteccao Obstaculo (i=%d,j=%d) ",x,y);
	//				gridObstaculos[0].push_back(i);
	//				gridObstaculos[1].push_back(j);
					ListaCordenadasObstaculos.push_back(make_pair(x,y));
			}

		}

	}
	
	printf("\nLista de Obstaculos:--------------------------");
	printf("\nNumero de Pontos Obstaculos: %d--------------------",ListaCordenadasObstaculos.size());
	for(int i=0;i<ListaCordenadasObstaculos.size();i++){
		printf("\nObstaculo %d em {%d,%d} ",
		i,ListaCordenadasObstaculos[i].first,ListaCordenadasObstaculos[i].second);
	}
		

}


void printGrid(){
	printf("Grid map:\n");
	int freeCells = 0;
	for(int i=0;i<rows;i++)
	{
		printf("Row no. %d\n",i);
		for(int j=0;j<cols;j++)
		{
			printf("%d ",gridTeste[i][j] ? 1 : 0); // Imprime 1 pra true(ocupado ou indefinido) ou 0 para false(livre)
		}
		printf("\n");
	}
}

// Impressao de Mapa(Grid) criado na mao para teste
void printGridTest(bool ComTrajeotoria){
	printf("Impressao de Mapa(Grid) criado na Mão para teste:\n");
	int freeCells = 0;
	printf("\n---------------------------------------");
	printf("\n----------MAPA-DE-TESTE----------------");
	printf("\n---------------------------------------\n");
	//	printf("\na1");
	int linhas = gridTeste.size();
	printf("\na2  Linhas = %d ",linhas);
	int colunas = gridTeste[0].size();		
	printf("\na3 Colunas  = %d \n",colunas);
//	printf("\nLinhas numero. %d",linhas);
//	printf("\nColunas numero. %d\n",colunas);
//	//for(int i=0;i<linhas;i++)
	for(int y=colunas-1;y>-1;y--)
	{
		//printf("Linha numero. %d\n",i);
		
		//for(int j=0;j<colunas;j++)
		for(int x=0;x<linhas;x++)
		{
			
			if(x==PosInicialRobo.first && y==PosInicialRobo.second){
				printf(ANSI_COLOR_CYAN "I " ANSI_COLOR_RESET,gridTeste[x][y]);
			} else if(x==PosFinalRobo.first && y==PosFinalRobo.second){
				printf(ANSI_COLOR_GREEN "X "ANSI_COLOR_RESET,gridTeste[x][y]);
			}else {
				if(gridTeste[x][y]){
				printf(ANSI_COLOR_RED "%d "ANSI_COLOR_RESET,1); // Imprime 1 pra true(ocupado ou indefinido) 
				} else{
				printf("%d ",0); // Imprime 0 para false(livre)
				}

			}
			if(ComTrajeotoria){
				for(int k=0;k<TrajetoriaCalculadaInt.size();k++){
					if(x==TrajetoriaCalculadaInt[k].first && y==TrajetoriaCalculadaInt[k].second){
						printf(ANSI_COLOR_YELLOW "T " ANSI_COLOR_RESET,gridTeste[x][y]);
					}
			}	}

		}
		printf("\n");
	}
	printf("---------------------------------------\n");

}

void inicializaObstaculos(){
      // linha 0 = [x1,x2,...,xn]
      // linha 1 = [y1,y2,...,yn]

/*	vector<vector<double> > listaDePontos1;
	listaDePontos1.resize(2);
	listaDePontos1[0].resize(1);
	listaDePontos1[1].resize(1);
	listaDePontos1[0][0]= 5; //X
	listaDePontos1[1][0]= 10; //Y
	Obstaculo obstaculo1;
	obstaculo1.setListadePontosDoObstaculo(listaDePontos1);

	vector<vector<double> > listaDePontos2;
	listaDePontos2.resize(2);
	listaDePontos2[0].resize(1);
	listaDePontos2[1].resize(1);
	listaDePontos2[0][0]= 6; //X
	listaDePontos2[1][0]= 10; //Y
	Obstaculo obstaculo2;
	obstaculo2.setListadePontosDoObstaculo(listaDePontos2);

	vector<vector<double> > listaDePontos3;
	listaDePontos3.resize(2);
	listaDePontos3[0].resize(1);
	listaDePontos3[1].resize(1);
	listaDePontos3[0][0]= 17; //X
	listaDePontos3[1][0]= 5; //Y
	Obstaculo obstaculo3;
	obstaculo3.setListadePontosDoObstaculo(listaDePontos3);

	vector<vector<double> > listaDePontos4;
	listaDePontos4.resize(2);
	listaDePontos4[0].resize(1);
	listaDePontos4[1].resize(1);
	listaDePontos4[0][0]= 17; //X
	listaDePontos4[1][0]= 6; //Y
	Obstaculo obstaculo4;
	obstaculo4.setListadePontosDoObstaculo(listaDePontos4);

	listadeobstaculos.push_back(obstaculo1);
	listadeobstaculos.push_back(obstaculo2);
	listadeobstaculos.push_back(obstaculo3);
	listadeobstaculos.push_back(obstaculo4);
*/





	vector<vector<double> > listaDePontos1;
	listaDePontos1.resize(2);
	listaDePontos1[0].resize(10);
	listaDePontos1[1].resize(10);

	listaDePontos1[0][0]= 5;
	listaDePontos1[1][0]= 10;
	listaDePontos1[0][1]= 6;
	listaDePontos1[1][1]= 10;
	listaDePontos1[0][2]= 7;
	listaDePontos1[1][2]= 10;
	listaDePontos1[0][3]= 8;
	listaDePontos1[1][3]= 10;
	listaDePontos1[0][4]= 9;
	listaDePontos1[1][4]= 10;
	listaDePontos1[0][5]= 10;
	listaDePontos1[1][5]= 10;
	listaDePontos1[0][6]= 11;
	listaDePontos1[1][6]= 10;
	listaDePontos1[0][7]= 12;
	listaDePontos1[1][7]= 10;
	listaDePontos1[0][8]= 13;
	listaDePontos1[1][8]= 10;
	listaDePontos1[0][9]= 14;
	listaDePontos1[1][9]= 10;

	Obstaculo obstaculo1;
	//obstaculo1 = new Obstaculo();
	obstaculo1.setListadePontosDoObstaculo(listaDePontos1);

	vector<vector<double> > listaDePontos2;
	listaDePontos2.resize(2);
	listaDePontos2[0].resize(10);
	listaDePontos2[1].resize(10);



      // linha 0 = [x1,x2,...,xn]
      // linha 1 = [y1,y2,...,yn]

	listaDePontos2[0][0]= 17;
	listaDePontos2[1][0]= 5;
	listaDePontos2[0][1]= 17;
	listaDePontos2[1][1]= 6;
	listaDePontos2[0][2]= 17;
	listaDePontos2[1][2]= 7;
	listaDePontos2[0][3]= 17;
	listaDePontos2[1][3]= 8;
	listaDePontos2[0][4]= 17;
	listaDePontos2[1][4]= 9;
	listaDePontos2[0][5]= 17;
	listaDePontos2[1][5]= 10;
	listaDePontos2[0][6]= 17;
	listaDePontos2[1][6]= 11;
	listaDePontos2[0][7]= 17;
	listaDePontos2[1][7]= 12;
	listaDePontos2[0][8]= 17;
	listaDePontos2[1][8]= 13;
	listaDePontos2[0][9]= 17;
	listaDePontos2[1][9]= 14;

//	Obstaculo obstaculo2 = new Obstaculo(listaDePontos2);
	Obstaculo obstaculo2;
	obstaculo2.setListadePontosDoObstaculo(listaDePontos2);
//*/
}

/*void MostrarListaObstaculos(){

	for(int i=0;i<listadeobstaculos.size();i++){
		printf("\nObstaculo %d",i);
		listadeobstaculos[i].showListadePontos();
	}


}*/
/*void MostrarDistanciaObstaculosaRobo(){

	for(int i=0;i<listadeobstaculos.size();i++){
		listadeobstaculos[i].calculaDistanciasRoboAoObstaculo(xAtualRobo,yAtualRobo,false);
		
		printf("\nMenor Distancia do Robô ao Obstaculo %d ocorre em {%f,%f} e é de = %f\n",
		i+1,
		listadeobstaculos[i].getXbiMenorDistancia(),
		listadeobstaculos[i].getYbiMenorDistancia(),
		listadeobstaculos[i].getMenordistancia());

}



//	printf("Menor Distancia do Robô ao Obstaculo 2 é de = %f \n",listadeobstaculos[1].retornaMenorDistanciaDoObstaculo(xAtualRobo,YAtualRobo,false));

}*/

void converterTrajetoriaParaInteiro(){



	for(int i=0;i<TrajetoriaCalculadaDouble.size();i++){

		TrajetoriaCalculadaInt.push_back(make_pair(round(TrajetoriaCalculadaDouble[i].first),
							   round(TrajetoriaCalculadaDouble[i].second)));
	}

	printf("\n\n----Trajetoria Arredondada");
	for(int j=0;j<TrajetoriaCalculadaInt.size();j++){
		       
		       printf("\nPonto %d = {%d,%d}",j,
		       TrajetoriaCalculadaInt[j].first,
		       TrajetoriaCalculadaInt[j].second);
		
	}

		


}

void removerValoresDuplicados(){

	printf("\n\n----Trajetoria com Pontos Unicos:");
	pair<int,int> pontosDeTeste;
	int count =0;
	vector< pair<int,int> > TrajetoriaCalculadaIntValoresUnicos;

	TrajetoriaCalculadaIntValoresUnicos.push_back(TrajetoriaCalculadaInt[0]);

	for(int i=1;i<TrajetoriaCalculadaInt.size();i++){


		for(int j=0;j<TrajetoriaCalculadaIntValoresUnicos.size();j++){
				
			if(pontosDeTeste == TrajetoriaCalculadaIntValoresUnicos[j]){
				count = count + 1;
			}
		}
		if(count = 0){
			TrajetoriaCalculadaIntValoresUnicos.push_back(pontosDeTeste);
			printf("\nPonto Valores Duplicadors= {%d,%df}",
		        pontosDeTeste.first,
		        pontosDeTeste.second);
		}
	}

	

}

void plotarMapaeTrajetoria(){
	Gnuplot gp;
	//gp << "set xrange [-1:21]\n" << std::endl;
	//gp << "set yrange [-1:21]\n"  << std::endl;

	gp << "plot '-' with points pt 3 title 'Lista Cordenadas Obstaculos', '-' with lines lw 2 title 'Trajetoria Calculada APF', '-' with points pt 5 lw 4 title 'Pos Robo Inicial', '-' with points pt 2 lw 4 title 'Pos Robo Final'  \n";
	gp.send1d(ListaCordenadasObstaculos);
//	gp.send1d(TrajetoriaCalculadaInt);
	gp.send1d(TrajetoriaCalculadaDouble);
	gp.send1d(PontoInicialRoboParaImpressao);
	gp.send1d(PontoFinalRoboParaImpressao);

//vector< pair<int,int> > PontoInicialRoboParaImpressao;
//vector< pair<int,int> > PontoFinalRoboParaImpressao;



/*	gp << "plot '-' with points title 'ListaCordenadasObstaculos' \n";
	gp.send1d(ListaCordenadasObstaculos);
	gp << "plot '-' with lines title 'TrajetoriaCalculadaInt' \n";
	gp.send1d(TrajetoriaCalculadaInt);
	gp.flush();
*/
	//gp << "plot '-' with points title 'PosInicialRobo'\n";
	//gp.send1d(PosInicialRobo);

//TrajetoriaCalculadaInt
	//gp << "<echo '1 1' with points ls 1\n";
//" with << PosInicialRobo.first << " " << PosInicialRobo.second << "" << std::endl;;
	//gp.send1d(PosInicialRobo);

	//gp << "plot '-' with points title 'PosFinalRobo' \n";
	//gp.send1d(PosFinalRobo);





//pointtype 2 pointsize 8
	std::cout << "Press enter to exit." << std::endl;
	std::cin.get();
//pair<int,int> PosInicialRobo;
//pair<int,int> PosAtualRoboInt;
//pair<double,double> PosAtualRoboDouble;
//pair<int,int> PosFinalRobo;


//<< gridTeste[][] << " with points title ""Array A"" " <<std::endl;
/*	std::vector<std::pair<double, double> > xy_pts_A;
	for(double x=-2; x<2; x+=0.01) {
		double y = x*x*x;
		xy_pts_A.push_back(std::make_pair(x, y));
	}

	std::vector<std::pair<double, double> > xy_pts_B;
	for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
		double theta = alpha*2.0*3.14159;
		xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
	}


	// Data will be sent via a temporary file.  These are erased when you call
	// gp.clearTmpfiles() or when gp goes out of scope.  If you pass a filename
	// (e.g. "gp.file1d(pts, 'mydata.dat')"), then the named file will be created
	// and won't be deleted (this is useful when creating a script).
//	gp << "plot" << gp.file1d(xy_pts_A) << "with lines title 'cubic',"
//	<< gp.file1d(xy_pts_B) << "with points title 'circle'" << std::endl;
*/
}

void cliqueAlgumaTeclaPraContinuar(){
	std::cout << "\n\nPress enter to continue." << std::endl;
	std::cin.get();
}

