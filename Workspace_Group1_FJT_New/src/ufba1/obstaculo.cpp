#include <iostream>
#include <vector>
#include <cmath>
#include <stdio.h>
#include "obstaculo.h"

using namespace std;



Obstaculo::Obstaculo(){
}

//Obstaculo::Obstaculo(vector<vector<int> > &lista){
//	this->listaDePontosDoObstaculo = lista;
//}

double Obstaculo::getMenordistancia(){return this->menordistancia; }
double Obstaculo::getXbiMenorDistancia(){return this->xBimenordistancia; }
double Obstaculo::getYbiMenorDistancia(){return this->yBimenordistancia;}
void Obstaculo::setMenordistancia(double value){this->menordistancia = value; }
void Obstaculo::setXbiMenorDistancia(double value){this->xBimenordistancia = value; }
void Obstaculo::setYbiMenorDistancia(double value){this->yBimenordistancia = value; }



vector<double> Obstaculo::calculaDistanciasRoboAoObstaculo(double xrobo,double yrobo,bool mostrar){

	setMenordistancia(100000);
	double distancia;
	vector<double> dadosPontoMenorDistancia;

	for(int i=0;i<this->listaDePontosDoObstaculo[0].size();i++){
			double xbi = this->listaDePontosDoObstaculo[0][i];
			double ybi = this->listaDePontosDoObstaculo[1][i];
			distancia = sqrt((xrobo-xbi)*(xrobo-xbi) + (yrobo-ybi)*(yrobo-ybi));
				if(mostrar){
					printf("Calculo Menor Distancia Obstaculo ao Robo\n");
					printf("SQRT[(%f - %f)^2 + (%f - %f)^2 ] = %f \n",xrobo,xbi,yrobo,ybi,distancia);
				}

				if(distancia < this->menordistancia){

					setMenordistancia(distancia);
					dadosPontoMenorDistancia.push_back(distancia);
					//printf("\n Setou a Menor Distancia Corretamente como %f",getMenordistancia());
					//printf("\n Setou a This->Menor Distancia Corretamente como %f",this->menordistancia);
					setXbiMenorDistancia(xbi);
					dadosPontoMenorDistancia.push_back(xbi);
					//printf("\n Setou o X da Menor Distancia Corretamente como %d",getXbiMenorDistancia());
					setYbiMenorDistancia(ybi);
					dadosPontoMenorDistancia.push_back(ybi);
					//printf("\n Setou o Y da Menor Distancia Corretamente como %d",getYbiMenorDistancia());
				}				
		}
//	printf("\nLista de informacao : {");	
//	for(int j=0;j<dadosPontoMenorDistancia.size();j++){
//		printf("%f,",dadosPontoMenorDistancia[j]);			
//	}
//	printf("}\n");	
	return dadosPontoMenorDistancia;
}

vector<vector<double> > Obstaculo::getListadePontosDoObstaculo(){
	return this->listaDePontosDoObstaculo;
}

void Obstaculo::setListadePontosDoObstaculo(std::vector<std::vector<double> > &list){
	this->listaDePontosDoObstaculo = list;
}

void Obstaculo::showListadePontos(){

		printf("\nX = {");
		for(int j=0;j<this->listaDePontosDoObstaculo[0].size();j++){
			printf("%f,",this->listaDePontosDoObstaculo[0][j]);
		}
		printf("}\n");

		printf("\nY = {");
		for(int j=0;j<this->listaDePontosDoObstaculo[1].size();j++){
			printf("%f,",this->listaDePontosDoObstaculo[1][j]);
		}
		printf("}\n");
}
