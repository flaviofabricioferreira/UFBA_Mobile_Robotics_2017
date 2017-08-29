//Codigo Trabalho UFBA Robotica Movel 20171
//Flavio Fabricio
//Jessica Motta
//Takaaki Oda Junior

//This is a ROS version of the standard Helloworld program.

#include <ros/ros.h> //This header defines the stantard ROS classes
#include <nav_msgs/GetMap.h>
#include <vector>


using namespace std; // uso de strings


//Declaração das variaveis que serão utilizadas
int rows;
int cols;
double mapResolution;
vector<vector<bool> > grid; // array de arrays de booleanos , ou seja, um array de duas dimensoes pra guardar o mapa


//Declaração das funções que lidarão com o mapa
bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printGrid();


int main(int argc, char** argv){

//Initialize the ROS System
ros::init(argc, argv, "UFBA_ROS_20171_FJT");
//Establish this program as a ROS node.
ros::NodeHandle nh;

// Faz o request do Mapa a partir do Serviço
if(!requestMap(nh)) // Se retornsr falso por algum problema, então sai do programa
exit(-1);

// Imprime na Tela o Mapa
printGrid();

//Send some output as a log message
ROS_INFO_STREAM("MAPA GERADO");


//Fecha o Programa
return 0;

}


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

	grid.resize(rows);
	for(int i=0;i<rows;i++){
		grid[i].resize(cols);	
	}

	int currCell = 0;

	for(int i=0;i<rows;i++){
		for(int j=0;j<cols;j++){
			if(map.data[currCell]==0) // Celula Livre
				grid[i][j] = false;
			else // Celula Ocupada = 100 ou indefinido (-1)
				grid[i][j] = true;
			currCell++;
		}
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
			printf("%d ",grid[i][j] ? 1 : 0); // Imprime 1 pra true(ocupado ou indefinido) ou 0 para false(livre)
		}
		printf("\n");
	}
}



