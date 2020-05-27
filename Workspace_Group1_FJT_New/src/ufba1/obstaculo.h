class Obstaculo{

	private:
		//vector<vector<int> > listaDePontosDoObstaculo;
		std::vector<std::vector<double> > listaDePontosDoObstaculo;
		double menordistancia;
		double xBimenordistancia;
		double yBimenordistancia;
	public:
		Obstaculo();//		Obstaculo(vector<vector<int> >);
		//Obstaculo(vector<vector<int> > &l);
//		Obstaculo(int**);
		std::vector<double> calculaDistanciasRoboAoObstaculo(double xrobo,double yrobo,bool mostrar);		       
//           	vector<vector<int> > getListadePontosDoObstaculo();
         	std::vector<std::vector<double> > getListadePontosDoObstaculo();
		void setListadePontosDoObstaculo(std::vector<std::vector<double> > &list);
//		void setListadePontosDoObstaculo(int**);
		void showListadePontos();

		double getMenordistancia();
		double getXbiMenorDistancia();
		double getYbiMenorDistancia();
		void setMenordistancia(double value);
		void setXbiMenorDistancia(double value);
		void setYbiMenorDistancia(double value);



};


