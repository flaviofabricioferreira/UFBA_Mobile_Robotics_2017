#Script Pra rodar comandos toda hora UFBA
#Abaixo Codigo pra rodar mais de um comando em outro terminal
#$ gnome-terminal -e "bash -c '<cmd1>;<cmd2>;exec $SHELL'"

clear
echo "------------------------------------"
echo "PASSO #0 = Compilando catkin_make.."
echo "------------------------------------"
catkin_make
echo "Waiting..."
sleep 10
echo "Completed"
echo "------------------------------------"
echo "PASSO #1 = Setando o Source.."
echo "------------------------------------"
source devel/setup.bash
echo "Waiting..."
sleep 5
echo "Completed"
echo "------------------------------------"
echo "PASSO #2 = Abrindo o Roscore em Outro Terminal"
echo "------------------------------------"
gnome-terminal -e "bash -c 'source devel/setup.bash;roscore;exec $SHELL'";
echo "Waiting..."
sleep 5
echo "Completed"
echo "------------------------------------"
echo "PASSO #3 = RODANDO Map_server a partir do mapa Salvo n pasta src/maps"
echo "------------------------------------"
gnome-terminal -e "bash -c 'rosrun map_server map_server src/maps/map.yaml;exec $SHELL'";
echo "Waiting..."
sleep 15
echo "Completed"
echo "------------------------------------"
echo "PASSO #4 = ABRINDO O Nosso Codigo fonte que recupera o mapa criado por Takaaki e imprime na tela"
echo "------------------------------------"
gnome-terminal -e "bash -c 'source devel/setup.bash;rosrun ufba1 mainUfba;exec $SHELL'";
echo "Waiting..."
sleep 30
echo "Completed"
echo "------------------------------------"
echo "PASSO #5 = SetaNDO o ponto inicial no mapa "
echo "------------------------------------"
echo "Não Implementado ainda!"
echo "------------------------------------"
echo "PASSO #6 = SetaNDO o ponto final"
echo "------------------------------------"
echo "Não Implementado ainda!"
echo "------------------------------------"
echo "PASSO #7 = CalculaNDO Rota a partir da APF Retornando Array com posições da rota de Xi a Xf"
echo "------------------------------------"
echo "Não Implementado ainda!"
echo "------------------------------------"
echo "PASSO #7 = CalculaNDO Controle Retornando Array com Velocidades angulares e lineares)"
echo "------------------------------------"
echo "Não Implementado ainda!"
echo "------------------------------------"
echo "Tudo Funcionou"
