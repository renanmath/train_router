# Projeto de roteamento de trens baseado em demandas.
Esse projeto foi implementando utilizando o framework Pyomo.

## Dados de entrada
Como dados de entrada, temos:
* Uma coleção de trens
* Uma coleção de termais
* Uma coleção de produtos
* Demandas de cada terminal, para cada produto
* Estoque inicial de cada terminal, para cada produto
* Matriz de tempos de viagens entre os terminais
* Número de vagões de cada trem, e peso médio de cada vagão

## Objetivo
O modelo base simula um horizonte de tempo curto, no qual
devemos decidir para onde enviar os primeiros trens a finalizarem a
operação de descarregamento.

O modelo não simula uma operação longa; seu objetivo é decidir qual
a melhor maneira de rotear os próximos trens, baseado no estado
atual.
Obteremos assim um próximo estado, o qual pode ser utilizado como
dado de entrada para o modelo, que poderá simular um estado
seguinte, e assim por diante.

## Premissas
Algumas premissas do modelo base:
* Podem haver trens em rota entre dois terminais, mas não
tomaremos decisão para esses trens.
* Pode haver trens em operação de descarregamento ou
esperando em fila no terminal. São para esses trens que
tomaremos decisão.
* Não pode haver trens em operação de carregamento nos
terminais.
* Por simplificação, vamos considerar que o tempo de operação
de carregamento ou descarregamento em cada terminal é fixo,
não importando o modelo do trem.