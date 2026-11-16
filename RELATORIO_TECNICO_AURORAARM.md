# Operação Linha Viva: o desafio do braço colaborativo AuroraArm

## Relatório técnico — Model-Based Design para Sistemas Ciber-Físicos

**Projeto:** Junta do cotovelo do braço colaborativo AuroraArm  
**Linguagem da implementação:** Python 3  
**Escopo:** demonstração acadêmica de modelagem, controle, verificação e geração de código


---

## Resumo

Este trabalho apresenta um fluxo simplificado de Model-Based Design aplicado à junta do cotovelo do AuroraArm, um robô colaborativo de seis graus de liberdade. A planta é modelada a partir das equações elétrica e mecânica de um motor CC de ímã permanente, utilizando posição, velocidade angular e corrente de armadura como variáveis de estado. Em seguida, é implementado um controlador PID discreto com saturação de tensão e mecanismo anti-windup.

O modelo calculado possui posto 3 para as matrizes de controlabilidade e observabilidade. No ponto crítico de operação, a corrente exigida é de 3,735 A e a tensão de regime é de 41,238 V, dentro dos limites de 8 A e 48 V. A verificação exaustiva das combinações de atraso do sensor e jitter apresenta pior caso de 24 ms, inferior ao prazo de segurança de 100 ms. Em uma resposta de degrau simulada, o controlador obteve overshoot de 6,18% e tempo de acomodação aproximado de 70 ms. O controlador C é gerado automaticamente a partir do modelo Python e comparado em uma execução SIL.

---

# 1. Contextualização do problema

A Aurora Robótica está integrando o AuroraArm a uma linha de montagem compartilhada com operadores humanos. Não há cerca física separando completamente o robô das pessoas. Por esse motivo, a junta do cotovelo deve ter comportamento previsível, limites de atuação explícitos e um supervisor de segurança capaz de comandar uma parada quando for detectada a aproximação de um operador.

O protótipo anterior apresentou um quase-acidente porque o torque não foi zerado a tempo durante uma condição de borda entre movimento e parada segura. O problema central não foi apenas o controlador, mas a ausência de rastreabilidade e de evidência reprodutível de que o firmware atendia aos requisitos.

Este projeto demonstra uma cadeia simplificada:

```text
Requisitos → Modelo físico → Controlador → Supervisor → Testes → Código C → SIL → Rastreabilidade
```

---

# 2. Requisitos considerados

## 2.1 Requisitos de desempenho

| ID | Requisito adotado |
|---|---|
| REQ-CTRL-AA-001 | O erro de posição em regime permanente deve ser nulo para uma referência em degrau. |
| REQ-CTRL-AA-002 | O overshoot da resposta de posição não deve exceder 8%. |
| REQ-CTRL-AA-003 | O tempo de acomodação da resposta de posição não deve exceder 300 ms. |
| REQ-CTRL-AA-004 | O comando de tensão deve respeitar o limite do barramento e tratar saturação/anti-windup. |

## 2.2 Requisitos de segurança colaborativa

| ID | Requisito | Classificação |
|---|---|---|
| REQ-SAFE-AA-001 | Ao detectar a mão do operador a menos de 500 mm do efetuador, o torque deve ser zerado em até 100 ms. | Segurança |
| REQ-SAFE-AA-002 | Uma vez detectada a aproximação, o movimento não pode ser retomado automaticamente; é necessário reconhecimento explícito. | Segurança |
| REQ-SAFE-AA-003 | Depois de removida a condição de perigo e confirmado o reconhecimento, o braço deve poder retornar à trajetória original. | Vivacidade |
| REQ-SAFE-AA-004 | A velocidade linear do efetuador dentro da zona colaborativa não pode exceder 250 mm/s. | Segurança |

Uma propriedade de **segurança** expressa que algo ruim nunca deve acontecer: por exemplo, o movimento não deve continuar depois da detecção de uma aproximação perigosa. Uma propriedade de **vivacidade** expressa que algo bom eventualmente acontece: depois de uma parada segura e de um reconhecimento válido, deve ser possível retornar ao estado de movimento.

---

# 3. Parâmetros da junta

| Parâmetro | Símbolo | Valor |
|---|---:|---:|
| Resistência da armadura | R | 0,8 Ω |
| Indutância da armadura | L | 1,2 mH |
| Constante de torque | Kt | 0,085 N·m/A |
| Constante de f.c.e.m. | Ke | 0,085 V·s/rad |
| Inércia refletida | J | 4,2 × 10⁻⁴ kg·m² |
| Atrito viscoso refletido | b | 1,5 × 10⁻⁴ N·m·s/rad |
| Redução harmônica | N | 100:1 |
| Tensão máxima | Vmax | 48 V |
| Corrente máxima | Imax | 8 A |
| Período de amostragem | Ts | 2 ms / 500 Hz |
| Torque de carga refletido | τload | 0,25 N·m |
| Velocidade crítica do motor | ω | 450 rad/s |

As grandezas mecânicas fornecidas foram consideradas refletidas para o eixo do motor. A posição da junta é relacionada à posição do motor por `θjunta = θmotor/N`.

---

# 4. Modelagem física em espaço de estados

## 4.1 Equação elétrica

Para a armadura do motor CC:

`v = L di/dt + R i + Ke ω`

Portanto:

`di/dt = -(R/L)i -(Ke/L)ω +(1/L)v`

## 4.2 Equação mecânica

O balanço de torques é:

`J dω/dt = Kt i - bω - τload`

Logo:

`dω/dt = (Kt/J)i -(b/J)ω - τload/J`

A posição é obtida de:

`dθ/dt = ω`

## 4.3 Variáveis de estado

Foi escolhido:

`x = [θ, ω, i]ᵀ`

Essa escolha representa a posição que deve ser controlada, a velocidade que participa da dinâmica mecânica e a corrente que representa diretamente o torque eletromagnético.

## 4.4 Matrizes do modelo

Com `u=v` e saída de posição `y=θ`:

`A = [[0, 1, 0], [0, -b/J, Kt/J], [0, -Ke/L, -R/L]]`

`B = [[0], [0], [1/L]]`

`C = [[1, 0, 0]]`

`D = [[0]]`

Substituindo os parâmetros:

```text
A = [[ 0,          1,          0        ],
     [ 0,         -0,3571429, 202,3810 ],
     [ 0,        -70,8333,   -666,6667 ]]

B = [[0], [0], [833,3333]]
C = [[1, 0, 0]]
D = [[0]]
```

O código correspondente está em `auroraarm/plant.py`.

---

# 5. Constantes de tempo, autovalores e propriedades estruturais

## 5.1 Constantes de tempo

Constante elétrica:

`τe = L/R = 0,0012/0,8 = 0,0015 s = 1,5 ms`

Constante mecânica aproximada:

`τm = J/b = (4,2×10⁻⁴)/(1,5×10⁻⁴) = 2,8 s`

Existe uma separação significativa entre as escalas elétrica e mecânica. Isso indica que a dinâmica de corrente muda muito mais rapidamente que a dinâmica de posição.

## 5.2 Autovalores

Os autovalores calculados numericamente são:

```text
λ1 = 0
λ2 = -22,6152
λ3 = -644,4086
```

O autovalor zero está associado ao integrador da posição. Os demais representam os modos eletromecânicos estáveis do modelo linearizado.

## 5.3 Controlabilidade

Foi construída a matriz:

`Co = [B AB A²B]`

O posto calculado é:

`rank(Co) = 3`

Como o sistema possui três estados, o modelo é controlável para a entrada de tensão.

## 5.4 Observabilidade

Para a saída de posição, foi construída:

`Ob = [C; CA; CA²]`

O posto calculado é:

`rank(Ob) = 3`

Assim, os três estados são observáveis no modelo ideal a partir da saída escolhida.

---

# 6. Ponto de operação crítico

O ponto de operação exige `ω=450 rad/s` com torque de carga de `0,25 N·m`.

A corrente de regime é obtida pelo balanço mecânico:

`i = (τload + bω)/Kt`

Substituindo:

`i = (0,25 + 1,5×10⁻⁴ × 450)/0,085`

`i = 3,7353 A`

A tensão de regime é:

`v = Ri + Keω`

`v = 0,8×3,7353 + 0,085×450`

`v = 41,2382 V`

As folgas são:

```text
Folga de corrente = 8 - 3,7353 = 4,2647 A
Folga de tensão   = 48 - 41,2382 = 6,7618 V
```

A operação de regime está dentro dos limites, mas a folga de tensão é relativamente pequena. Durante transitórios, erro de posição ou perturbações, o comando pode saturar. Por isso foi implementado anti-windup.

---

# 7. Projeto do controlador

Foi implementado um PID discreto com período de 2 ms:

```text
Kp = 10,0
Ki = 0,5
Kd = 0,3
Kaw = 20,0
u_min = -48 V
u_max = +48 V
```

O algoritmo calcula:

`e[k] = r[k] - y[k]`

`d[k] = (e[k] - e[k-1])/Ts`

`u* [k] = Kp e[k] + I[k] + Kd d[k]`

O comando aplicado é limitado:

`u[k] = saturate(u*[k], -48, +48)`

O integrador utiliza back-calculation:

`I[k+1] = I[k] + Ts(Ki e[k] + Kaw(u[k] - u*[k]))`

Quando o comando satura, o termo de correção impede o crescimento indefinido do integrador. O código está em `auroraarm/controller.py`.

## 7.1 Resultado da resposta ao degrau

Foi simulada uma referência de 0,1 rad durante 2 s, com a planta discreta e sem torque de carga nessa avaliação de desempenho. Os resultados foram:

| Métrica | Resultado | Limite | Situação |
|---|---:|---:|---|
| Overshoot | 6,18% | 8% | Atende |
| Tempo de acomodação | 70 ms | 300 ms | Atende |
| Tensão máxima | 16 V | 48 V | Atende |

Esses resultados são válidos para o cenário simulado e não substituem ensaios físicos.

---

# 8. Escolha do período de amostragem

O laço de controle solicitado opera a 500 Hz:

`Ts = 2 ms`

Em relação à dinâmica mecânica:

`τm/Ts = 2,8/0,002 = 1400 amostras por constante de tempo`

Para a posição, o período é conservador e permite acompanhar adequadamente a dinâmica lenta. Entretanto, em relação à constante elétrica:

`τe/Ts = 1,5/2 = 0,75`

Isso significa que o período de posição não resolve completamente a dinâmica elétrica rápida. Em um produto real seria recomendável uma malha interna de corrente com frequência maior e uma malha externa de posição a 500 Hz.

---

# 9. Supervisor de segurança

O supervisor foi modelado com quatro estados:

```text
MOVING
   | proximidade detectada
   v
SAFE_STOP_PENDING
   | comando de parada aplicado
   v
SAFE_STOP
   | reconhecimento do operador
   v
ACK_REQUIRED
   | condição segura confirmada
   v
MOVING
```

O retorno ao movimento somente ocorre depois de uma sequência explícita de parada e reconhecimento. Assim, o evento de proximidade não pode simplesmente desaparecer e causar retomada automática.

## 9.1 Verificação do prazo de 100 ms

Foram combinados exaustivamente:

- atrasos do sensor: 0, 5, 10, 15 e 20 ms;
- jitter: 0 e 2 ms;
- aplicação do próximo comando: 2 ms.

O tempo total foi calculado como:

`Tresponse = Tsensor + Tjitter + Tcontrol`

O pior caso é:

`Tresponse,max = 20 + 2 + 2 = 24 ms`

Como:

`24 ms < 100 ms`

as 10 combinações verificadas satisfazem o requisito no modelo finito utilizado. A evidência está em `results/deadline_cases.csv` e a implementação em `auroraarm/safety.py`.

Essa análise não modela falhas de sensor, atrasos de comunicação, falhas de atuador, execução concorrente real, watchdog ou comportamento mecânico residual. Esses pontos devem ser tratados em uma análise de segurança real.

---

# 10. Geração automática de código e teste SIL

O arquivo `auroraarm/codegen.py` gera o arquivo:

`generated_c/controller_generated.c`

O cabeçalho do C contém rastreabilidade para os requisitos de controle, segurança e para o teste SIL. O C é compilado com `gcc` e recebe uma sequência de entradas igual à usada na referência Python.

O erro máximo absoluto medido na sequência foi aproximadamente:

`erro máximo = 2,4 × 10⁻⁶`

Esse valor é compatível com diferenças de arredondamento entre a aritmética Python e `float` em C usando a tolerância acadêmica de `3×10⁻⁶`. A comparação demonstra consistência numérica para a sequência testada, mas não é equivalência formal para todos os valores possíveis.

---

# 11. Testes e cobertura

A suíte cobre:

1. posto de controlabilidade;
2. posto de observabilidade;
3. limites de corrente e tensão no ponto de operação;
4. todas as combinações do atraso de segurança;
5. transições do supervisor;
6. retorno ao estado `MOVING` apenas após reconhecimento.

Estados cobertos:

```text
MOVING, SAFE_STOP_PENDING, SAFE_STOP, ACK_REQUIRED
```

Transições cobertas:

```text
MOVING → SAFE_STOP_PENDING
SAFE_STOP_PENDING → SAFE_STOP
SAFE_STOP → ACK_REQUIRED
ACK_REQUIRED → MOVING
```

Os testes estão em `tests/test_project.py`. A matriz requisito–modelo–código–teste está em `results/traceability.csv`.

---

# 12. Matriz de rastreabilidade

| Requisito | Modelo | Código | Evidência/teste |
|---|---|---|---|
| REQ-CTRL-AA-001 | Planta + PID | `controller.py` | resposta ao degrau |
| REQ-CTRL-AA-002 | Planta + PID | `controller.py` | cálculo de overshoot |
| REQ-CTRL-AA-003 | Planta + PID | `run_all.py` | cálculo do tempo de acomodação |
| REQ-CTRL-AA-004 | Saturação e anti-windup | `controller.py` | teste de limite de tensão |
| REQ-SAFE-AA-001 | Supervisor + atrasos | `safety.py` | `deadline_cases.csv` |
| REQ-SAFE-AA-002 | Autômato do supervisor | `safety.py` | teste de transições |
| REQ-SAFE-AA-003 | Transição de vivacidade | `safety.py` | retorno após reconhecimento |
| REQ-SAFE-AA-004 | Limite de velocidade | `plant.py` | teste de monitoramento |

---

# 13. Conclusão

O pacote desenvolvido demonstra uma cadeia coerente de Model-Based Design para a junta do AuroraArm. O modelo possui controlabilidade e observabilidade completas para a configuração escolhida. O ponto de operação crítico permanece dentro dos limites de corrente e tensão. O PID com saturação e anti-windup atende aos limites de desempenho no cenário simulado. A verificação finita do supervisor mostra pior resposta de 24 ms, inferior ao prazo de 100 ms, para os atrasos especificados.

A geração automática de C e o teste SIL fornecem uma evidência reprodutível de consistência entre o modelo e a implementação gerada. A matriz de rastreabilidade liga os requisitos aos arquivos e testes.

Apesar disso, o resultado ainda não sustenta sozinho a liberação de uma célula colaborativa real. Seriam necessárias validação experimental, testes HIL, análise de falhas, qualificação da ferramenta de geração, revisão independente, verificação de limites de força e velocidade no efetuador, avaliação de sensores e atuadores, e atendimento completo ao processo normativo aplicável.

---

# Referências

1. INTERNATIONAL ORGANIZATION FOR STANDARDIZATION. **ISO/TS 15066: Robots and robotic devices — Collaborative robots**. Geneva: ISO, 2016.
2. INTERNATIONAL ELECTROTECHNICAL COMMISSION. **IEC 61508: Functional safety of electrical/electronic/programmable electronic safety-related systems**. Geneva: IEC, 2010.
3. SICILIANO, Bruno; SCIAVICCO, Lorenzo; VILLANI, Luigi; ORIOLO, Giuseppe. **Robotics: Modelling, Planning and Control**. London: Springer, 2010.
4. BAIER, Christel; KATOEN, Joost-Pieter. **Principles of Model Checking**. Cambridge: MIT Press, 2008.
5. MODELICA ASSOCIATION. **Functional Mock-up Interface Specification, version 3.0**. 2022.
6. AURORA ROBÓTICA. **Projeto acadêmico AuroraArm: código, testes e evidências**. Repositório local deste trabalho, 2026.

---

## Anexo A — Arquivos do repositório

```text
auroraarm/plant.py
 auroraarm/controller.py
 auroraarm/safety.py
 auroraarm/codegen.py
 auroraarm/run_all.py
generated_c/controller_generated.c
tests/test_project.py
results/evidence.json
results/deadline_cases.csv
results/traceability.csv
```

## Anexo B — Como reproduzir

```bash
cd auroraarm_project
py -m pip install -r requirements.txt
py -m auroraarm.run_all
```
