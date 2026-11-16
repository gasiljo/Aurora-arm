# AuroraArm — pacote didático de Model-Based Design

Projeto simples em Python para a junta do cotovelo do AuroraArm. O pacote demonstra:

- modelo eletromecânico em espaço de estados;
- simulação discreta da planta;
- controlador PID de posição com saturação de tensão/corrente e anti-windup;
- verificação exaustiva do prazo de parada de segurança sob atraso do sensor e jitter;
- geração automática de um controlador C a partir do modelo Python;
- teste SIL de equivalência numérica Python × C;
- testes de transições do supervisor e matriz de rastreabilidade.

> **Escopo:** exemplo acadêmico. Os números e evidências não substituem validação independente, HIL, análise de perigos, qualificação de ferramenta ou certificação segundo ISO/TS 15066, IEC 61508 ou norma equivalente.

## Execução

```bash
py -m pip install -r requirements.txt
py -m auroraarm.run_all
```

Os resultados são gravados em `results/`. O código C gerado fica em `generated_c/controller_generated.c`.

## Estrutura

- `auroraarm/plant.py`: parâmetros e matrizes A, B, C, D.
- `auroraarm/controller.py`: PID discreto, saturação e anti-windup.
- `auroraarm/safety.py`: supervisor e verificação por varredura de atrasos.
- `auroraarm/codegen.py`: geração automática do C.
- `auroraarm/sil.py`: compilação e comparação Python/C.
- `auroraarm/run_all.py`: execução reprodutível de todas as evidências.
- `tests/`: testes automatizados.
- `results/`: memorial numérico, cobertura e rastreabilidade.

### Windows sem GCC

O comando `py -m auroraarm.run_all` funciona mesmo sem GCC ou Clang instalados. O arquivo C continua sendo gerado em `generated_c/controller_generated.c`; para o SIL, o script usa automaticamente uma implementação Python com aritmética `float32` quando não encontra um compilador C. Para executar o SIL nativo, instale MinGW-w64/GCC ou LLVM/Clang e deixe o executável disponível no `PATH`.
