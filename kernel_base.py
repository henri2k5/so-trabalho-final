"""
================================================================================
                        Trabalho Final de Sistemas Operacionais
                                  KERNEL BASE
================================================================================

                                    ATENÇÃO
    Este arquivo define a arquitetura central do Sistema Operacional Simulado, incluindo
    o hardware (CPU, RAM, Disco) e o núcleo do Kernel.

    Cada equipe deve focar em implementar a sua função designada DENTRO desta
    classe Kernel, nos locais indicados com "A EQUIPE X DEVE IMPLEMENTAR...".
"""

import os
import time
from enum import Enum
from collections import deque
from dataclasses import dataclass, field

# ================================================================================
# 1. CONSTANTES DE CONFIGURAÇÃO DO HARDWARE E DO KERNEL
# ================================================================================

TAMANHO_RAM_BYTES = 4096            # 4 KB de memória RAM
TAMANHO_BLOCO_DISCO_BYTES = 128     # Cada bloco no disco terá 128 bytes
NUM_BLOCOS_DISCO = 256              # Nosso disco terá 256 blocos (total 32 KB)
NOME_ARQUIVO_DISCO = "disco.bin"    # Arquivo que simulará nosso disco
QUANTUM_RR = 4                      # Número de instruções por fatia de tempo para o Round-Robin

# ================================================================================
# 2. DEFINIÇÃO DAS ESTRUTURAS DE DADOS CENTRAIS (PCB, TCB, ESTADOS)
# ================================================================================

class EstadoProcesso(Enum):
    """ Enumeração para os estados de um processo. """
    NOVO = "NOVO"
    PRONTO = "PRONTO"
    EXECUCAO = "EXECUÇÃO"
    BLOQUEADO = "BLOQUEADO"
    TERMINADO = "TERMINADO"

@dataclass
class TCB:
    """ Thread Control Block (TCB): Representa uma thread. """
    tid: int
    pid_pai: int
    estado: EstadoProcesso
    contador_de_programa: int = 0
    registradores: dict = field(default_factory=dict)

@dataclass
class PCB:
    """ Process Control Block (PCB): Representa um processo. """
    pid: int
    nome_programa: str
    estado: EstadoProcesso
    prioridade: int = 1
    contador_de_programa: int = 0
    registradores: dict = field(default_factory=dict)
    # Cada processo tem sua própria lista de threads
    threads: list[TCB] = field(default_factory=list)
    # Informações de memória
    endereco_base_memoria: int = -1
    tamanho_memoria: int = 0
    tabela_de_paginas: dict = field(default_factory=dict)


# ================================================================================
# 3. CLASSES DE SIMULAÇÃO DO HARDWARE
# ================================================================================

class RAM:
    """ Simula a Memória RAM como um array de bytes. """
    def __init__(self, tamanho):
        self.tamanho = tamanho
        self.memoria = bytearray(tamanho)
        print(f"[Hardware] RAM de {tamanho} bytes inicializada.")

    def ler(self, endereco, quantidade):
        if endereco + quantidade > self.tamanho:
            raise MemoryError(f"Acesso inválido na RAM no endereço {endereco}")
        return self.memoria[endereco : endereco + quantidade]

    def escrever(self, endereco, dados):
        if endereco + len(dados) > self.tamanho:
            raise MemoryError(f"Escrita inválida na RAM no endereço {endereco}")
        self.memoria[endereco : endereco + len(dados)] = dados

class Disco:
    """ Simula o Disco Rígido, usando um arquivo local como backing store. """
    def __init__(self, caminho, num_blocos, tamanho_bloco):
        self.caminho_arquivo = caminho
        self.num_blocos = num_blocos
        self.tamanho_bloco = tamanho_bloco
        tamanho_total = num_blocos * tamanho_bloco
        
        if not os.path.exists(caminho):
            with open(caminho, "wb") as f:
                f.write(bytearray(tamanho_total))
        self.arquivo = open(caminho, "rb+")
        print(f"[Hardware] Disco de {tamanho_total / 1024:.2f} KB inicializado em '{caminho}'.")

    def ler_bloco(self, numero_bloco):
        if not 0 <= numero_bloco < self.num_blocos:
            raise IOError(f"Tentativa de ler bloco inválido: {numero_bloco}")
        posicao = numero_bloco * self.tamanho_bloco
        self.arquivo.seek(posicao)
        return self.arquivo.read(self.tamanho_bloco)

    def escrever_bloco(self, numero_bloco, dados):
        if not 0 <= numero_bloco < self.num_blocos:
            raise IOError(f"Tentativa de escrever em bloco inválido: {numero_bloco}")
        if len(dados) > self.tamanho_bloco:
            raise ValueError("Dados maiores que o tamanho do bloco.")
        # Garante que os dados tenham sempre o tamanho do bloco (preenche com zeros)
        dados_bloco = dados.ljust(self.tamanho_bloco, b'\0')
        posicao = numero_bloco * self.tamanho_bloco
        self.arquivo.seek(posicao)
        self.arquivo.write(dados_bloco)
        self.arquivo.flush()

    def __del__(self):
        self.arquivo.close()

class CPU:
    """ Simula a Unidade Central de Processamento. """
    def __init__(self):
        self.pc = 0
        self.registradores = {'R1': 0, 'R2': 0, 'R3': 0}
        self.processo_atual = None
        print("[Hardware] CPU inicializada.")

    def executar_instrucao(self):
        if self.processo_atual:
            # Simula a execução de uma instrução avançando o Program Counter
            self.pc += 1
            print(f"[CPU] Instrução executada para o PID {self.processo_atual.pid}. PC atual: {self.pc}")
        else:
            print("[CPU] Ociosa.")
            
    def carregar_contexto(self, pcb):
        self.processo_atual = pcb
        self.pc = pcb.contador_de_programa
        self.registradores = pcb.registradores.copy()
        
    def salvar_contexto(self):
        if self.processo_atual:
            self.processo_atual.contador_de_programa = self.pc
            self.processo_atual.registradores = self.registradores.copy()

# ================================================================================
# 4. CLASSE PRINCIPAL DO KERNEL (AQUI ENTRAM AS IMPLEMENTAÇÕES DAS EQUIPES)
# ================================================================================

class Kernel:
    def __init__(self):
        # Inicializa o Hardware
        self.ram = RAM(TAMANHO_RAM_BYTES)
        self.disco = Disco(NOME_ARQUIVO_DISCO, NUM_BLOCOS_DISCO, TAMANHO_BLOCO_DISCO_BYTES)
        self.cpu = CPU()

        # Estruturas de Dados Centrais do Kernel
        self.tabela_de_processos = {}
        self.fila_de_prontos = deque()
        self.fila_de_bloqueados = {}
        self.quantum_restante = QUANTUM_RR
        self.proximo_pid = 1 # PID 0 pode ser reservado para o 'init' ou 'idle'

        # Módulos do SO (serão as funções implementadas pelas equipes)
        self.rodando = False
        print("[Kernel] Núcleo do SO inicializado.")
    
    # --- Funções do Núcleo ---
    def bootstrap(self):
        """ Prepara o SO para execução, criando o primeiro processo 'init'. """
        print("[Kernel] Sistema operacional inicializando (bootstrap)...")
        # Exemplo: cria um processo inicial para que o sistema não comece vazio
        self.sys_create_process("init")
        self.rodando = True
        print("[Kernel] Bootstrap concluído.")

    def loop_principal(self):
        """ O ciclo principal de execução do Sistema Operacional. """
        print("\n[Kernel] Iniciando loop principal de execução...")
        while self.rodando:
            # Pega o processo atual da CPU para verificar seu estado
            processo_saindo = self.cpu.processo_atual
            
            # Se o processo que estava executando terminou ou bloqueou, o escalonador não precisa
            # colocá-lo de volta na fila de prontos.
            colocar_de_volta_na_fila = processo_saindo is not None and processo_saindo.estado == EstadoProcesso.EXECUCAO

            # Chama o escalonador para decidir quem será o próximo
            proximo_processo = self.schedule_rr(processo_saindo, colocar_de_volta_na_fila)
            
            if proximo_processo:
                # Salva o contexto do processo que estava saindo (se houver um)
                if processo_saindo:
                    self.cpu.salvar_contexto()
                
                # Carrega o contexto do novo processo
                self.cpu.carregar_contexto(proximo_processo)
                proximo_processo.estado = EstadoProcesso.EXECUCAO
                
                # Reinicia o quantum para o novo processo
                self.quantum_restante = QUANTUM_RR
            
            elif processo_saindo and not proximo_processo:
                # Caso onde o último processo terminou/bloqueou e não há mais ninguém pronto
                self.cpu.salvar_contexto()
                self.cpu.processo_atual = None # Garante que a CPU fique ociosa
                
            # Executa uma instrução na CPU
            self.cpu.executar_instrucao()
            self.quantum_restante -= 1

            time.sleep(0.5) # Atraso para podermos observar a simulação

    # ============================================================================
    # ÁREA DE IMPLEMENTAÇÃO DAS EQUIPES
    # ============================================================================

    # --- Equipe 1: Criação e Encerramento de Processos ---
    def sys_create_process(self, nome_programa):
        """
        Responsável por criar um novo processo.
        - Deve gerar um PID único.
        - Criar e inicializar um PCB.
        - Alocar memória para o processo (chamar a função da Equipe 6).
        - Mudar o estado para PRONTO e inserir na fila de prontos.
        - Retorna o PID do novo processo ou -1 em caso de erro.
        """
        # 
        # A EQUIPE 1 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        
        print(f"[Kernel] (Equipe 1) Criando processo '{nome_programa}'...")
        
        # Lógica de placeholder para o bootstrap funcionar:
        pid = self.proximo_pid
        self.proximo_pid += 1
        
        # Futuramente, deve chamar self.sys_malloc() da Equipe 6
        # endereco_memoria = self.sys_malloc(1024) # Ex: 1KB por processo
        # if endereco_memoria == -1:
        #    print(f"[Kernel] (Equipe 1) Falha ao alocar memória para o processo.")
        #    return -1
            
        pcb = PCB(pid=pid, nome_programa=nome_programa, estado=EstadoProcesso.PRONTO)
        # pcb.endereco_base_memoria = endereco_memoria
        
        self.tabela_de_processos[pid] = pcb
        self.fila_de_prontos.append(pcb)
        
        print(f"[Kernel] (Equipe 1) Processo {pid} criado e pronto.")
        return pid
        
    def sys_terminate_process(self, pid):
        """
        Responsável por encerrar um processo.
        - Deve mudar o estado para TERMINADO.
        - Liberar a memória do processo (chamar a função da Equipe 6).
        - Remover o PCB de todas as estruturas do sistema.
        - Retorna True em caso de sucesso, False caso contrário.
        """
        # 
        # A EQUIPE 1 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 1) AINDA NÃO IMPLEMENTADO: Encerrar processo {pid}.")
        pass

    # --- Equipe 2: Comunicação (Memória Compartilhada) ---
    def sys_shm_create(self, tamanho):
        """
        Cria uma nova região de memória compartilhada.
        - Deve alocar um bloco na RAM (pode usar a função da Equipe 6).
        - Deve retornar uma chave/ID único para esta região.
        """
        # 
        # A EQUIPE 2 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 2) AINDA NÃO IMPLEMENTADO: Criar memória compartilhada de {tamanho} bytes.")
        pass

    # --- Equipe 3: Comunicação (Troca de Mensagens) ---
    def sys_msg_send(self, dest_pid, mensagem):
        """
        Envia uma mensagem para outro processo.
        - Deve localizar o buffer de mensagens do destinatário.
        - Adicionar a mensagem.
        - Se o destinatário estava bloqueado esperando, deve desbloqueá-lo.
        """
        # 
        # A EQUIPE 3 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 3) AINDA NÃO IMPLEMENTADO: Enviar mensagem para {dest_pid}.")
        pass
    
    def sys_msg_receive(self, pid):
        """
        Recebe uma mensagem.
        - Se houver mensagem, retorna-a.
        - Se não, bloqueia o processo (muda estado, remove da fila de prontos).
        """
        # 
        # A EQUIPE 3 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 3) AINDA NÃO IMPLEMENTADO: Receber mensagem para {pid}.")
        pass

        # --- Equipe 4: Criação e Encerramento de Threads ---
    def sys_create_thread(self, pid, funcao_inicio):
        """
        Cria uma nova thread dentro de um processo existente.
        - Deve gerar um TID único para o processo.
        - Criar e inicializar um TCB.
        - Adicionar o TCB à lista de threads do PCB e à fila de prontos do escalonador.
        """
        # 
        # A EQUIPE 4 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 4) AINDA NÃO IMPLEMENTADO: Criar thread para o processo {pid}.")
        pass
        if pid not in self.tabela_de_processos:
            print(f"[Kernel] Erro: Processo {pid} nao encontrado")
            return -1
        
        processo_pai = self.tabela_de_processos[pid]

        novo_tid = len(processo_pai.threads)

        nova_thread = TCB(tid=novo_tid,
                          pid_pai=pid,
                          estado=EstadoProcesso.PRONTO,
                          contador_de_programa=funcao_inicio
                          )
        processo_pai.threads.append(nova_thread)


    # --- Equipe 5: Escalonamento de Processos (Round-Robin) ---
    def schedule_rr(self, processo_saindo, colocar_de_volta_na_fila):
        """
        Implementa o algoritmo de escalonamento Round-Robin.
        - É chamado pelo loop principal.
        - Verifica se o quantum acabou ou se o processo bloqueou/terminou.
        - Se sim, deve escolher o próximo da fila de prontos.
        - Retorna o PCB do próximo processo a ser executado.
        """
        # 
        # A EQUIPE 5 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        
        # Lógica de placeholder para o sistema rodar:
        if self.quantum_restante <= 0 or (processo_saindo and processo_saindo.estado != EstadoProcesso.EXECUCAO):
            # O quantum acabou ou o processo não está mais em execução (bloqueou/terminou)
            
            # Se o processo que estava saindo ainda deve voltar para a fila
            if colocar_de_volta_na_fila:
                print(f"[Kernel] (Equipe 5) Quantum expirou para PID {processo_saindo.pid}. Voltando para a fila.")
                processo_saindo.estado = EstadoProcesso.PRONTO
                self.fila_de_prontos.append(processo_saindo)
            
            # Tenta pegar o próximo processo da fila
            if self.fila_de_prontos:
                proximo = self.fila_de_prontos.popleft()
                print(f"[Kernel] (Equipe 5) Escalonador: Trocando para PID {proximo.pid}")
                return proximo
            else:
                # Ninguém na fila, CPU fica ociosa
                print(f"[Kernel] (Equipe 5) Escalonador: Fila de prontos vazia. CPU ociosa.")
                return None 
        
        # Se não for nenhuma das condições acima, o processo atual continua executando
        return processo_saindo 
        
    # --- Equipe 6: Alocação e Liberação de Memória Física ---
    def sys_malloc(self, tamanho):
        """
        Aloca um bloco de memória contígua na RAM.
        - Deve implementar um algoritmo como First-Fit ou Best-Fit.
        - Gerenciar uma lista/mapa de blocos livres.
        - Retorna o endereço base do bloco alocado ou -1 se não houver espaço.
        """
        # 
        # A EQUIPE 6 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 6) AINDA NÃO IMPLEMENTADO: Alocar {tamanho} bytes.")
        return -1 # Retorna -1 para indicar falha
    
    def sys_free(self, endereco):
        """
        Libera um bloco de memória.
        - Deve marcar o bloco como livre e tentar fundi-lo com vizinhos livres.
        """
        # 
        # A EQUIPE 6 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 6) AINDA NÃO IMPLEMENTADO: Liberar memória no endereço {endereco}.")
        pass
    
    # --- Equipe 7: Gerenciamento de Memória Virtual ---
    def vm_translate_address(self, pid, endereco_logico):
        # Define o tamanho da pagina
        tamanho_pagina = TAMANHO_BLOCO_DISCO_BYTES

        processo = self.tabela_de_processos.get(pid)
        if processo is None:
            print(f"[Kernel] (Equipe 7) Erro: Processo {pid} não encontrado para tradução de endereço.")
            return -1
        
        numero_pagina = endereco_logico // tamanho_pagina
        deslocamento = endereco_logico % tamanho_pagina

        frame = processo.tabela_de_paginas.get(numero_pagina)

        if frame is None:
            print(f"[Kernel] (Equipe 7) Falha na tradução: Página {numero_pagina} não mapeada para o processo {pid}.")
            
            novo_frame = self.sys_malloc(tamanho_pagina)
            if novo_frame == -1:
                print(f"[Kernel] (Equipe 7) Falha ao alocar memória para página {numero_pagina} do processo {pid}.")
                return -1
            
            processo.tabela_de_paginas[numero_pagina] = novo_frame
            frame = novo_frame

        endereco_fisico = (frame * tamanho_pagina) + deslocamento
        print(f"[Kernel] (Equipe 7) Tradução: Lógico {endereco_logico} (P={numero_pagina}, D={deslocamento}) -> Físico {endereco_fisico} (Frame {frame}).")
        return endereco_fisico

    # --- Equipe 8: Gerenciamento de Arquivos ---
    def sys_create_file(self, nome):
        """ Cria um arquivo vazio no disco. """
        # 
        # A EQUIPE 8 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 8) AINDA NÃO IMPLEMENTADO: Criar arquivo {nome}.")
        pass
    
    def sys_write_file(self, nome, dados):
        """ Escreve dados em um arquivo. """
        # 
        # A EQUIPE 8 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 8) AINDA NÃO IMPLEMENTADO: Escrever no arquivo {nome}.")
        pass

    def sys_read_file(self, nome):
        """ Lê dados de um arquivo. """
        # 
        # A EQUIPE 8 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 8) AINDA NÃO IMPLEMENTADO: Ler o arquivo {nome}.")
        pass
        
    def sys_delete_file(self, nome):
        """ Exclui um arquivo do disco. """
        # 
        # A EQUIPE 8 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 8) AINDA NÃO IMPLEMENTADO: Deletar o arquivo {nome}.")
        pass

    # --- Equipe 9: Interpretador de Comandos ---
    def shell_parse_and_execute(self, comando_str):
        """
        Interpreta um comando do usuário e chama a função de sistema correspondente.
        - Deve fazer o parsing da string de comando.
        - Chamar a função sys_* apropriada deste Kernel.
        - Retorna o resultado da operação para o usuário.
        """
        # 
        # A EQUIPE 9 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print(f"[Kernel] (Equipe 9) AINDA NÃO IMPLEMENTADO: Interpretar comando '{comando_str}'.")
        pass
        
    # --- Equipe 10: Listagem de Processos (htop) ---
    def sys_htop(self):
        """
        Gera uma string formatada com a lista de todos os processos e seus estados.
        - Deve varrer a tabela de processos.
        - Para cada processo, coletar PID, nome, estado, etc.
        - Formatar tudo em uma única string legível, como uma tabela.
        - Retorna a string. Não deve usar print().
        """
        # 
        # A EQUIPE 10 DEVE IMPLEMENTAR ESTA FUNÇÃO
        # 
        print("[Kernel] (Equipe 10) Gerando listagem de processos.")
        pass

# ================================================================================
# 5. PONTO DE ENTRADA PRINCIPAL DA SIMULAÇÃO
# ================================================================================

if __name__ == "__main__":
    # Cria o Kernel, que por sua vez inicializa todo o hardware
    kernel_so = Kernel()
    
    # Inicia o sistema operacional
    kernel_so.bootstrap()
    
    # Inicia o loop principal de execução do SO
    try:
        kernel_so.loop_principal()
    except KeyboardInterrupt:
        print("\n[Kernel] Simulação interrompida pelo usuário (Ctrl+C).")
