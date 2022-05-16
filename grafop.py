import time

class Grafo:
  def __init__(self,num_vert=0,num_arestas=0,lista_adj=None,mat_adj=None,tipo=0):
    self.num_vert=num_vert
    self.num_arestas=num_arestas
    self.tipo=tipo
    if lista_adj == None:
      self.lista_adj = [[]for i in range(num_vert)]
    else : 
      self.lista_adj = lista_adj
    if mat_adj == None:
      self.mat_adj = [[0 for i in range(num_vert)]for j in range(num_vert)]
    else:
      self.mat_adj = mat_adj
      

  def add_aresta (self,u,v,w=1):
    if u < self.num_vert and v < self.num_vert:
      self.num_arestas +=1 
      self.lista_adj[u].append((v,w))
      self.mat_adj[u][v] = w
    else:
      print("Aresta Inválida")

  def add_aresta_t (self,u,v,w=1):
    if u < self.num_vert and v < self.num_vert:
      self.num_arestas +=1
      self.lista_adj[u].append((v,w))
    else:
      print("Aresta Inválida")

  def ler_arquivo(self,nome_arq):
    try: 
      arq = open(nome_arq)
      str = arq.readline()
      str = str.split(" ")
      self.num_vert = int(str[0])
      aux = int(str[1])
      self.lista_adj = [[]for i in range(self.num_vert)]
      self.mat_adj = [[0 for i in range(self.num_vert)]for j in range(self.num_vert)]
      '''le cada aresta do arquivo'''
      for i in range(aux):
        str = arq.readline()
        str = str.split(" ")
        u = int(str[0])
        v = int(str[1])
        w = int(str[2])
        self.add_aresta(u,v,w)
        if(w>1 and self.tipo == 0):
          self.tipo = 1
        elif(w<0):
          self.tipo = 2
    except IOError:
      print("Não foi possivel ler o arquivo!")

  def ler_arquivo_t(self,nome_arq):
    try:
      arq = open(nome_arq)
      str = arq.readline()
      str = str.split(" ")
      self.num_vert = int(str[0])
      aux = int(str[1])
      self.lista_adj = [[]for i in range(self.num_vert)]
      '''edição para o trabalho'''
      for i in range(aux):
        str = arq.readline()
        str = str.split(" ")
        u = int(str[0])
        v = int(str[1])
        w = int(str[2])
        self.add_aresta_t(u,v,w)
        if(w>1 and self.tipo == 0):
          self.tipo = 1
        elif(w<0):
          self.tipo = 2
    except IOError:
      print("Não foi possivel ler o arquivo!")

  def densidade(self):
    '''Calula a densidade de um grafo'''
    dens = self.num_arestas/((self.num_vert)*(self.num_vert - 1))
    return dens

  def subgrafo (self,g2)->bool:
    '''determina se g2 é subgrafo de self'''
    if g2.num_vert > self.num_vert or g2.num_arestas > self.num_arestas:
      '''verificar se g2 tem mais vértices que self'''
      return False
    for i in range(len(g2.mat_adj)):
      for j in range(len(g2.mat_adj[i])):
        if g2.mat_adj[i][j] != 0 and self.mat_adj[i][j] == 0:
          return False
    return True

  def busca_largura(self,s):
    desc = [0 for i in range(self.num_vert)]
    q = [s]
    r = [s]
    desc[s] = 1
    while q:
      u = q.pop(0)
      for (v, w) in self.lista_adj[u]:
        if desc[v] == 0:
          q.append(v)
          r.append(v)
          desc[v] = 1
    return r

  def busca_profundidade(self,s):
    desc = [0 for i in range(self.num_vert)]
    S = [s]
    R = [s]
    desc[s] = 1
    while S:
      desempilhar = True
      u = S[len(S)-1]
      for (v,w) in self.lista_adj[u]:
        if  desc[v] == 0:
          S.append(v)
          R.append(v)
          desc[v] = 1
          desempilhar = False
          break
      if desempilhar:
        S.pop()
    return R

  def conexo(self,s):
    r = self.busca_profundidade(s)
    if len(r) == self.num_vert:
      print("Grafo conexo")
    else:
      print("Grafo desconexo")

  def ciclo(self,s):
    desc = [0 for i in range(self.num_vert)]
    S = [s]
    desc[s] = 1
    while S:
      desempilhar = True
      u = S[len(S)-1]
      for (v,w) in self.lista_adj[u]:
        if  desc[v] == 0:
          S.append(v)
          desc[v] = 1
          desempilhar = False
          break
        if  desc[v] == 1:
          print("Ciclo Encontrado")
          return 0
      if desempilhar:
        S.pop()
      if len(S) > 0:
        for elem in desc:
          print(elem)
          if elem == 0:
            S.append(elem)
    print("Nenhum ciclo encontrado, encerrando a função!!!")


  def print_dijkstra(pred,s,d):
    if len(pred) != 0:
      i = d
      caminho = [i]
      while i != s:
        caminho.insert(0,pred[i])
        i = pred[i]
      print("Caminho:", caminho)
    else:
      print("Não foi encontrado caminho do vértice origem ao vértice destino!!")
    
  def dijkstra(self,s):
    custo = 0
    dist = [float("inf") for i in range(self.num_vert)]
    pred = [None for i in range(self.num_vert)]
    dist[s] = 0
    Q = []
    for i in range(len(self.lista_adj)):
      Q.append(i)
    while Q:
      if Q[0] != s:
        min = float("inf")
        for elem in Q:
          if dist[elem] < min:
            min = dist[elem]
            r = elem
      else:
        r = Q[0]
      u = r
      Q.remove(u)
      for (v,w) in self.lista_adj[u]:
        custo = custo + 1
        if dist[v] > dist[u]+w:
          dist[v] = dist[u]+ w
          pred[v] = u
    return dist,pred,custo

  def bellman_ford(self,s):
    custo = 0
    dist = [float("inf") for i in range(self.num_vert)]
    pred = [None for i in range(self.num_vert)]
    dist[s] = 0
    for u in range(len(self.lista_adj)):
      trocou = True
      for (v,w) in self.lista_adj[u]:
        custo = custo + 1
        if dist[v] > dist [u] + w:
          dist[v] = dist[u] + w
          pred[v] = u
          trocou = False
      if trocou: 
        return dist,pred,custo

  def floyd_warshall(self,s):
    for i in range(self.num_vert):
      for j in range(self.num_vert):
        if i == j:
          dist[i][j] = 0
        elif self.mat_adj[i][j] != 0:
          dist[i][j] = self.mat_adj[i][j]
          pred[i][j] = i
        else: 
          dist[i][j] = float("inf")
          pred[i][j] = 0
    for k in range(self.num_vert):
      for i in range(self.num_vert):
        for j in range(self.num_vert):
          if dist[i][j] > dist[i][k] + dist[k][j]:
            dist[i][j] = dist[i][k] + dist[k][j]
            pred[i][j] = pred[k][j]

  def caminho_minimo_l(self,s):
    custo = 0
    dist = [float("inf") for i in range(self.num_vert)]
    pred = [None for i in range(self.num_vert)]
    q = [s]
    dist[s] = 0
    while q:
      u = q.pop(0)
      for (v, w) in self.lista_adj[u]:
        custo = custo + 1
        if dist[v] == float("inf"):
          q.append(v)
          dist[v] = dist[u] + 1
          pred[v] = u
    return dist, pred, custo
  
  def melhor_codigo(self):
    print("Informe o arquivo:", end = " ")
    arquivo = input()
    print()
    print("Informe o vertice de origem:", end = " ")
    origem = int(input())
    print("Informe agora o vertice destino:", end = " ")
    destino = int(input())
    print("processando...")
    self.ler_arquivo_t(arquivo)
    custo = 0
    if self.tipo == 0:
      print("Utilizando Busca em largura")
      time_inicial = time.time()
      r = self.caminho_minimo_l(origem)
      time_final = time.time()
      Grafo.print_dijkstra(r[1],origem,destino)
      custo = r[2]
    elif self.tipo == 1: 
      print("Utilizando Dijkstra")
      time_inicial = time.time()
      r = self.dijkstra(origem)
      time_final = time.time()
      Grafo.print_dijkstra(r[1],origem,destino)
      custo = r[2]
    else:
      print("Utilizando Bellman Ford")
      time_inicial = time.time()
      r = self.bellman_ford(origem)
      time_final = time.time()
      Grafo.print_dijkstra(r[1],origem,destino)
      custo = r[2]
    print("Custo: ",custo)
    print("Tempo: ",time_final - time_inicial, "segundos")
        
  
  '''def cria_dimacs(self):
    meuArquivo = open('grafo.txt', 'w')
    meuArquivo.write(self.num_vert, ' ', self.num_arestas, '\n')'''
    
    
    
  
  

      