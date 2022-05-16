import grafos_matriz as grafos
import grafos_lista as gl
import grafop

'''g_lista =[[1],
          [0,3],
          [3],
          [1,2]]

g_mat_teste =[[0,1,1,1,1],
              [1,0,1,1,1],
              [1,1,0,1,1],
              [1,1,1,0,1],
              [1,1,1,1,0]
              ]
g_matriz = [[0,1,0,0],
            [1,0,0,1],
            [0,0,0,1],
            [0,1,1,0]  
            ]

g2_lista = [[1,3,4],
            [0,4],
            [],
            [0,4],
            [0,1,3]]

g2_matriz =[[0,1,0,1,1],
            [1,0,0,1,1],
            [0,0,0,0,0],
            [1,0,0,0,1],
            [1,1,0,1,0]
            ]
'''
'''print("o vértice possui grau ", grafos.grau(g_matriz,1))
if grafos.regular(g_matriz):
  print("Grafo Regular")
else:
  print("Grafo não regular")
print(grafos.completo(g_mat_teste))
for i in range(10):
  g_mat = grafos.gerador_de_grafo(5)
  grafos.printa_grafo(g_mat)
  print()
  print(grafos.completo(g_mat))
g1 = grafo.Grafo(4)
print(g1.mat_adj)
g1.add_aresta(0,2)
g1.add_aresta(1,3)
print(g1.mat_adj)
gl.printa_grafo_lista(gl.gerador_de_grafo_lista(5))'''
'''g1 = grafop.Grafo()
g1.ler_arquivo("grafo1.txt")
print(g1.lista_adj)
print()
g2 = grafop.Grafo()
g2.ler_arquivo("grafo2.txt")
for elem in g2.lista_adj:
  print(elem)
d = g1.densidade()
print()
print(d)
d = g2.densidade()
print(d)'''

g1 = grafop.Grafo()
'''grafos.printa_grafo(g1.mat_adj)'''
'''l = g1.busca_largura(0)
for elem in l:
  print(elem)
'''
'''l = g1.busca_profundidade(0)
for elem in l:
  print(elem)'''
g1.melhor_codigo()
