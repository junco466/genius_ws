import numpy as np

def pesos(d):
    
    w = [-0.2, -0.6, 1, 0.6, 0.2]
    lista = list(divList(d, len(d)//5))
    print(len(lista))

    prom = []
    for ob in lista:
        prom.append(np.mean(ob))

    print(prom)
    aux = np.matmul(w,prom)
    # suma = sum(abs(aux))
    print(aux)

        
    # angulo = 90
    # h = 1
    # aux = np.zeros(angulo//5)
    # aux1 = 0

    # while h < 6:

    #     aux = dist[angulo*h//5]

    #     for dist in p:
    #         pass

    #     aux1 = angulo*h//5
    #     h = h+1

def divList(p, n):
    for i in range(0,len(p), n):
        yield p[i:i+n]


p = np.random.uniform(low=0,high=4,size=90)
pesos(p)

