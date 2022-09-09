from ctypes import LibraryLoader
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
#import jupyter
import matplotlib.pyplot as plt

# lidarDist = ctrl.Antecedent(np.arange(0.3, 3.8, 0.1), 'distance')
# direction = ctrl. Antecedent(np.arange(0, 10, 1), 'direction')
lid = ctrl.Antecedent(np.arange(-5,8,0.1),'detection')

linearSpeed = ctrl.Consequent(np.arange(0, 1, 0.01), 'linear')
angularSpeed = ctrl.Consequent(np.arange(-3.0, 3, 0.1), 'angular')

lid['choque-derecha'] = fuzz.trimf(lid.universe, [-5, -5, -3])
lid['corregir-derecha'] = fuzz.trimf(lid.universe, [-4, -2, 0])
lid['choque-centro'] = fuzz.trimf(lid.universe, [-1, 0, 2])
lid['corregir-izquierda'] = fuzz.trimf(lid.universe, [0, 2.5, 4])
lid['ideal'] = fuzz.trimf(lid.universe, [3.5, 4, 4.5])
lid['corregir-izquierda2'] = fuzz.trimf(lid.universe, [4, 5, 7])
lid['choque-izquierda'] = fuzz.trimf(lid.universe, [6, 7, 8])

# lidNames = ['right-center','right','center','right','farRight']
# lid.automf(names=lidNames)
    
# lidarNames = ['closest','close','medium','far','further']
# lidarDist.automf(names=lidarNames)

# directionNames = ['farLeft','left','center','right','farRight']
# direction.automf(names=directionNames)

# linearNames = ['slow','average','fast']
# linearSpeed.automf(names=linearNames)

linearSpeed['stop'] = fuzz.trimf(linearSpeed.universe, [0, 0, 0.05])
linearSpeed['slow'] = fuzz.trimf(linearSpeed.universe, [0.04, 0.1, 0.2])
linearSpeed['average'] = fuzz.trimf(linearSpeed.universe, [0.1, 0.5, 0.9])
linearSpeed['fast'] = fuzz.trimf(linearSpeed.universe, [0.8, 1 ,1])

angularNames = ['sharpRight','right','softRight','stright','softLeft','left','sharpLeft']
angularSpeed.automf(names=angularNames)

# fuzzyLidar.view()
# plt.show()

# direction.view()
# plt.show()

# lid.view()
# plt.show()

linearSpeed.view()
plt.show()

# angularSpeed.view()
# plt.show()

#REGLAS:
rule1 = ctrl.Rule(lid['choque-derecha'],consequent= (linearSpeed['stop'],angularSpeed['sharpLeft']))

rule2 = ctrl.Rule(lid['choque-izquierda'], consequent = (linearSpeed['stop'], angularSpeed['sharpRight']))

rule3 = ctrl.Rule(lid['choque-centro'], consequent = linearSpeed['stop'])

rule4 = ctrl.Rule(lid['corregir-derecha'], (linearSpeed['average'], angularSpeed['right']))
rule5 = ctrl.Rule(lid['corregir-izquierda'], (linearSpeed['average'], angularSpeed['left']))

rule6 = ctrl.Rule(lid['corregir-izquierda2'], (linearSpeed['average'], angularSpeed['left']))

rule7 = ctrl.Rule(lid['ideal'], (linearSpeed['fast'],angularSpeed['stright']))


# Se crea el contorlador del sistema
bot_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7])
# Se realiza una simulación dle controlador par auna cituación en específico
bot_sim = ctrl.ControlSystemSimulation(bot_ctrl)
# Se dan valores a las entradas del sistema
bot_sim.input['detection'] = 0

# Se procesan los datos y se obtiene el resultado
bot_sim.compute()
# print('linear: ' ,bot_sim.output['linear'])
# print('angular: ', bot_sim.output['angular'])
bot_sim.output['']