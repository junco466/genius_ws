from ctypes import LibraryLoader
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
#import jupyter
import matplotlib.pyplot as plt

lidarDist = ctrl.Antecedent(np.arange(0.3, 3.8, 0.1), 'distance')
direction = ctrl. Antecedent(np.arange(0, 10, 1), 'direction')

linearSpeed = ctrl.Consequent(np.arange(0, 1, 0.01), 'linear')
angularSpeed = ctrl.Consequent(np.arange(-3.0, 3, 0.1), 'angular')

lidarNames = ['closest','close','medium','far','further']
lidarDist.automf(names=lidarNames)

directionNames = ['farLeft','left','center','right','farRight']
direction.automf(names=directionNames)

linearNames = ['slowest','slower','slow','average','fast','faster','fastest']
linearSpeed.automf(names=linearNames)

angularNames = ['sharpRight','right','softRight','stright','softLeft','left','sharpLeft']
angularSpeed.automf(names=angularNames)

fuzzyLidar.view()
plt.show()

direction.view()
plt.show()

linearSpeed.view()
plt.show()

angularSpeed.view()
plt.show()

#REGLAS:
rule1 = ctrl.Rule(lidarDist['closest'], linearSpeed['slowest'])
rule2 = ctrl.Rule(lidarDist['further'], linearSpeed['fastest'])
rule3 = ctrl.Rule(direction['center'] & lidarDist['close'], linearSpeed['slowest'])
