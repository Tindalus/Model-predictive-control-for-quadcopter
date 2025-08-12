import numpy as np
from scipy.optimize import minimize
from dynamics import dynamics
import os
from optima import optima

#simulation parameters
n = 12 #State vector dimention
m = 6 #Input vector dimention
Ts = 0.1 #Time step
Tstop = 20 #Simulation length in seconds
N = int(Tstop/Ts)
X = np.zeros(shape = (n,1)) #inital conditions vector


#Storage
State = []
Input = []

#prediction parameters
PredHorizon = 2 #in seconds    #2 good
PredStep = 0.3



#weight  matrixes for cost function 
Weights1 = np.eye(n*int(PredHorizon/PredStep),n*int(PredHorizon/PredStep)) #for X vector
for i in range(int((int(PredHorizon/PredStep))/2)):
    #z coord
    Weights1[i*2*n,i*2*n] = np.float64(Weights1[i*2*n,i*2*n]*150*np.exp(0.005*i)) #positional #150*np.exp(0.01*i))
    Weights1[i*2*n+1,i*2*n+1] = np.float64(Weights1[i*2*n+1,i*2*n+1]*0.000001) #velocities #0.0001
    #x coord
    Weights1[i*2*n+2,i*2*n+2] = np.float64(Weights1[i*2*n+2,i*2*n+2]*50*np.exp(0.01*i)) #positional #150*np.exp(0.01*i))
    Weights1[i*2*n+3,i*2*n+3] = np.float64(Weights1[i*2*n+3,i*2*n+3]*100*np.exp(0.01*i)) #positional #150*np.exp(0.01*i))
    #y coord
    Weights1[i*2*n+4,i*2*n+4] = np.float64(Weights1[i*2*n+4,i*2*n+4]*50*np.exp(0.01*i)) #positional #150*np.exp(0.01*i))
    Weights1[i*2*n+5,i*2*n+5] = np.float64(Weights1[i*2*n+5,i*2*n+5]*100*np.exp(0.01*i)) #positional #150*np.exp(0.01*i))
    #phi coord
    Weights1[i*2*n+6,i*2*n+6] = np.float64(Weights1[i*2*n+6,i*2*n+6]*100*np.exp(0.005*i)) #positional #150*np.exp(0.01*i))
    Weights1[i*2*n+7,i*2*n+7] = np.float64(Weights1[i*2*n+7,i*2*n+7]*100*np.exp(0.005*i)) #positional #150*np.exp(0.01*i))
    #theta coord
    Weights1[i*2*n+8,i*2*n+8] = np.float64(Weights1[i*2*n+8,i*2*n+8]*100*np.exp(0.005*i)) #positional #150*np.exp(0.01*i))
    Weights1[i*2*n+9,i*2*n+9] = np.float64(Weights1[i*2*n+9,i*2*n+9]*100*np.exp(0.005*i)) #positional #150*np.exp(0.01*i))
    #psi coord
    Weights1[i*2*n+10,i*2*n+10] = np.float64(Weights1[i*2*n+10,i*2*n+10]*150*np.exp(0.005*i)) #positional #150*np.exp(0.01*i))
    Weights1[i*2*n+11,i*2*n+11] = np.float64(Weights1[i*2*n+11,i*2*n+11]*150*np.exp(0.005*i)) #positional #150*np.exp(0.01*i))


Weights2 = np.eye(m*int(PredHorizon/PredStep),m*int(PredHorizon/PredStep)) #for U vector
for i in range(int(int(PredHorizon/PredStep)*m)):
    Weights2[i,i] = np.float64(Weights2[i,i])*10000*np.exp(0.01*i) #for U vector #10000

# Weights3 = np.eye(m*int(PredHorizon/PredStep),m*int(PredHorizon/PredStep)) #for input error
# for i in range(int(int(PredHorizon/PredStep)*m)):
#     Weights3[i,i] = np.float64(Weights3[i,i])*1*np.exp(0.01*i) 




#tragectory generation
DesiredTrajectory = np.zeros(shape=((N+100)*n,1))
VariableIndex=0
for j in range(0,N+100):
    DesiredTrajectory[j*n+VariableIndex] = 1
VariableIndex=2
for j in range(100,N+100):
    DesiredTrajectory[j*n+VariableIndex] = 1
# VariableIndex=4
# for j in range(450,N+100):
#     DesiredTrajectory[j*n+VariableIndex] = 1


#desired input signal
Udesired = np.zeros(shape = ((N+100)*m,1))


#progress bar lol
ProgressBar = ['.' for j in range(100)]

#Main cycle
for j in range(N):

    if round((j*Ts*100)%(PredStep*100)) == 0: #only calculating U every Ts/Predstep tics
        #prediction cut for curent step
        Xdesired = DesiredTrajectory[j*n:(j+int(PredHorizon/PredStep))*n,:]
        U = optima(Xdesired,Udesired,X,PredHorizon,PredStep,Weights1,Weights2)
        
    Uapplied = np.zeros(shape=(m,1))
    Uapplied[:,0] = U['x'][0:m]
    X,T = dynamics(Ts,X,Uapplied,1) #appling only first input vector
    
    State.append(X) #saving X state vector at j time moment
    Input.append(Uapplied) #saving applied U state vector at j time moment

    #Progress displey
    ProgressBarSum = ''
    ProgressBar[int((j)/N*100)] = '/'
    for i in range(100):
        ProgressBarSum += ProgressBar[i]
    os.system('cls' if os.name == 'nt' else 'clear')
    print(f'{int((j+1)/N*100)}'+'% '+'Calculated ' + f'[{ProgressBarSum}]')

print('\nDone')

#saving data for plotting in different file
np.save(os.path.join('D:\Python\ConfData\MPC_X'),State)
np.save(os.path.join('D:\Python\ConfData\MPC_U'),Input)
