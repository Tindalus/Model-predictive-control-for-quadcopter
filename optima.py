import numpy as np
from scipy.optimize import minimize
from dynamics import dynamics

def optima(Xdesired:list,Udesired:list,Xinitial:list,PredHorizon:float,PredStep:float,Weights1:list,Weights2:list):
    """
    This function returns solution of the optimal criterion acounting for model prediction
    
    Args:
        Xdesired (listlike): listlike of the cut of desired trajectory. Dimentions (int(PredHorizon/PredStep)*n,1)
        Udesired (listlike): listlike of desired input signal for the whole trajectory + some more (due to the funtion trying to predict beyond simulation time) times * input dimention, so for example ((N+100)*m,1)
        Xinitial (listlike): listlike of the initial state variables. (n,1), where n is the number of state dimentions
        PredHorizon (float): time of prognosis in seconds
        PredStep (float): numerical grid step for prediction model
        Weights1 (listlike): weights for the first term of optimal criterion
        Weights2 (listlike): weights for the second term of optimal criterion

    Returns:
        dict: U['x'] contains the sequence of inputs for the whole prediction period. See more in the documentation for minimize function from scipy.optimize https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html 
    """
    n = 12 #X vector dimention
    m = 6 #U vector dimention
    HorizonLen = int(PredHorizon/PredStep)

    State = np.zeros(shape=(n*HorizonLen,1)) #empty state vector

    #defining cost function
    def Cost(U):
        T = 1
        X = Xinitial #setting initial state in the nested function even though the input goes in the main one
        
        #transforming bullshit for minimize function from scipy
        U_ = np.zeros(shape = (len(U),1))
        U_[0:len(U),0] = U[:]
        U = U_
        

        #propagating trajectory
        for i in range(HorizonLen):
            X,T = dynamics(PredStep,X,U[i*m:(i+1)*m,:],0)
            #State acquisition
            State[i*n:(i+1)*n,:] = X

        #convertion for phi and theta decired angles
        for i in range(HorizonLen):
            Xdesired[i*n+8] = np.arctan((U[1]*np.cos(Xdesired[i*n+10])+U[2]*np.sin(Xdesired[i*n+10]))/T) 
            Xdesired[i*n+6] = np.arctan((U[1]*np.sin(Xdesired[i*n+10])-U[2]*np.cos(Xdesired[i*n+10]))/T)
        

        #state error vector computations
        ErrorVector = np.zeros(shape = (n*HorizonLen,1))
        for i in range(HorizonLen):
            ErrorVector[i*n:(i+1)*n,:] = Xdesired[i*n:(i+1)*n,:]-State[i*n:(i+1)*n,:]

        #Input difference error
        InputErrorVector = np.zeros(shape = (m*HorizonLen,1))
        for i in range(HorizonLen):
            InputErrorVector[i*m:(i+1)*m,:] = U[0:m,:] - U[i*m:(i+1)*m,:]

        InputAccuracyVector = np.zeros(shape = (m*HorizonLen,1))
        for i in range(HorizonLen):
            InputAccuracyVector[i*m:(i+1)*m,:] = Udesired[i*m:(i+1)*m,:] - U[i*m:(i+1)*m,:]

        #cost funtion value (MPC2013 p.205)
        F = 0
        for i in range(HorizonLen):
            F1 = np.matmul(ErrorVector[i*n:(i+1)*n,:].T,np.matmul(Weights1[i*n:(i+1)*n,i*n:(i+1)*n],ErrorVector[i*n:(i+1)*n,:]))
            F2 = np.matmul(InputErrorVector[i*m:(i+1)*m,:].T,np.matmul(Weights2[i*m:(i+1)*m,i*m:(i+1)*m],InputErrorVector[i*m:(i+1)*m,:]))
            # F3 = np.matmul(InputAccuracyVector[i*m:(i+1)*m,:].T,np.matmul(Weights3[i*m:(i+1)*m,i*m:(i+1)*m],InputAccuracyVector[i*m:(i+1)*m,:]))
            F = F + 0.5*(F1 + F2)
        return F[0][0]


    #initial guess
    U0 = [0]*m*HorizonLen

    #forming bound for input U
    bnd = []
    for i in range(HorizonLen):
        bnd.append((-10,10)) # u1 bound
        bnd.append((-10,10)) # u2 bound
        bnd.append((-10,10)) # u3 bound
        bnd.append((-10,10)) # u4 bound
        bnd.append((-10,10)) # u5 bound
        bnd.append((-10,10)) # u6 bound
    Bounds = tuple(bnd)

    # finding minimum of the cost function
    U = minimize(Cost,U0,method='SLSQP',bounds = Bounds)

    return U

