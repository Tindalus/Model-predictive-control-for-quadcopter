import matplotlib.pyplot as plt
import numpy as np

Ts = 0.1 #Time step
Tstop = 20 #Simulation length in seconds
N = int(Tstop/Ts)
n = 12

State = np.load('D:\Python\ConfData\MPC_X.npy')
Input = np.load('D:\Python\ConfData\MPC_U.npy')

Zcoord = [State[i][0][0] for i in range(N)]
Xcoord = [State[i][2][0] for i in range(N)]
Ycoord = [State[i][4][0] for i in range(N)]
U3 = [Input[i][2][0] for i in range(N)]
U1 = [Input[i][0][0] for i in range(N)]
U2 = [Input[i][1][0] for i in range(N)]

print(max(Zcoord))

t = np.arange(0,Tstop,Ts)   
figure, axis = plt.subplots(6, 1) 
axis[0].plot(t, Zcoord, color='black', linewidth=2, markersize=2) 
axis[0].set_title("z coordinate response") 
axis[0].grid()
axis[0].set_xlabel('t, с.')
axis[0].set_ylabel('z(t), м.')
axis[1].step(t, U3,color='black') 
axis[1].set_title("Regulator output") 
axis[1].grid()
axis[1].set_xlabel('t, с.')
axis[1].set_ylabel('U3')
axis[2].plot(t, Xcoord, color='black', linewidth=2, markersize=2) 
axis[2].set_title("x coordinate response") 
axis[2].grid()
axis[2].set_xlabel('t, с.')
axis[2].set_ylabel('x(t), м.')
axis[3].step(t, U1,color='black') 
axis[3].set_title("Regulator output") 
axis[3].grid()
axis[3].set_xlabel('t, с.')
axis[3].set_ylabel('U1')
axis[4].plot(t, Ycoord, color='black', linewidth=2, markersize=2) 
axis[4].set_title("y coordinate response") 
axis[4].grid()
axis[4].set_xlabel('t, с.')
axis[4].set_ylabel('y(t), м.')
axis[5].step(t, U2,color='black') 
axis[5].set_title("Regulator output") 
axis[5].grid()
axis[5].set_xlabel('t, с.')
axis[5].set_ylabel('U2')
plt.tight_layout()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(Xcoord, Ycoord, Zcoord, 'blue')
ax.set_xlim3d(-1, 1)  # X-axis from 0 to 1
ax.set_ylim3d(-1, 1)  # Y-axis from 0 to 1
ax.set_zlim3d(0, 1)  # Z-axis from 0 to 1
plt.show()
