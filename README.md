**Controller based on MPC method and its apptication for quadcopter dynamic model control**  
These materials have been used to obtain results published in [this paper](https://doi.org/10.26160/2541-8637-2025-15-7-16).  

**Quick overview of the files included:**  
`optima.py` contains function of controller implementation. The controller does:
1. model behavior prognosis
![model prognosis](resources/modelPrognosis.png)
2. error calcutation
![error](resources/error.png)
3. otimal criterion calcutation
![error](resources/optimal.png)
4. minimization of this criterion.  
_Mind that it uses `minimize` function from `scipy.optimize`._  
This function also contains in itself initial input signal guess as well as bound conditions for signal set to (-10,10). Both of this can be changed if needed.

`dynamics.py` conteins dynamic model funtion (see Quadcopter-model [repository](https://github.com/Tindalus/Quadcopter-model)). In this case it outputs total angular velocity as well.  
`Model.py` central script simulating model's controlled behavior and containg controller parameters and desired projectory setup.

## Setup
