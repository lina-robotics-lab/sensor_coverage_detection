# Quick started to Control System Toolbox in Matlab

Tutorial resource is grabbed from matlab documentations: https://www.mathworks.com/help/control/getstart/linear-lti-models.html

Let's begin by building some simple control models.

## Steady State Systems specified by A, B, C D

```matlab
R= 2.0 % Ohms
L= 0.5 % Henrys
Km = .015 % torque constant
Kb = .015 % emf constant
Kf = 0.2 % Nms
J= 0.02 % kg.m^2

A = [-R/L -Kb/L; Km/J -Kf/J]
B = [1/L; 0];
C = [0 1];
D = [0];
sys_dc = ss(A,B,C,D)
%% ss stands for 'steady state'
```

```matlab
>> get(sys_dc)
                A: [2×2 double]
                B: [2×1 double]
                C: [0 1]
                D: 0
                E: []
           Scaled: 0
        StateName: {2×1 cell}
        StateUnit: {2×1 cell}
    InternalDelay: [0×1 double]
       InputDelay: 0
      OutputDelay: 0
               Ts: 0
         TimeUnit: 'seconds'
        InputName: {''}
        InputUnit: {''}
       InputGroup: [1×1 struct]
       OutputName: {''}
       OutputUnit: {''}
      OutputGroup: [1×1 struct]
            Notes: [0×1 string]
         UserData: []
             Name: ''
     SamplingGrid: [1×1 struct]

>> sys_dc.A

ans =

   -4.0000   -0.0300
    0.7500  -10.0000
```

## Using a Kalman Filter with an existing system

Tutorial from matlab official website:https://www.mathworks.com/help/control/ug/kalman-filtering.html

**The measurement model for Kalman Filter** 

$$
x[n+1]=Ax[n]+Bu[n]+Bw[n]
$$
$$
y[n]=Cx[n]
$$

Here x is system state to be estimated, u is input to system, w is input noise but treated as input as well. y is the measurement output on state x. 

A, B, C are matrices.

Converting this model into a state space model object in matlab, we use

```matlab
Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');		
```

---

*The docs of ss(A,B,C,D,Ts) explains this object initialization:*

*`sys = ss(A,B,C,D,Ts) `  creates the discrete-time model*
$$
x[n+1]=Ax[n]+Bu[n]
$$
$$
y[n]=Cx[n]+Du[n]
$$



*with sample time `Ts` (in seconds). Set `Ts = -1` or `Ts = []` to leave the sample time unspecified.*

---



Back to the mathematical description of measurement model, let $y_v[n]$ be a noisy measurement of the output y[n], with v[n] denoting the measurement noise:
$$
y_v[n]=y[n]+v[n]
$$
It is assumed we only receive data in the form of y_v[n], but not directly y[n] itself.

Next, we introduce the covariance matrices Q and R, which are another two important variable in Kalman Filter.
$$
E(w[n]w[n]^T)=Q
$$

$$
E(v[n]v[n]^T)=R
$$



Now we are ready to create our Kalman Filter.

```matlab
Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');		
Q = 1; 
R = 1;
[kalmf,L,P,M] = kalman(Plant,Q,R);
```

`kalmf` is a state space model, which is the block for Kalman Filter.