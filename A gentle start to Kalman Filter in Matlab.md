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

**The measurement model assumed by Kalman Filter** 

Kalman Filter users assume there is a system with unknown state x. Only the input value u and noisy output value $y_v[n]$ can be measured.

The state space of this interesting system is:
$$
x[n+1]=Ax[n]+Bu[n]+Bw[n]
\\
y[n]=Cx[n]
$$

Here x is system state to be estimated, u is input to system, w is input noise but treated as input as well. y is the measurement output on state x. Noisy output y_v[n] will be defined later.

A, B, C are matrices. They are assumed to be known parameters, and should be fed to the Kalman Filter.

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



Q and R are parameters to be passed into Kalman Filter. They basically contains all the information about noises in the system.

***Remark: it is assumed that we know the system dynamics, from A, B, and C matrices; we also know the noise distribution, from Q and R matrices; what we do not know is the system state x, and that is to be estimated from a series of input-respond(u,y_v) values.*** 

Now we are ready to create our Kalman Filter.

```matlab
Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');		
Q = 1; 
R = 1;
[kalmf,L,P,M] = kalman(Plant,Q,R);
```

`kalmf` is a state space model, which is the block for Kalman Filter.

To under stand what are L, P, M and the actual structure of kalmf, we introduce the prediction model of Kalman Filter.

**The prediction model of Kalman Filter**

First, let's do some easy control theory maths again.

The prediction model essentially defines the Kalman Filter as a linear plant.

The inputs of Kalman Filter are
$$
u[n]
$$

$$
y_v[n]
$$



The state of Kalman Filter are the real-state estimations, namely
$$
\hat x
$$
The output of Kalman Filter is output estimation
$$
\hat y
$$



First, after a new measure of y_v[n] is received, we can calculate
$$
\hat x[n|n]=\hat x[n|n−1]+M(y_v[n]−C\hat x[n|n−1])
$$
all the hats means they are estimated values rather than true measurements.
$$
\hat x[n|n-1]=>priori-estaimation
$$

$$
\hat x[n|n]=>posteriori-estimation
$$

priori/posteriori corresponds to the estimation before or after the measurement of y_v.

What this equation does is a P-controller on x[n|n, such that the quasi-estimation error y_v[n]-Cx[n|n-1] is minimized. 

**So, M is the control gain of quasi-estimation error.**

Suppose after this, we also receive the measurement of input u[n], then
$$
\hat x[n+1|n]=A\hat x[n|n]+Bu[n]
$$
Also, the calculation of output estimation mimics the actual system dynamics
$$
\hat y[n|n] = C\hat x[n|n]
$$


So we calculate the priori estimation for the next time-step, and that is ready to be used when the next y_v[n+1] measurement comes in.

In fact, the x[n|n] state is redundant in this model if we always get y_v[n] and u[n] simultaneously. 

---

**Express ^x[n|n] in terms of ^x[n|n-1] in both ^y[n|n] and ^x[n+1|n] yields:**
$$
\hat x[n+1|n]=A(I−MC)\hat x[n|n−1]+[B\ \ AM][u[n]\ \ y_v[n]]'
$$

$$
\hat y[n|n]=C(I−MC)\hat x[n|n−1]+CMy_v[n].
$$

**That becomes the state transition equations of Kalman Filter, in standard LTI form.**

---

End of control theory maths.



Start of some optimization and statistics maths.

What should we set as the control gain M? The best way is to setup an optimization objective, and choose M such that the objective is optimized.

We define the **squared expected measurement error**, which is the trace of error covariance,  to be our objective :
$$
Objective=trace(P) = trace(E[(x-\hat x)(x-\hat x)^T]),\hat x \ depends \ on \ M
$$
**So, P is the measurement error covariance matrix.**

Take derivative over the objective over M, set the derivative to be 0, solve for M, obtain the optimal M, use it in our Kalman Filter. Detailed calculation omitted. 

End of all maths.





Back to matlab code. If we give some value to the parameter matrices manually,

```matlab
A = [1.1269   -0.4940    0.1129;
     1.0000         0         0;
          0    1.0000         0];

B = [-0.3832;
      0.5919;
      0.5191];

C = [1 0 0];

Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');

Q = 1; 
R = 1;
[kalmf,L,P,M] = kalman(Plant,Q,R);

>> M

M =

    0.3798
    0.0817
   -0.2570

>> P

P =

    0.6124    0.1318   -0.4144
    0.1318    0.7301    0.3890
   -0.4144    0.3890    0.9888

```

We can verify if kalmf contains the correct LTI system as specified in the maths

```matlab

>> kalmf

kalmf =
 
  A = 
             x1_e      x2_e      x3_e
   x1_e    0.7683    -0.494    0.1129
   x2_e    0.6202         0         0
   x3_e  -0.08173         1         0
 
  B = 
               u        y
   x1_e  -0.3832   0.3586
   x2_e   0.5919   0.3798
   x3_e   0.5191  0.08173
 
  C = 
             x1_e      x2_e      x3_e
   y_e     0.6202         0         0
   x1_e    0.6202         0         0
   x2_e  -0.08173         1         0
   x3_e     0.257         0         1
 
  D = 
               u        y
   y_e         0   0.3798
   x1_e        0   0.3798
   x2_e        0  0.08173
   x3_e        0   -0.257
```

In particular, if C = [1 0 0], there should be 
$$
A(I-M[1\ 0\ 0])=[A_1-M|A_2|A_3]
$$
See the `kalmf` above, and this is true.

End of lengthy system modeling on KF.

## Simulate an estimation process with Kalman Filter

Create a real plant.

```matlab
a = A;
b = [B B 0*B];
c = [C;C];
d = [0 0 0;0 0 1];
P = ss(a,b,c,d,-1,'inputname',{'u' 'w' 'v'},'outputname',{'y' 'yv'});
```

Check the corresponding maths model.


$$
\dot x = Ax + Bu + Bw+0v\\
y = Cx+0u+0w+0v\\
y_v = Cx +0u+0w+ v
$$


Create a corresponding Mimic_Plant, with the same dynamcis

```matlab
Mimic_Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
```

Notice the control input of P and Mimic_Plant should be identical, therefore w noise term is included in the Mimic_Plant, but v noise term is not included. 

Create a Kalman Filter on the Mimic_Plant,

```matlab
[kalmf, L, P, M] = kalman(Mimic_Plant,Q,R)
```

Delete the ^x output of kalmf

```matlab
kalmf = kalmf(1,:);
```

Create a parallel model for P and kalmf

---

parallel()

`sys = parallel(sys1,sys2,inp1,inp2,out1,out2) `  forms the more general parallel connection shown in the following figure.

![img](https://www.mathworks.com/help/control/ref/refl2p135.gif)



The vectors `inp1` and `inp2` contain indexes into the input channels of `sys1` and `sys2`, respectively, and define the input channels *u1* and *u2* in the diagram. Similarly, the vectors `out1` and `out2` contain indexes into the outputs of these two systems and define the output channels *y1* and *y2* in the diagram. The resulting model `sys` has [*v1* ; *u* ; *v2*] as inputs and [*z1* ; *y* ; *z2*] as outputs. 

---

Then use `parallel` to form the parallel connection of the following illustration.

![img](https://www.mathworks.com/help/examples/control/win64/KalmanFilteringExample_03.gif)

```matlab
sys = parallel(P,kalmf,1,1,[],[]);
```

This means the 1st input of P and 1st input of kalmf are shared input, other input/output are left intact.

Connect the 2nd output of sys to the 4th input of sys:

```matlab
SimModel = feedback(sys,1,4,2,1);   % Close loop around input #4 and output #2
SimModel = SimModel([1 3],[1 2 3]); % Delete yv from I/O list
% SimModel = SimModel(out_indices, in_indices)
```

sys_2_out->Identity Plant->*(+1)->sys_4_in

Prepare input data

```
t = [0:100]';
u = sin(t/5);

n = length(t);
rng default
w = sqrt(Q)*randn(n,1);
v = sqrt(R)*randn(n,1);
```

Call lsim to run simulation

```
[out,x] = lsim(SimModel,[w,v,u]);

y = out(:,1);   % true response
ye = out(:,2);  % filtered response
yv = y + v;     % measured response
```

Plot out response, done.

