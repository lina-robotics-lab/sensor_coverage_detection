# Using Extended Kalman Filter for Estimation under Nonlinear Dynamics

Resource from https://www.mathworks.com/help/control/ref/extendedkalmanfilter.html

Recall in linear Kalman Filter we have

$$
x[n+1]=Ax[n]+Bu[n]+Bw[n]
\\
y[n]=Cx[n]
$$

In extended Kalman Filter, we relax the assumption that the measurement model is linear.

$$
x[n+1]=f(x[n],u[n],w[n])
\\
y[n]=h(x[n])
$$

And normally, we assume w=0.

If we let
$$
A = \frac{\part f}{\part x}\\
B = \frac{\part f}{\part u}\\
C = \frac{\part h}{\part x}\\
$$
Then we can still calculate the M matrix, and apply the same P-controller on estimation error.

Under this setup, the update for Extended Kalman Filter becomes:
$$
\hat x[n|n] = \hat x[n|n-1]+M(y_v[n] - f(x[n|n-1]n,u[n],0))\\
\hat y[n|n] = h(x[n|n])
$$
Notice we use f and h as backbox functions which accept queries.