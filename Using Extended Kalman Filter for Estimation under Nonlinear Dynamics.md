# Using Extended Kalman Filter for Estimation under Nonlinear Dynamics

The [EKFUsageDemo.m](EKFUsageDemo.m) contains a working script for this documentation. You may run it in matlab to see how to actually use EKF in matlab.



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
\hat x[n|n] = \hat x[n|n-1]+M(y_v[n] - f(\hat x[n|n-1]n,u[n],0))\\
\hat y[n|n] = h(\hat x[n|n])
$$
Notice we use f and h as blackbox functions which accept queries.

## EKF in matlab

```matlab
ekf = extendedKalmanFilter(StateTransitionFcn,MeasurementFcn,InitialState)
```

Creates an EKF.

In the maths notion defined above

```matlab
 ekf = extendedKalmanFilter(@f,@h,InitialState)
```

More concretely,

```matlab
ekf = extendedKalmanFilter(@dynamics.stateUpdate,@measure.measureUpdate,[0.1;0.1])
```

Notice `[0.1;0.1]` determines the shape of all following updates.

The expected signature of `f, h` are 

```matlab
f(x,u)
h(x,u,v)
```

Now we can introduce predict and correct function.

---

```matlab
ekf.predict(u)
```

This corresponding behavior is

`ekf.State=f(ekf.State,u)`

if u is absent(as in our application), directly call `ekf.predict()` to update `ekf.State`

---

```matlab
ekf.correct(y,u)
```

It corresponds to the following maths
$$
\hat x[n|n] = \hat x[n|n-1]+M(y_v[n] - h(\hat x[n|n-1],u[n]))
$$
and the following code

```matlab
ekf.State = ekf.State+ M(y-h(ekf.State,u))
```



in our application, u is absent, so we directly use

```matlab
ekf.correct(y_v)
```

to do the State estimation correction.

## *The standard code of EKF update   

```matlab
actual_loc=dynamics.stateUpdate(actual_loc);
plant_measurement = meas.measureUpdate(actual_loc);
ekf.correct(plant_measurement);
ekf.predict();
```
**Important: for the ekf states to be properly updated, the call of ekf.correct() must be followed by call ekf.predict() ! If we do not ekf.predict() after calling ekf.correct(), the state of ekf will be wrong!**

## Pay special attention to shape consistency when giving input to EKF

- y_v **must be of the same shape of the output of** `h`, crucial when calling ekf.correct().
- We typically will change the shape of y_v when modifying the number of sensors to use in Measurement class, so make sure the shape are consistent.
- The initial state should be of the same shape of the input of `f`, mind this when doing ekf initialization.
- The input and output shape of `f` should be the same. 

