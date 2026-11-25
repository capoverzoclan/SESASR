import numpy as np
import sympy
from math import cos,sin
from sympy import symbols, Matrix


def velocity_mm_Gt(x, u, dt):
    """
    Evaluate Jacobian Gt w.r.t state x=[x, y, theta]
    """
    theta = x[2]
    v, w = u[0], u[1]
    r = v / w
    Gt = np.array(
        [
            [1, 0, -r * cos(theta) + r * cos(theta + w * dt)],
            [0, 1, -r * sin(theta) + r * sin(theta + w * dt)],
            [0, 0, 1],
        ]
    )

    return Gt


def velocity_mm_Vt(x, u, dt):
    """
    Evaluate Jacobian Vt w.r.t command u=[v,w]
    """
    theta = x[2]
    v, w = u[0], u[1]
    r = v / w
    Vt = np.array(
        [
            [
                -sin(theta) / w + sin(theta + w * dt) / w,
                dt * v * cos(theta + w * dt) / w + v * sin(theta) / w**2 - v * sin(theta + w * dt) / w**2,
            ],
            [
                -cos(theta) / w - cos(theta + w * dt) / w,
                dt * v * sin(theta + w * dt) / w - v * cos(theta) / w**2 + v * cos(theta + w * dt) / w**2,
            ],
            [0, dt],
        ]
    )

    return Vt



def squeeze_sympy_out(func):
    # inner function
    def squeeze_out(*args):
        out = func(*args).squeeze()
        return out

    return squeeze_out


def velocity_mm_simpy():
    """
    Define Jacobian Gt w.r.t state x=[x, y, theta] and Vt w.r.t command u=[v, w]
    """
    x, y, theta, v, w, dt = symbols("x y theta v w dt")
    R = v / w
    beta = theta + w * dt
    gux = Matrix(
        [
            [x - R * sympy.sin(theta) + R * sympy.sin(beta)],
            [y + R * sympy.cos(theta) - R * sympy.cos(beta)],
            [beta],
        ]
    )

    eval_gux = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), gux, "numpy"))

    Gt = gux.jacobian(Matrix([x, y, theta]))
    eval_Gt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Gt, "numpy"))
    #print("Gt:", Gt)

    Vt = gux.jacobian(Matrix([v, w]))
    eval_Vt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Vt, "numpy"))
    #print("Vt:", Vt)

    return eval_gux, eval_Gt, eval_Vt


def landmark_sm_simpy():
    x, y, theta, mx, my = symbols("x y theta m_x m_y")

    hx = Matrix(
        [
            [sympy.sqrt((mx - x) ** 2 + (my - y) ** 2)],
            [sympy.atan2(my - y, mx - x) - theta],
        ]
    )
    eval_hx = squeeze_sympy_out(sympy.lambdify((x, y, theta, mx, my), hx, "numpy"))

    Ht = hx.jacobian(Matrix([x, y, theta]))
    eval_Ht = squeeze_sympy_out(sympy.lambdify((x, y, theta, mx, my), Ht, "numpy"))
    # print("Ht:", Ht)

    return eval_hx, eval_Ht

def velocity_mm_extended_simpy():
    """
    Define Jacobian Gt w.r.t state x=[x, y, theta] and Vt w.r.t command u=[v, w]
    """
    x, y, theta, v, w, dt = symbols("x y theta v w dt")
    R = v / w
    beta = theta + w * dt
    gux = Matrix(
        [
            [x - R * sympy.sin(theta) + R * sympy.sin(beta)],
            [y + R * sympy.cos(theta) - R * sympy.cos(beta)],
            [beta],
            [v],
            [w],
        ]
    )
    #print("gux:", gux)

    eval_gux = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), gux, "numpy"))

    Gt = gux.jacobian(Matrix([x, y, theta,v,w]))
    eval_Gt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Gt, "numpy"))
    #print("Gt:", Gt)

    Vt = gux.jacobian(Matrix([v, w]))
    eval_Vt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Vt, "numpy"))
    #print("Vt:", Vt)

    return eval_gux, eval_Gt, eval_Vt

def odometry_sm_simpy():
    x, y, theta, v, w = symbols("x y theta v w")
    h_odom = Matrix([[v],[w]])
    eval_h_odom = squeeze_sympy_out(sympy.lambdify((v, w), h_odom, "numpy"))
    H_odom = h_odom.jacobian(Matrix([x, y, theta, v, w]))
    eval_H_odom = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w),H_odom,"numpy"))
    #print("h: ", h_odom)
    #print("H: ", H_odom)
    return eval_h_odom, eval_H_odom

def imu_sm_simpy():
    x, y, theta, v, w = symbols("x y theta v w")
    h_imu=Matrix([w])
    eval_h_imu = squeeze_sympy_out(sympy.lambdify(w,h_imu,"numpy"))
    H_imu = h_imu.jacobian(Matrix([x, y, theta, v, w]))
    eval_H_imu = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w),H_imu,"numpy"))
    #print(h_imu)
    #print(H_imu)
    return eval_h_imu, eval_H_imu

def landmark_sm_extended_simpy():
    x, y, theta, v, w, mx, my = symbols("x y theta v w m_x m_y")

    hx = Matrix(
        [
            [sympy.sqrt((mx - x) ** 2 + (my - y) ** 2)],
            [sympy.atan2(my - y, mx - x) - theta],
        ]
    )
    eval_hx = squeeze_sympy_out(sympy.lambdify((x, y, theta, mx, my), hx, "numpy"))

    Ht = hx.jacobian(Matrix([x, y, theta, v, w]))
    eval_Ht = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, mx, my), Ht, "numpy"))
    #print("Ht:", Ht)

    return eval_hx, eval_Ht