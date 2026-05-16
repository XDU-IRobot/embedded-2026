import sympy as sp

def var_str(idx, name):
    return f"{name}{idx}"

def main():
    t = sp.Symbol('t')
    q1 = sp.Function('q1')(t)
    q2 = sp.Function('q2')(t)
    dq1 = sp.diff(q1, t)
    dq2 = sp.diff(q2, t)
    ddq1 = sp.diff(dq1, t)
    ddq2 = sp.diff(dq2, t)

    q = [q1, q2]
    dq = [dq1, dq2]
    ddq = [ddq1, ddq2]

    # 参数定义
    m1, m2 = sp.symbols('m1 m2', real=True, positive=True)
    l1 = sp.symbols('l1', real=True) # Yaw偏心
    l2 = sp.symbols('l2', real=True) # Pitch偏心
    g = sp.symbols('g', real=True, positive=True)

    I1xx, I1yy, I1zz = sp.symbols('I1xx I1yy I1zz', real=True, positive=True)
    I2xx, I2yy, I2zz = sp.symbols('I2xx I2yy I2zz', real=True, positive=True)
    
    I1 = sp.diag(I1xx, I1yy, I1zz)
    I2 = sp.diag(I2xx, I2yy, I2zz)

    # Yaw绕Z旋转
    R1 = sp.Matrix([
        [sp.cos(q1), -sp.sin(q1), 0],
        [sp.sin(q1),  sp.cos(q1), 0],
        [0,           0,          1]
    ])
    # Pitch绕局部Y旋转
    R2_1 = sp.Matrix([
        [ sp.cos(q2), 0, sp.sin(q2)],
        [ 0,          1, 0         ],
        [-sp.sin(q2), 0, sp.cos(q2)]
    ])
    R2 = R1 * R2_1
    print("R1 =", R1)
    print("R2 =", R2)

    # 质心位置计算
    p1 = sp.Matrix([0, 0, l1])  # Yaw轴质心仅在Z轴上有偏移
    p2 = sp.Matrix([0, 0, l1]) + R1 * sp.Matrix([0, 0, l1])
    # Pitch质心设为在自身的X轴有偏移l2，相对Yaw的原点为[0,0,H]，但为了简化直接只看最终位置
    p2_local = sp.Matrix([l2, 0, 0])
    p2 = sp.Matrix([0, 0, l1]) + R2 * p2_local

    v1 = sp.diff(p1, t)
    v2 = sp.diff(p2, t)

    # 局部角速度算子
    w1_world = sp.Matrix([0, 0, dq1])
    w2_world = w1_world + R1 * sp.Matrix([0, dq2, 0])

    w1_local = R1.T * w1_world
    w2_local = R2.T * w2_world

    K1 = 0.5 * m1 * sp.simplify((v1.T * v1)[0]) + 0.5 * sp.simplify((w1_local.T * I1 * w1_local)[0])
    K2 = 0.5 * m2 * sp.simplify((v2.T * v2)[0]) + 0.5 * sp.simplify((w2_local.T * I2 * w2_local)[0])
    K = sp.simplify(K1 + K2)
    print("Kinetic Energy K =", K)

    P1 = m1 * g * p1[2]
    P2 = m2 * g * p2[2]
    P = sp.simplify(P1 + P2)
    print("Potential Energy P =", P)

    # M矩阵
    M = sp.zeros(2, 2)
    for i in range(2):
        for j in range(2):
            M[i, j] = sp.simplify(sp.diff(sp.diff(K, dq[i]), dq[j]))
    
    print("\nM_11 =", M[0, 0])
    print("M_12 =", M[0, 1])
    print("M_21 =", M[1, 0])
    print("M_22 =", M[1, 1])

    # C矩阵: 利用Christoffel符号
    C = sp.zeros(2, 2)
    for k in range(2):
        for j in range(2):
            c_kj = 0
            for i in range(2):
                c_ijk = 0.5 * (sp.diff(M[k, j], q[i]) + sp.diff(M[k, i], q[j]) - sp.diff(M[i, j], q[k]))
                c_kj += c_ijk * dq[i]
            C[k, j] = sp.simplify(c_kj)
            
    print("\nC_11 =", C[0, 0])
    print("C_12 =", C[0, 1])
    print("C_21 =", C[1, 0])
    print("C_22 =", C[1, 1])

    # G向量
    G = sp.zeros(2, 1)
    for i in range(2):
        G[i, 0] = sp.simplify(sp.diff(P, q[i]))
        
    print("\ng_1 =", G[0, 0])
    print("g_2 =", G[1, 0])

if __name__ == "__main__":
    main()
