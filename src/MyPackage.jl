module MyPackage

using LinearAlgebra # ベクトル演算のために必要

"""
perform_MyPackage(F, tmax, x0, v0, h)

2階常微分方程式を4次のルンゲ＝クッタ法（RK4）で数値的に解く関数。

# 引数
- `F`: 力の関数。`F(x, v, t)` の形式で、加速度を返す関数。
- `tmax`: シミュレーションの最大時間。
- `x0`: 初期位置。
- `v0`: 初期速度。
- `h`: 時間ステップサイズ。

# 戻り値
- `x_final`: `tmax` における最終位置。
- `v_final`: `tmax` における最終速度。
"""
function perform_MyPackage(F, tmax, x0, v0, h)
    t = 0.0
    x = x0
    v = v0

    # 2階ODEを1階連立ODEに変換する関数
    # 状態ベクトル Y = [x, v]
    # Y' = [v, F(x,v,t)]
    # ここで F(x,v,t) は加速度 (d^2x/dt^2)
    function f_system(Y, t_current)
        current_x, current_v = Y[1], Y[2]
        acceleration = F(current_x, current_v, t_current)
        return [current_v, acceleration] # [dx/dt, dv/dt]
    end

    # シミュレーションループ
    while t < tmax
        # 現在の時間ステップで tmax を超えないように調整
        dt_actual = min(h, tmax - t)

        # RK4法の計算ステップ
        # k1 = f(Y_n, t_n)
        k1 = f_system([x, v], t)

        # k2 = f(Y_n + (dt_actual/2)k1, t_n + dt_actual/2)
        # ベクトル Y_n + (dt_actual/2)k1 を計算するために LinearAlgebra の + や * が便利
        k2 = f_system([x + (dt_actual/2) * k1[1], v + (dt_actual/2) * k1[2]], t + dt_actual/2)

        # k3 = f(Y_n + (dt_actual/2)k2, t_n + dt_actual/2)
        k3 = f_system([x + (dt_actual/2) * k2[1], v + (dt_actual/2) * k2[2]], t + dt_actual/2)

        # k4 = f(Y_n + dt_actual*k3, t_n + dt_actual)
        k4 = f_system([x + dt_actual * k3[1], v + dt_actual * k3[2]], t + dt_actual)

        # 次の時間ステップの状態を更新
        # Y_{n+1} = Y_n + (dt_actual/6) * (k1 + 2k2 + 2k3 + k4)
        x += (dt_actual/6) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
        v += (dt_actual/6) * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2])
        
        t += dt_actual
    end

    return x, v
end

end # module MyPackage
