module MyPackage

export perform_MyPackage

"""
perform_MyPackage(F, tmax, x0, a0, h)

数値的に時間発展を行い、時刻、位置、速度の配列を返します。

# 引数
- `F::Function`: (x, t) -> 加速度を返す関数
- `tmax::Float64`: シミュレーションの終了時刻 (> 0)
- `x0::Float64`: 初期位置
- `a0::Float64`: 初期速度
- `h::Float64`: 時間ステップ幅 (> 0)

# 返り値
- `(t, x, a)`:
  - `t::Vector{Float64}`: 0 から tmax までの時刻配列
  - `x::Vector{Float64}`: 各時刻での位置
  - `a::Vector{Float64}`: 各時刻での速度
"""
function perform_MyPackage(F::Function, tmax::Float64, x0::Float64, a0::Float64, h::Float64)
    # 入力チェック
    @assert tmax > 0 "tmax must be positive"
    @assert h > 0 "h must be positive"

    # ステップ数および配列の準備
    N = Int(floor(tmax / h)) + 1
    t = range(0.0, length=N, step=h)
    x = Vector{Float64}(undef, N)
    a = Vector{Float64}(undef, N)

    # 初期条件
    x[1] = x0
    a[1] = a0

    # 時間発展ループ
    for i in 1:N-1
        a[i+1] = a[i] + h * F(x[i], t[i])
        x[i+1] = x[i] + h * a[i+1]
    end

    return t, x, a
end

end  # module
