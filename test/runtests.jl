using MyPackage
import MyPackage: perform_MyPackage
using Test

@testset "MyPackage.jl" begin

    @testset "Uniform motion" begin
        F(x, t) = 0.0
        tmax, x0, a0, h = 1.0, 0.0, 1.0, 1e-3

        # 時系列データを受け取る
        t, x, a = perform_MyPackage(F, tmax, x0, a0, h)
        
        # デバッグ表示を入れる
        @show x[1], a[1]          # 初期値
        @show x[end], a[end]      # 最終ステップの値

        # アサーション
        @test isapprox(x[end], x0 + a0 * tmax, rtol=1e-10)
        @test isapprox(a[end], a0,             rtol=1e-10)
    end

    @testset "Uniform acceleration motion" begin
        F(x, t) = 1.0
        tmax, x0, a0, h = 1.0, 0.0, 0.0, 1e-4

        t, x, a = perform_MyPackage(F, tmax, x0, a0, h)
        
        @show x[end], a[end]

        @test isapprox(x[end], x0 + 0.5 * F(0,0) * tmax^2, rtol=1e-3)
        @test isapprox(a[end], F(0,0) * tmax,             rtol=1e-3)
    end

    @testset "Spring motion" begin
        k = 1.0
        F(x, t) = -k * x
        tmax, x0, a0, h = 2π, 1.0, 0.0, 1e-4

        t, x, a = perform_MyPackage(F, tmax, x0, a0, h)

        @show x[end], a[end]

        ω = √k
        @test isapprox(x[end], x0 * cos(ω * tmax), atol=1e-2)
        @test isapprox(a[end], -x0 * ω * sin(ω * tmax), atol=1e-2)
    end

end
