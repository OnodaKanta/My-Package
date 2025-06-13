using MyPackage
import MyPackage: perform_MyPackage
using Test

@testset "MyPackage.jl" begin
    # 等速度運動のテスト
    @testset "Uniform motion" begin
        F(x, t) = 0.0
        tmax, x0, a0, h = 1.0, 0.0, 1.0, 1e-3

        t, x, a = perform_MyPackage(F, tmax, x0, a0, h)
        x_final = x[end]
        a_final = a[end]

        expected_x = x0 + a0 * tmax
        expected_a = a0

        @test isapprox(x_final, expected_x, rtol=1e-10)
        @test isapprox(a_final, expected_a, rtol=1e-10)
    end

    # 等加速度運動のテスト
    @testset "Uniform acceleration motion" begin
        F(x, t) = 1.0
        tmax, x0, a0, h = 1.0, 0.0, 0.0, 1e-4

        t, x, a = perform_MyPackage(F, tmax, x0, a0, h)
        x_final = x[end]
        a_final = a[end]

        expected_x = x0 + a0 * tmax + 0.5 * F(0,0) * tmax^2
        expected_a = a0 + F(0,0) * tmax

        @test isapprox(x_final, expected_x, rtol=1e-3)
        @test isapprox(a_final, expected_a, rtol=1e-3)
    end

    # バネの運動のテスト
    @testset "Spring motion" begin
        k = 1.0
        F(x, t) = -k * x
        tmax, x0, a0, h = 2π, 1.0, 0.0, 1e-4

        t, x, a = perform_MyPackage(F, tmax, x0, a0, h)
        x_final = x[end]
        a_final = a[end]

        ω = √k
        expected_x = x0 * cos(ω * tmax)
        expected_a = -x0 * ω * sin(ω * tmax)

        @test isapprox(x_final, expected_x, atol=1e-2)
        @test isapprox(a_final, expected_a, atol=1e-2)
    end
end
