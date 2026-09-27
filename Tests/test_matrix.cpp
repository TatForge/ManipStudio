#include "gtest/gtest.h"
#include "my_math.h"

#include <utility>
#include <vector>

using namespace RoboMath;

TEST(Matrix, DefaultConstructorIsEmpty)
{
    Matrix m;
    EXPECT_EQ(m.GetRowsCount(), 0);
    EXPECT_EQ(m.GetColsCount(), 0);
}

TEST(Matrix, SizedConstructorReadWrite)
{
    Matrix m(2, 3);
    EXPECT_EQ(m.GetRowsCount(), 2);
    EXPECT_EQ(m.GetColsCount(), 3);
    m(0, 0) = 1;
    m(1, 2) = 9;
    EXPECT_DOUBLE_EQ(m(0, 0), 1);
    EXPECT_DOUBLE_EQ(m(1, 2), 9);
}

TEST(Matrix, NestedInitializerListConstructor)
{
    Matrix m{{1, 2, 3}, {4, 5, 6}};
    EXPECT_EQ(m.GetRowsCount(), 2);
    EXPECT_EQ(m.GetColsCount(), 3);
    EXPECT_DOUBLE_EQ(m.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(m.at(1, 2), 6);
}

TEST(Matrix, CopyConstructorIsDeepCopy)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b(a);
    b(0, 0) = 99;
    EXPECT_DOUBLE_EQ(a.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(b.at(0, 0), 99);
}

TEST(Matrix, MoveConstructorTransfersData)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b(std::move(a));
    EXPECT_DOUBLE_EQ(b.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(b.at(1, 1), 4);
}

TEST(Matrix, Size)
{
    Matrix m(2, 3);
    EXPECT_EQ(m.Size(), sizeof(scalar) * 6);
}

TEST(Matrix, EqualityOperator)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b{{1, 2}, {3, 4}};
    Matrix c{{1, 2}, {3, 5}};
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

TEST(Matrix, CopyAssignmentIsDeepCopy)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b{{0, 0}, {0, 0}};
    b = a;
    b(0, 0) = 99;
    EXPECT_DOUBLE_EQ(a.at(0, 0), 1);
}

TEST(Matrix, AddDiagonal)
{
    Matrix m{{1, 2}, {3, 4}};
    m.AddDiagonal(10);
    EXPECT_DOUBLE_EQ(m.at(0, 0), 11);
    EXPECT_DOUBLE_EQ(m.at(1, 1), 14);
    EXPECT_DOUBLE_EQ(m.at(0, 1), 2);
    EXPECT_DOUBLE_EQ(m.at(1, 0), 3);
}

TEST(Matrix, PlusMatrixAddsInPlace)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b{{5, 6}, {7, 8}};
    a.PlusMatrix(b);
    EXPECT_TRUE(a == Matrix({{6, 8}, {10, 12}}));
}

TEST(Matrix, MakeIdentity)
{
    Matrix m(3, 3);
    m.MakeIdentity();
    EXPECT_TRUE(m == Matrix({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));
}

TEST(Matrix, MakeZero)
{
    Matrix m{{1, 2}, {3, 4}};
    m.MakeZero();
    EXPECT_TRUE(m == Matrix({{0, 0}, {0, 0}}));
}

TEST(Matrix, Transpose)
{
    Matrix m{{1, 2, 3}, {4, 5, 6}};
    m.Transpose();
    EXPECT_EQ(m.GetRowsCount(), 3);
    EXPECT_EQ(m.GetColsCount(), 2);
    EXPECT_TRUE(m == Matrix({{1, 4}, {2, 5}, {3, 6}}));
}

TEST(Matrix, SetSubmatrix)
{
    Matrix m(3, 3);
    m.MakeZero();
    Matrix sub{{1, 2}, {3, 4}};
    m.SetSubmatrix(sub, 1, 1);
    EXPECT_TRUE(m == Matrix({{0, 0, 0}, {0, 1, 2}, {0, 3, 4}}));
}

TEST(Matrix, SetSubColumnFromVector3)
{
    Matrix m(4, 2);
    m.MakeZero();
    m.SetSubColumn(Vector3(1, 2, 3), 0, 0);
    EXPECT_DOUBLE_EQ(m.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(m.at(1, 0), 2);
    EXPECT_DOUBLE_EQ(m.at(2, 0), 3);
    EXPECT_DOUBLE_EQ(m.at(3, 0), 0);
}

// SetColumn has two overloads with inconsistent column-index conventions:
// the Vector overload is 1-based (`ColNumber - 1`), the Vector3 overload is
// 0-based. Calling both with the same ColNumber therefore targets different
// columns -- this test documents that inconsistency safely (using only
// documented-valid indices, never the underflowing ColNumber=0 case of the
// Vector overload).
TEST(Matrix, SetColumnOverloadsDisagreeOnIndexConvention)
{
    Matrix m1(3, 3);
    m1.MakeZero();
    m1.SetColumn(Vector({1, 2, 3}), 1); // 1-based -> writes column 0

    Matrix m2(3, 3);
    m2.MakeZero();
    m2.SetColumn(Vector3(1, 2, 3), 1); // 0-based -> writes column 1

    EXPECT_DOUBLE_EQ(m1.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(m2.at(0, 0), 0);
    EXPECT_DOUBLE_EQ(m2.at(0, 1), 1);
}

TEST(Matrix, MultuplyTwoArgComputesProduct)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b{{2, 0}, {1, 2}};
    Matrix result = a.Multuply(a, b);
    EXPECT_TRUE(result == Matrix({{4, 4}, {10, 8}}));
}

TEST(Matrix, OperatorStarComputesProduct)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b{{2, 0}, {1, 2}};
    Matrix result = a * b;
    EXPECT_TRUE(result == Matrix({{4, 4}, {10, 8}}));
}

// Documents a real bug: the single-argument Multiply(matrix2, ret_matrix)
// ignores *this entirely and computes matrix2 * matrix2 instead of
// (*this) * matrix2 (my_math.cpp uses matrix2.at(...) where this->at(...)
// was intended). Expected to FAIL until fixed.
TEST(Matrix, MultiplySingleArgComputesThisTimesArg)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b{{2, 0}, {1, 2}};
    Matrix expected = a * b; // trusted-correct reference
    Matrix result(2, 2);
    a.Multiply(b, &result);
    EXPECT_TRUE(result == expected);
}

TEST(Matrix, MultiplyTransposeMatrix)
{
    Matrix m{{1, 2, 3}, {5, 6, 7}, {7, 8, 9}, {4, 3, 2}, {7, 6, 5}};
    Vector v{0.1, 0.5, 0.9};
    EXPECT_EQ(m.MultiplyTranspose(m), Matrix({{14, 38, 50, 16, 34}, {38, 110, 146, 52, 106}, {50, 146, 194, 70, 142}, {16, 52, 70, 29, 56}, {34, 106, 142, 56, 110}}));
    EXPECT_EQ(m.MultiplyTranspose(v), Vector({3.8, 9.8, 12.8, 3.7, 8.2}));
}

TEST(Matrix, OperatorStarVector)
{
    Matrix identity(3, 3);
    identity.MakeIdentity();
    Vector v({1, 2, 3});
    Vector result = identity * v;
    EXPECT_TRUE(result == v);
}

TEST(Matrix, OperatorStarScalar)
{
    Matrix m{{1, 2}, {3, 4}};
    Matrix result = m * 2.0;
    EXPECT_TRUE(result == Matrix({{2, 4}, {6, 8}}));
}

// Documents a real bug: operator+ writes the right-hand operand's data into
// *this's own storage (instead of into the freshly-constructed result), so
// it both mutates the left operand and returns an unpopulated result.
// Expected to FAIL until fixed.
TEST(Matrix, OperatorPlusAddsCorrectly)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b{{5, 6}, {7, 8}};
    Matrix result = a + b;
    EXPECT_TRUE(result == Matrix({{6, 8}, {10, 12}}));
}

TEST(Matrix, OperatorPlusDoesNotMutateLeftOperand)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix a_copy = a;
    Matrix b{{5, 6}, {7, 8}};
    Matrix result = a + b;
    EXPECT_TRUE(a == a_copy);
}

TEST(Matrix, SolveGaussAndGetTheRootsSolveLinearSystem)
{
    // 2x + y = 5
    //  x - y = 1  -> x = 2, y = 1
    Matrix m{{2, 1, 5}, {1, -1, 1}};
    m.SolveGauss();
    Vector roots(2);
    m.GetTheRoots(roots);
    EXPECT_NEAR(roots.at(0), 2.0, 1e-6);
    EXPECT_NEAR(roots.at(1), 1.0, 1e-6);
}

TEST(Matrix, MatrixCoutDoesNotCrash)
{
    Matrix m{{1, 2}, {3, 4}};
    testing::internal::CaptureStdout();
    m.MatrixCout("test");
    testing::internal::GetCapturedStdout();
}

namespace {

// Solves A x = b via the project's SolveGauss + GetTheRoots on [A | b].
Vector SolveWithGauss(const std::vector<std::vector<double>> &A, const std::vector<double> &b)
{
    const int n = static_cast<int>(A.size());
    Matrix aug(n, n + 1);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
            aug(i, j) = A[i][j];
        aug(i, n) = b[i];
    }
    aug.SolveGauss();
    Vector x(n);
    aug.GetTheRoots(x);
    return x;
}

} // namespace

// Exactly the shape of system the damped-least-squares IK solves every
// iteration: (J J^T + lambda^2 I) x = e, a symmetric positive definite 6x6.
TEST(Matrix, SolveGaussSolvesDampedLeastSquaresSystem)
{
    Matrix J{{0.1, -0.4, 0.3, 0.0, 0.2, 0.05},
             {0.6, 0.1, -0.2, 0.3, 0.0, 0.1},
             {-0.2, 0.5, 0.4, 0.1, -0.3, 0.0},
             {0.0, 0.0, 1.0, 0.2, 0.0, 0.7},
             {0.3, 1.0, 0.0, -0.5, 0.9, 0.0},
             {1.0, 0.0, 0.0, 0.4, 0.1, 0.7}};
    Matrix Jt = J;
    Jt.Transpose();
    Matrix U = J * Jt;
    U.AddDiagonal(damping_constant_sq);
    std::vector<double> x_true{0.3, -1.2, 0.8, 2.0, -0.5, 0.1};
    std::vector<std::vector<double>> A(6, std::vector<double>(6));
    std::vector<double> b(6, 0);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            A[i][j] = U(i, j);
            b[i] += U(i, j) * x_true[j];
        }
    Vector x = SolveWithGauss(A, b);
    for (int i = 0; i < 6; i++)
        EXPECT_NEAR(x.at(i), x_true[i], 1e-9) << "i = " << i;
}

TEST(Matrix, SolveGaussPivotsOnFirstColumn)
{
    // First pivot is 0 -> rows must be swapped.
    Vector x = SolveWithGauss({{0, 2, 1}, {1, 1, 1}, {2, 1, 3}}, {7, 6, 13});
    EXPECT_NEAR(x.at(0), 1, 1e-9);
    EXPECT_NEAR(x.at(1), 2, 1e-9);
    EXPECT_NEAR(x.at(2), 3, 1e-9);
}

// Documents a real bug: SolveGauss only searches for a pivot in column 0 of
// the *original* rows (`m_data_ptr + m_cols * j`) instead of the current
// column below the current diagonal element. After the first elimination
// step that column is all zeros, so no row swap ever happens again and a
// zero on the diagonal at step 2+ produces a division by zero (NaN/inf
// roots) even though the system is perfectly regular.
// x = (1, 2, 3); after eliminating column 0 the (1,1) element becomes 0.
TEST(Matrix, SolveGaussPivotsInLaterColumns)
{
    Vector x = SolveWithGauss({{1, 1, 1}, {1, 1, 2}, {1, 2, 3}}, {6, 9, 14});
    EXPECT_NEAR(x.at(0), 1, 1e-9);
    EXPECT_NEAR(x.at(1), 2, 1e-9);
    EXPECT_NEAR(x.at(2), 3, 1e-9);
}

TEST(Matrix, NonSquareProductHasCorrectShapeAndValues)
{
    Matrix a{{1, 2, 3}, {4, 5, 6}};         // 2x3
    Matrix b{{7, 8}, {9, 10}, {11, 12}};    // 3x2
    Matrix ab = a * b;
    ASSERT_EQ(ab.GetRowsCount(), 2);
    ASSERT_EQ(ab.GetColsCount(), 2);
    EXPECT_TRUE(ab == Matrix({{58, 64}, {139, 154}}));
    Matrix ba = b * a;
    ASSERT_EQ(ba.GetRowsCount(), 3);
    ASSERT_EQ(ba.GetColsCount(), 3);
    EXPECT_TRUE(ba == Matrix({{39, 54, 69}, {49, 68, 87}, {59, 82, 105}}));
}

TEST(Matrix, NonSquareTransposeRoundTrip)
{
    Matrix a{{1, 2, 3}, {4, 5, 6}};
    Matrix t = a;
    t.Transpose();
    ASSERT_EQ(t.GetRowsCount(), 3);
    ASSERT_EQ(t.GetColsCount(), 2);
    EXPECT_DOUBLE_EQ(t(2, 1), 6);
    EXPECT_DOUBLE_EQ(t(0, 1), 4);
    t.Transpose();
    EXPECT_TRUE(t == a);
}

// Despite the name, Matrix::MultiplyTranspose(Vector&) computes M * v
// (row-by-column), consistent with Matrix4x4::MultiplyTranspose(Vector).
TEST(Matrix, MultiplyTransposeVectorComputesMatrixTimesVector)
{
    Matrix m{{1, 2, 3}, {4, 5, 6}};
    Vector v({1, 0, -1});
    Vector r = m.MultiplyTranspose(v);
    ASSERT_EQ(r.GetLength(), 2);
    EXPECT_DOUBLE_EQ(r.at(0), -2);
    EXPECT_DOUBLE_EQ(r.at(1), -2);
}

TEST(Matrix, MoveAssignmentTransfersData)
{
    Matrix a{{1, 2}, {3, 4}};
    Matrix b;
    b = Matrix{{5, 6}, {7, 8}};
    EXPECT_TRUE(b == Matrix({{5, 6}, {7, 8}}));
    a = std::move(b);
    EXPECT_TRUE(a == Matrix({{5, 6}, {7, 8}}));
    EXPECT_EQ(b.GetDataPtr(), nullptr);
}

TEST(Matrix, SizeReturnsBytes)
{
    Matrix m(3, 5);
    EXPECT_EQ(m.Size(), sizeof(scalar) * 15);
}

// Documents a real bug: the bounds asserts in Matrix::operator()/at use
// `col <= m_cols` / `row <= m_rows`, so an index equal to the dimension (one
// past the end) is accepted silently instead of tripping the assert.
TEST(MatrixDeathTest, AccessOnePastLastColumnAsserts)
{
    EXPECT_DEATH(
        {
            Matrix m(2, 2);
            volatile scalar v = m(0, 2);
            (void)v;
        },
        "");
}

TEST(MatrixOfMatrix6x6, ElementsStartAsIdentity4x4)
{
    MatrixOfMatrix6x6 mm;
    Matrix identity(4, 4);
    identity.MakeIdentity();
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            ASSERT_EQ(mm(i, j).GetRowsCount(), 4);
            ASSERT_EQ(mm(i, j).GetColsCount(), 4);
            EXPECT_TRUE(mm(i, j) == identity) << i << "," << j;
        }
}

TEST(MatrixOfMatrix6x6, ElementsAreIndependent)
{
    MatrixOfMatrix6x6 mm;
    mm(2, 3)(0, 0) = 42;
    EXPECT_DOUBLE_EQ(mm(2, 3)(0, 0), 42);
    EXPECT_DOUBLE_EQ(mm(3, 2)(0, 0), 1);
    EXPECT_DOUBLE_EQ(mm(2, 4)(0, 0), 1);
}
