// #pragma once
#include<bits/stdc++.h>
// #include "rational.h"

// моя реализация STATIC_ASSERT
#define STATIC_ASSERT(x) [[maybe_unused]] typedef int a[(x) ? 1 : -1]

// я не совсем понял в чем проблема с табуляцией, но на всякий случай поставил в vs code все пробелы

//compile

namespace CheckPrime {
    template<unsigned N, unsigned P>
    struct CalculatePrime {
        static const int prime = (CalculatePrime<N - 1, P>::prime < (int)(P % N != 0)) ? CalculatePrime<N - 1, P>::prime : (P % N != 0); // ugly hack to avoid warning
    };

    template<unsigned P>
    struct CalculatePrime<1, P> {
        static const int prime = 1;
    };
    
    template<unsigned N>
    struct IsPrime {
        static const bool prime = CalculatePrime<(int)sqrt(N), N>::prime;
    };
}

//helper method

static int gcd(int a, int b, int &x, int &y) {
    if (!a) {
        x = 0;
        y = 1;
        return b;
    }
    int x_, y_;
    int gcd_ = gcd(b % a, a, x_, y_);
    x = y_ - (b / a) * x_;
    y = x_; 
    return gcd_;
}

template<typename T>

static T abs(const T &x) {
    return (x < 0) ? -x : x;
}

template<unsigned int MOD>
class Finite {
  private:
    long long _n;
    void recover();
  public:
    Finite();
    Finite(const long long _n);
    ~Finite();

    Finite& operator++();
    Finite& operator+=(const Finite &other);
    Finite& operator-=(const Finite &other);
    Finite& operator*=(const Finite &other);
    Finite operator+(const Finite &other) const;
    Finite operator-(const Finite &other) const;
    Finite operator*(const Finite &other) const;
    Finite& operator=(const Finite &other);
    Finite& operator/=(const Finite &other);
    Finite operator/(const Finite &other) const;
    bool operator==(const Finite &other) const;
    bool operator!=(const Finite &other) const;
    Finite pow(const long long k) const;
    Finite inverse() const;
    template<unsigned int OTHER_MOD>
    friend std::ostream& operator<<(std::ostream &stream, const Finite<OTHER_MOD> &x); 
};

template<unsigned int MOD>
void Finite<MOD>::recover() {
    if (_n < 0) {
        _n = MOD - abs(_n) % MOD;
    }
    _n %= MOD;
}

template<unsigned int MOD>
Finite<MOD> Finite<MOD>::pow(const long long deg) const {
    if (deg == 0) {
        return Finite<MOD>(1);
    }
    if (deg % 2 == 0) {
        Finite<MOD> ans = this->pow(deg / 2);
        return ans * ans;
    }
    return Finite<MOD>((*this) * this->pow(deg - 1));
}


template<unsigned int MOD>
Finite<MOD>::Finite(const long long n_) { 
    _n = n_;
    recover();    
}

template<unsigned int MOD>
Finite<MOD>::Finite() { 
    _n = 0ll;    
}


template<unsigned int MOD>
Finite<MOD>::~Finite(){
    
}

template<unsigned int MOD>
Finite<MOD> Finite<MOD>::inverse() const {
    const int p = CheckPrime::IsPrime<MOD>::prime;
    STATIC_ASSERT(p == 1); 
    int cmp, inversed;
    gcd(this->_n, MOD, inversed, cmp);
    return Finite<MOD>(inversed);
}

template<unsigned int MOD>
Finite<MOD>& Finite<MOD>::operator++() { 
    _n++;
    return *this;
}

template<unsigned int MOD>
Finite<MOD>& Finite<MOD>::operator+=(const Finite &other) {
    _n += other._n;
    recover();
    return *this;
}

template<unsigned int MOD>
Finite<MOD>& Finite<MOD>::operator-=(const Finite &other) {
    _n -= other._n;
    recover();
    return *this;
}

template<unsigned int MOD>Finite<MOD>& Finite<MOD>::operator*=(const Finite &other) {
    _n *= other._n;
    recover();
    return *this;
}

template<unsigned int MOD>
Finite<MOD> Finite<MOD>::operator+(const Finite &other) const {
    Finite<MOD> result(this->_n);
    result += other._n;
    result.recover();
    return result;
}

template<unsigned int MOD>
Finite<MOD> Finite<MOD>::operator-(const Finite &other) const {
    Finite<MOD> result(this->_n);
    result -= other._n;
    result.recover();
    return result;
}

template<unsigned int MOD>
Finite<MOD> Finite<MOD>::operator*(const Finite &other) const {
    Finite<MOD> result(this->_n);
    result *= other._n;
    return result;
}

template<unsigned int MOD>
Finite<MOD>& Finite<MOD>::operator=(const Finite &other) {
    this->_n = other._n;
    recover();
    return *this;
}

template<unsigned int MOD>
Finite<MOD>& Finite<MOD>::operator/=(const Finite &other) {
    *this *= other.inverse();
    return *this;
}

template<unsigned int MOD>
Finite<MOD> Finite<MOD>::operator/(const Finite &other) const {
    Finite<MOD> ans(_n);
    return ans /= other;
}

template<unsigned int MOD>
bool Finite<MOD>::operator==(const Finite &other) const { 
    return (_n == other._n);
}

template<unsigned int MOD>
bool Finite<MOD>::operator!=(const Finite &other) const { 
    return (_n != other._n);
}

template<unsigned int OTHER_MOD>
std::ostream& operator<<(std::ostream &stream, const Finite<OTHER_MOD> &x) {
    return (stream << x._n);
}

// -------------------------------------------
// matrix

// compile time calculations to use templates
template<unsigned N>
struct Deg2 {
    static const unsigned long long prev_ans = ((N % 2 == 1) ? (Deg2<N - 1>::deg) : (Deg2<N / 2>::deg));
    static const unsigned long long deg = (N > prev_ans) ? 2 * prev_ans : prev_ans;
};

template<>
struct Deg2<1> {
    static const unsigned long long prev_ans = 1;
    static const unsigned long long deg = 0;
};

template<unsigned int M, unsigned int N, typename Field>
class BaseMatrix {
  protected:
    std::vector<std::vector<Field>> _matrix;      
    void _addRow(const unsigned to, const unsigned row);
    void _multRow(const unsigned row, const Field& scalar);
    template<unsigned int K>
    BaseMatrix<M, K, Field> _multiply(const BaseMatrix<N, K, Field> &other) const;    
  public:
    Field gaussMethod();
    BaseMatrix();
    BaseMatrix(const Field &C);
    BaseMatrix(const std::vector<std::vector<Field> >& cmp);
    ~BaseMatrix();
      
    BaseMatrix& operator=(const BaseMatrix &other); 

    std::vector<Field>& operator[](const int &ind);    
    std::vector<Field> operator[](const int &ind) const;
    
    BaseMatrix& operator+=(const BaseMatrix &other);
    BaseMatrix& operator-=(const BaseMatrix &other);
    BaseMatrix operator+(const BaseMatrix &other) const;
    BaseMatrix operator-(const BaseMatrix &other) const;
    bool operator==(const BaseMatrix &other) const;
    bool operator!=(const BaseMatrix &other) const;
         
    BaseMatrix operator*(const Field &cmp) const;
    BaseMatrix& operator*=(const Field &cmp);
    template<unsigned int OTHER_M, unsigned int OTHER_N, typename OTHER_Field>
    friend BaseMatrix<OTHER_M, OTHER_N, OTHER_Field> operator*(const OTHER_Field &cmp, const BaseMatrix<OTHER_M, OTHER_N, OTHER_Field> &m);
    

    template<unsigned int K>
    BaseMatrix<M, K, Field> operator*(const BaseMatrix<N, K, Field> &other) const;
    
    Field det() const; 
    Field trace() const;
    unsigned rank() const;
    BaseMatrix<N, M, Field> transposed() const;
       
    BaseMatrix inverted() const;
    BaseMatrix<M, N, Field>& invert();          

    std::vector<Field> getRow(const unsigned r);
    std::vector<Field> getColumn(const unsigned c);

    template<unsigned int OTHER_M, unsigned int OTHER_N, typename OTHER_Field>
    friend std::ostream& operator<<(std::ostream &stream, const BaseMatrix<OTHER_M, OTHER_N, OTHER_Field> &other);
};

// helper method

template<unsigned M, unsigned N, typename Field>
void BaseMatrix<M, N, Field>::_addRow(unsigned to, unsigned row) {
    // return;
    for (unsigned i = 0; i < N; ++i) {
        // continue; 
        _matrix[to][i] = _matrix[row][i];
    }
}

template<unsigned M, unsigned N, typename Field>
void BaseMatrix<M, N, Field>::_multRow(const unsigned row, const Field& scalar) {
    for (unsigned i = 0; i < N; ++i) {
        _matrix[row][i] *= scalar;
    }
}

template<unsigned M, unsigned N, typename Field>
template<unsigned K>
BaseMatrix<M, K, Field> BaseMatrix<M, N, Field>::_multiply(const  BaseMatrix<N, K, Field> &other) const {
    BaseMatrix<M, K, Field> res(0);
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned colum = 0; colum < K; ++colum) {
            for (unsigned  num = 0; num < N; ++num) {
                res[row][colum] += this->_matrix[row][num] * other[num][colum]; 
            }
        }
    }
    return res;
}


// constructors

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M, N, Field>::BaseMatrix() {
    _matrix.resize(M, std::vector<Field> (N));
}

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M, N, Field>::BaseMatrix(const Field &scalar) {
    _matrix.resize(M, std::vector<Field> (N, scalar));
}

template<unsigned M, unsigned N, typename Field>
BaseMatrix<M, N, Field>::BaseMatrix(const std::vector<std::vector<Field> >& matrix_) {
    if (matrix_.size() != M || matrix_[0].size() != N) {
        return;
    }
    _matrix.resize(M, std::vector<Field> (N, 0));
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < N; ++column) {
            _matrix[row][column] = matrix_[row][column];
        }
    }
}

// destructor
template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M, N, Field>::~BaseMatrix() {}

// operator =

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M, N, Field>& BaseMatrix<M, N, Field>::operator=(const BaseMatrix &other) { 
    for (unsigned i = 0; i < M; ++i) {
        for (unsigned j = 0; j < N; ++j) {
            _matrix[i][j] = other._matrix[i][j];
        }
    }
    return *this;
} 

// operators ==, !=

template<unsigned int M, unsigned int N, typename Field>
bool BaseMatrix<M, N, Field>::operator==(const BaseMatrix &other) const {
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned  column = 0; column < N; ++column) {
            if (_matrix[row][column] != other._matrix[row][column])
                return 0;
        }
    }
    return 1;
}
    
template<unsigned int M, unsigned int N, typename Field>
bool BaseMatrix<M, N, Field>::operator!=(const BaseMatrix &other) const {
    return !(*this == other);
}


// operators +=, -=, +, -
template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M,N,Field>& BaseMatrix<M, N, Field>::operator+=(const BaseMatrix &other) {
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < N; ++column) {
            _matrix[row][column] += other._matrix[row][column];
        }
    }
    return *this;
}

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M,N,Field>& BaseMatrix<M, N, Field>::operator-=(const BaseMatrix &other) {
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < N; ++column) {
            _matrix[row][column] -= other._matrix[row][column];
        }
    }
    return *this;
}

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M,N,Field> BaseMatrix<M, N, Field>::operator+(const BaseMatrix &other) const {
    BaseMatrix<M, N, Field> res(0);
    res += *this;
    res += other; 
    return res;
}

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M,N,Field> BaseMatrix<M, N, Field>::operator-(const BaseMatrix &other) const {
    BaseMatrix<M, N, Field> res(0);
    res += *this;
    res -= other; 
    return res;
}

// operators *, *= whit scalar

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M, N, Field>& BaseMatrix<M, N, Field>::operator*=(const Field &scalar) {
    for (unsigned i = 0; i < M; ++i) {
        for (unsigned j = 0; j < N; ++j) {
            _matrix[i][j] *= scalar;
        }
    }
    return *this;
} 

template<unsigned int M, unsigned int N, typename Field>
BaseMatrix<M, N, Field> BaseMatrix<M, N, Field>::operator*(const Field &scalar) const {
    BaseMatrix<M, N, Field> res = *this;
    res *= scalar;
    return res;
} 

template<unsigned int OTHER_M, unsigned int OTHER_N, typename OTHER_Field>
BaseMatrix<OTHER_M, OTHER_N, OTHER_Field> operator*(const OTHER_Field &scalar, const BaseMatrix<OTHER_M, OTHER_N, OTHER_Field> &other) {
    return (other * scalar);
}

// operator * with matrix
// Strassen algorithm
template<unsigned M, unsigned N, typename Field>
template<unsigned K>
BaseMatrix<M, K, Field> BaseMatrix<M, N, Field>::operator*(const  BaseMatrix<N, K, Field> &other) const {
    const int max_size = std::max(M, (std::max(N, K)));
    if (max_size < 64) {
        return this->_multiply(other);
    }
    // compile time calculations
    const unsigned new_size = (Deg2<max_size>::deg > 2) ? Deg2<max_size>::deg : 2;
    BaseMatrix<new_size / 2, new_size / 2, Field> A[2][2], B[2][2], P[7], C[2][2];
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < N; ++column) {
            unsigned q_row = (row < new_size / 2) ? 0 : 1;
            unsigned q_col = (column < new_size / 2) ? 0 : 1;
            
            A[q_row][q_col][row % (new_size / 2)][column % (new_size / 2)] = this->_matrix[row][column];
        }
    }

    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < N; ++column) {
            unsigned q_row = (row < new_size / 2) ? 0 : 1;
            unsigned q_col = (column < new_size / 2) ? 0 : 1;
            
            B[q_row][q_col][row % (new_size / 2)][column % (new_size / 2)] = other[row][column];
        }
    }

    P[0] = (A[0][0] + A[1][1]) * (B[0][0] + B[1][1]);
    P[1] = (A[1][0] + A[1][1]) * B[0][0];
    P[2] = A[0][0] * (B[0][1] - B[1][1]);   
    P[3] = A[1][1] * (B[1][0] - B[0][0]);
    P[4] = (A[0][0] + A[0][1]) * B[1][1];
    P[5] = (A[1][0] - A[0][0]) * (B[0][0] + B[0][1]);
    P[6] = (A[0][1] - A[1][1]) * (B[1][0] + B[1][1]);

    C[0][0] = P[0] + P[3] - P[4] + P[6];
    C[0][1] = P[2] + P[4];
    C[1][0] = P[1] + P[3];
    C[1][1] = P[0] - P[1] + P[2] + P[5];
    
    BaseMatrix<M, K, Field> res(0); 
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < K; ++column) {
            unsigned q_row = (row < new_size / 2) ? 0 : 1;
            unsigned q_col = (column < new_size / 2) ? 0 : 1;

            res[row][column] = C[q_row][q_col][row % (new_size / 2)][column % (new_size / 2)]; 
        }
    }
    return res;
}

template<unsigned M, unsigned N, typename Field>
Field BaseMatrix<M, N, Field>::gaussMethod() {
    const unsigned MIN = std::min(N, M);
    Field det_ = 1;

    for (unsigned column = 0; column < MIN; ++column) {
        int is_zero = 0;
        if (_matrix[column][column] == 0) {
            is_zero = 1;
            for (unsigned row = column + 1; row < M; ++row) {
                if (_matrix[row][column] != 0) {
                    _addRow(column, row);
                    is_zero = 0;
                    break;
                }
            }
        }
        if (is_zero) {
            continue;
        }
        det_ *= _matrix[column][column];
        _multRow(column, Field(1) / _matrix[column][column]);
        for (unsigned row = column + 1; row < M; ++row) {
            if (_matrix[row][column] == 0) {
                continue;
            }
            Field scalar = _matrix[row][column];
            for (unsigned i = 0; i < N; ++i) {
                _matrix[row][i] -= _matrix[column][i] * scalar;
            } 
        }
    }

    int rank = 0;
    for (unsigned i = 0; i < MIN; ++i) {
        if (_matrix[i][i] != 0) {
            rank++;
        }
    }

    for (int column = rank - 1; column >= 0; --column) {
        for (int row = column - 1; row >= 0; --row) {
            if (_matrix[row][column] != 0) {
                Field scalar = _matrix[row][column];
                for (unsigned i = 0; i < N; ++i) {
                    _matrix[row][i] -= _matrix[column][i] * scalar;
                }
            }
        }
    }
    return det_;         
}


// det, trace, rank

template<unsigned M, unsigned N, typename Field>
Field BaseMatrix<M, N, Field>::trace() const {
    STATIC_ASSERT(N == M);
    Field trace_ = 0;
    for (unsigned i = 0; i < N; ++i) 
        trace_ += _matrix[i][i];
    return trace_;
}

template<unsigned M, unsigned N, typename Field>
Field BaseMatrix<M, N, Field>::det() const {
    STATIC_ASSERT(N == M);          
    BaseMatrix<N, M, Field> copy_ = *this;    
    return copy_.gaussMethod();
}

template<unsigned M, unsigned N, typename Field>
unsigned BaseMatrix<M, N, Field>::rank() const {
    BaseMatrix<M, N, Field> copy_ = *this;
    copy_.gaussMethod(); 
    unsigned rank = 0;
    const unsigned MIN = std::min(M, N);
    unsigned row = 0;
    unsigned column = 0;
    for (; row < MIN && column < N; ++row) {
        while (column < N && copy_._matrix[row][column] == 0) {
            column++;          
        }
        if (column >= N) {
            break;
        }
        rank++;
    }
    return rank; 
}

// transposed

template<unsigned M, unsigned N, typename Field>
BaseMatrix<N, M, Field> BaseMatrix<M, N, Field>::transposed() const {
    BaseMatrix<N, M, Field> transposed_;
    // std::cout << *this << '\n';
    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < N; ++column) {
            transposed_[column][row] = _matrix[row][column];
        }
    }
    // std::cout << "------------\n";
    // for (int i = 0; i < M; ++i) {
    //     for (int j = 0; j  < N; ++j) {
    //         std::cout << transposed_[i][j] << ' ';
    //     }
    //     std::cout << '\n';
    // }
    //  std::cout << transposed_ << '\n';
    return transposed_;
}


// operator []

// lvalue
// maybe better to use proxy
template<unsigned M, unsigned N, typename Field>
std::vector<Field>& BaseMatrix<M, N, Field>::operator[](const int &ind) {
    return _matrix[ind];
}

template<unsigned M, unsigned N, typename Field>
std::vector<Field> BaseMatrix<M, N, Field>::operator[](const int &ind) const {
    return _matrix[ind];
}


// invert

template<unsigned M, unsigned N, typename Field>
BaseMatrix<M, N, Field>& BaseMatrix<M, N, Field>::invert() {
    STATIC_ASSERT(N == M);
 
    std::vector<std::vector<Field> > cmp(M, std::vector<Field> (2 * N, 0));
    
    for (unsigned row = 0; row < M; ++row) {
        cmp[row][row + N] = Field(1);
        for (unsigned column = 0; column < N; ++column) {
            cmp[row][column] = _matrix[row][column];
        }
    } 
         
    BaseMatrix<M, 2 * N, Field> help(cmp);
        
    help.gaussMethod();
    unsigned rank = 0;
    for (unsigned i = 0; i < N; ++i) {
        if (help[i][i] == 0) {
            break;
        }
        rank++;
    } 
        
    if (rank < N) {
        std::cout << "bad rank\n";
        return *this;
    } 

    for (unsigned row = 0; row < M; ++row) {
        for (unsigned column = 0; column < N; ++column) {
            _matrix[row][column] = help[row][column + N];
        }
    }
    return *this;
}

template<unsigned M, unsigned N, typename Field>
BaseMatrix<M, N, Field> BaseMatrix<M, N, Field>::inverted() const {
    BaseMatrix<M, N, Field> answer = *this;
    return answer.invert();
}

// getRow, getColumn

template<unsigned M, unsigned N, typename Field>
std::vector<Field> BaseMatrix<M, N, Field>::getRow(const unsigned row) {
    return _matrix[row];
}

template<unsigned M, unsigned N, typename Field>
std::vector<Field> BaseMatrix<M, N, Field>::getColumn(const unsigned column) {
    std::vector<Field> column_(M);
    for (unsigned row = 0; row < M; ++row) {
        column_[row] = _matrix[row][column];
    }
    return column_;
}   

template<unsigned int OTHER_M, unsigned int OTHER_N, typename OTHER_Field>
std::ostream& operator<<(std::ostream &stream, const BaseMatrix<OTHER_M, OTHER_N, OTHER_Field> &m) {
    for (unsigned i = 0; i < OTHER_M; ++i) {
        for (unsigned j = 0; j < OTHER_N; ++j) {
            stream << m._matrix[i][j] << " ";
        }
        stream << '\n';
    }
    return stream; 
}
// Matrix

template<unsigned M, unsigned N, typename Field = double>
class Matrix : public BaseMatrix<M, N, Field> {
  public:
    Matrix() {
        const unsigned MIN = std::min(M, N);
        this->_matrix.resize(M, std::vector<Field> (N, 0));
        for (unsigned i = 0; i < MIN; ++i)
            this->_matrix[i][i] = (Field)1;
    }

    Matrix(const int n) {
        this->_matrix.resize(M, std::vector<Field> (N, n));
    }

    Matrix(const BaseMatrix<M, N, Field> &A) {
        for (unsigned row = 0; row < M; ++row) {
            for (unsigned column = 0; column < N; ++column) {
                this->_matrix[row][column] = A[row][column];
            }
        }
    }

    Matrix(const std::initializer_list<std::vector<Field> > &list) {
        this->_matrix.resize(M, std::vector<Field>(1,0)); 
        int i = 0;
        for (auto &elem : list) {
            this->_matrix[i] = elem;
            i++;
        }
    }
};

template<unsigned M, unsigned N, unsigned P>
class Matrix<M, N, Finite<P> > : public BaseMatrix<M, N, Finite<P> > {
  private:
    void is_field()  {
        Finite<P> a(1);
        a.inverse();
    }
  public:
    Matrix() {
        is_field();   
        this->_matrix.resize(M, std::vector<Finite<P> > (N, 0));
        for (unsigned i = 0; i < ((M < N) ? M : N); ++i)
            this->_matrix[i][i] = (Finite<P>)1;
    }

    Matrix(const int n) {
        is_field();
        this->_matrix.resize(M, std::vector<Finite<P> > (N, n));
    }

    Matrix(const BaseMatrix<M, N, Finite<P> > &A) {
        for (unsigned i = 0; i < M; ++i) {
            for (unsigned j = 0; j < N; ++j) {
                this->_matrix[i][j] = A[i][j];
            }
        }
    }
    Matrix(const std::initializer_list<std::vector<Finite<P> > > &list) {
        this->_matrix.resize(M, std::vector<Finite<P> >(1,0)); 
        int i = 0;
        for (auto &elem : list) {
            this->_matrix[i] = elem;
            i++;
        }
    }
};

// SquareMatrix


// I tried to realize SquareMatrix with temolate, but there were some problems

// template<unsigned M, typename Field>
// using SquareMatrix = BaseMatrix<M, M, Field>;


template<unsigned N, typename Field = double> 
class SquareMatrix : public BaseMatrix<N, N, Field> {
  public:
    SquareMatrix(const int n) {
        this->_matrix.resize(N, std::vector<Field> (N, n));
    }

    SquareMatrix() {
        this->_matrix.resize(N, std::vector<Field> (N, 0));
        for (unsigned i = 0; i < N; ++i)
            this->_matrix[i][i] = (Field)1;
    }

    SquareMatrix(const BaseMatrix<N, N, Field> &donor) {
        for (unsigned row = 0; row < N; ++row) {
            for (unsigned column = 0; column < N; ++column) {
                this->_matrix[row][column] = donor[row][column];
            }
        }
    } 

    SquareMatrix(const std::initializer_list<std::vector<Field> > &list) {
        this->_matrix.resize(N, std::vector<Field>(1,0)); 
        int row = 0;
        for (auto &elem : list) {
            this->_matrix[row] = elem;
            ++row;
        }
    }

    SquareMatrix& operator*=(const SquareMatrix &other) {
        SquareMatrix<N, Field> res = (*this) * other;
        *this = res;
        return *this;
    }
};

//  #include "test_matrix.h"

// int main() {
//     std::cout << "Tests\n";
//     testMatrix();
//     return 0;
// }