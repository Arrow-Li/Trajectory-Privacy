#include <iostream>
#include <iomanip>
#include <memory.h>
#include <math.h>

#define INF 0x3f3f3f3f

using namespace std;

class Matrix
{
private:
    int row, column;
    double **mat;
public:
    Matrix(int, int);
    Matrix(const Matrix &);
    void set(int, int, double);
    double weight(int, int);
    void show();
    void plus(int, Matrix&);
    void del(int);
    //void Del(int);
};

Matrix::Matrix(int m, int n) {
    row = m;
    column = n;
    mat = new double*[row];
    for (int i = 0; i < row; ++i) {
        mat[i] = new double[column];
        for (int j = 0; j < column; ++j) {
            mat[i][j] = INF;
            mat[i][i] = 0;
        }
    }
}

Matrix::Matrix(const Matrix &M) {
    row = M.row;
    column = M.column;
    mat = new double*[row];
    for (int i = 0; i < row; ++i) {
        mat[i] = new double[column];
        memcpy(mat[i], M.mat[i], sizeof(M.mat[i])*column);
    }
}

void Matrix::set(int i, int j, double x) {
    mat[i][j] = x;
    mat[j][i] = x;
}

double Matrix::weight(int i, int j) {
    return mat[i][j];
}

void Matrix::show() {
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < column; ++j) {
            if (mat[i][j] != INF) {
                if (mat[i][j] == 0)
                    cout << 0 << " ";
                else
                    cout << 1 << " ";
            }
            else
                cout << "* ";
        }
        cout << endl;
    }
    cout << endl;
}

void Matrix::del(int x) {
    for (int i = 0; i < row; ++i)
        for (int j = x; j < column - 1; ++j)
            mat[i][j] = mat[i][j + 1];
    for (int i = 0; i < column; ++i)
        for (int j = x; j < row - 1; ++j)
            mat[j][i] = mat[j + 1][i];
    row--; column--;
}

void Matrix::plus(int length, Matrix& M) {
    int i = 0, j = 0;
    Matrix new_Matrix(row + length, column + length);
    for (int k = 0; k < row; ++k) {
        for (int l = 0; l < column; ++l) {
            new_Matrix.mat[k][l] = mat[k][l];
        }
    }
    for (int m = row; m < new_Matrix.row; ++m) {
        for (int k = column; k < new_Matrix.column; ++k) {
            new_Matrix.mat[m][k] = M.mat[i][j];
            j++;
        }
        j = 0;
        i++;
    }
    for (int k = 0; k < row; ++k)
        delete []mat[k];
    delete mat;
    row += length; column += length;
    mat = new_Matrix.mat;
}