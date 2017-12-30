#include <iostream>
#define INF 0x3f3f3f3f

class Matrix {
  int n, length;
  double *mat;

public:
  Matrix(int);
  Matrix(const Matrix &);
  void set_value(int, int, double);
  double get_value(int, int);
  void merge(int, int, double, Matrix &);
  void del(int);
  void print();
  void show(); // TODO temp
};

Matrix::Matrix(int n) {
  this->n = n;
  length = (n + 1) * n / 2;
  mat = new double[length];
  for (int i = 0; i < length; i++)
    mat[i] = INF;
}

Matrix::Matrix(const Matrix &m) {
  this->n = m.n;
  this->length = m.length;
  this->mat = new double[this->n];
  for (int i = 0; i < n; i++)
    this->mat[i] = m.mat[i];
}

void Matrix::set_value(int row, int col, double w) {
  if ((row >= n || col >= n) || (row < 0 || col < 0)) {
    std::cout << "Set Error!" << std::endl;
    return;
  }
  if (row >= col)
    mat[row * (row + 1) / 2 + col] = w;
  else
    mat[col * (col + 1) / 2 + row] = w;
}

double Matrix::get_value(int row, int col) {
  if (row < col) {
    int temp = row;
    row = col;
    col = temp;
  }
  return mat[row * (row + 1) / 2 + col];
}

void Matrix::merge(int v1, int v2, double w, Matrix &m) {
  int i, begin, end, new_n = this->n + m.n, new_length;
  new_length = this->length + m.length + this->n * m.n;
  double *new_mat = new double[new_length];
  std::copy(mat, mat + length, new_mat);
  i = length;
  for (int row = 0; row < m.n; row++) {
    for (int j = 0; j < this->n; j++, i++)
      new_mat[i] = INF;
    begin = row * (row + 1) / 2;
    end = begin + row + 1;
    std::copy(m.mat + begin, m.mat + end, new_mat + i);
    i += end - begin;
  }
  delete mat;
  v2 += n;
  n = new_n;
  mat = new_mat;
  length = new_length;
  set_value(v1, v2, w);
  n = new_n;
}

void Matrix::del(int x) {
  if ((x < 0 || x >= n) || mat == NULL) {
    std::cout << "Delete Error!" << std::endl;
    return;
  }
  int i = 0;
  double *new_mat;
  new_mat = new double[length - n];
  for (int row = 0; row < n; row++) {
    if (row == x)
      continue;
    for (int col = 0; col < row + 1; col++) {
      if (col == x)
        continue;
      new_mat[i] = get_value(row, col);
      i++;
    }
  }
  delete mat;
  mat = new_mat;
  length = length - n;
  n--;
}

void Matrix::print() {
  if (n < 1) {
    std::cout << "Empty Matrix!" << std::endl;
    return;
  }
  for (int row = 0; row < n; row++) {
    for (int col = 0; col < n; col++) {
      double w = get_value(row, col);
      if (w == INF)
        std::cout << "* ";
      else
        std::cout << get_value(row, col) << " ";
    }
    std::cout << std::endl;
  }
}

void Matrix::show() {
  if (n < 1) {
    std::cout << "Empty Matrix!" << std::endl;
    return;
  }
  for (int row = 0; row < n; row++) {
    for (int col = 0; col < n; col++) {
      double w = get_value(row, col);
      if (w == INF)
        std::cout << "* ";
      else
        std::cout << 1 << " ";
    }
    std::cout << std::endl;
  }
}