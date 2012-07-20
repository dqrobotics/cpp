#include <iostream>
#include<math.h> //biblioteca para funçoes matemáticas

#ifndef Matriz_H
#define Matriz_H
#define MAX 8

class Matriz{
    public:
    int L;
    int C;
    double elem[MAX][MAX];

    Matriz();
    Matriz(int l, int c);
    ~Matriz();
    friend Matriz operator+(Matriz A, Matriz B);
    friend Matriz operator*(Matriz A, Matriz B);
};

#endif // MATRIZ
