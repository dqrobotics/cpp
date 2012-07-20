#include "Matriz.h"
#include <iostream>
#include<math.h>

using std::cout;

Matriz::Matriz() {};
Matriz::Matriz(int l, int c) {
    L = l;
    C = c;
    for(int i=0; i<l; i++) {
        for(int j=0; j<c; j++) {
         elem[i][j] = 0;
        }
    }
};

Matriz::~Matriz() {};

Matriz operator+(Matriz A, Matriz B){
    Matriz C(A.L,A.C);
    for(int i=0; i<A.L; i++) {
        for(int j=0; j<A.C; j++) {
         C.elem[i][j] = A.elem[i][j] + B.elem[i][j];
        }
    }
    return C;
};

Matriz operator*(Matriz A, Matriz B){
    Matriz MatC(A.L,B.C);
    if(A.C == B.L){
        for(int i=0; i<B.C; i++) {
            for(int j=0; j<A.L; j++) {
                for(int k=0; k<A.C; k++) {
             MatC.elem[j][i] = MatC.elem[j][i] + (A.elem[j][k] * B.elem[k][i]);
                }
            }
        }
    }
    else
    cout << "ERRO: NUMERO DE COLUNAS DA MATRIZ A DEVE SER IGUAL AO NUMERO DE LINHAS DA MATRIZ B \n";

    return MatC;
};

//int main() {
//
//    Matriz A(4,4);
//    Matriz B(4,1);
//    Matriz C;
//
//    for(int i=0; i<A.L; i++) {
//        for(int j=0; j<A.C; j++) {
//         A.elem[i][j] = 2;
//        }
//    }
//    for(int i=0; i<B.L; i++) {
//        for(int j=0; j<B.C; j++) {
//         B.elem[i][j] = 2;
//        }
//    }
//    C = A * B;
//    for(int i=0; i<C.L; i++) {
//        for(int j=0; j<C.C; j++) {
//         cout << " " << C.elem[i][j];
//        }
//        cout << "\n";
//    }
//	system("PAUSE");
//}
