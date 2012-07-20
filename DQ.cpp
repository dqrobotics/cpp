#include "DQ.h"
#include <iostream>
#include<math.h>

//#include <boost/numeric/ublas/vector.hpp>
//#include <boost/numeric/ublas/io.hpp>
//using namespace boost::numeric::ublas;
using std::cout;

//Construtor sem parametros
DQ::DQ() {
    for(int n = 0; n < 8; n++){
        q[n] = 0;
	}
};

//Construtor com parametros
DQ::DQ(double v[], int tamanho){
    if(tamanho == 8 || tamanho == 4 || tamanho == 1) {
		for(int n = 0; n < tamanho ; n++) {
            q[n] = v[n];
		}
		for(int n = tamanho; n < 8 ; n++) {
            q[n] = 0;
        }
	}
	else
		//erro
		cout << "\n" << "ERRO: VETOR V DEVE TER TAMANHOS 8, 4 OU 1";
};
//
//DQ::DQ(const DQ &dq){
//	for(int n = 0; n < 8; n++){
//	q[n] = dq.q[n];
//	}
//};

//Destrutor da classe
DQ::~DQ(){};

                                     //Metodos de chamada das constantes
DQ const DQ::E() {
	return set_E();
};

DQ const DQ::i() {
	return set_i();
};

DQ const DQ::j() {
	return set_j();
};

DQ const DQ::k() {
	return set_k();
};

DQ const DQ::P() {
    return set_P();
}

DQ const DQ::D() {
    return set_D();
}

DQ const DQ::Re() {
    return set_Re();
}

DQ const DQ::Im() {
    return set_Im();
}

DQ const DQ::conj() {
    return set_conj();
}

double const DQ::norm() {
    return set_norm();
}

Matriz const DQ::Hplus4() {
    set_Hplus4();
    return op_Hplus4;
}

Matriz const DQ::Hminus4() {
    set_Hminus4();
    return op_Hminus4;
}

Matriz const DQ::Hplus8() {
    set_Hplus8();
    return op_Hplus8;
}

Matriz const DQ::Hminus8() {
    set_Hminus8();
    return op_Hminus8;
}

Matriz const DQ::vec4() {
    set_vec4();
    return op_vec4;
}

Matriz const DQ::vec8() {
    set_vec8();
    return op_vec8;
}

                                        //Metodos Privados da Classe
DQ DQ::set_E() {
    double v[8];
    for(int n = 0; n < 8; n++){
	v[n] = 0;
}
    v[4] = 1;
    return DQ(v,8);
};

DQ DQ::set_i() {
    double v[8];
    for(int n = 0; n < 8; n++){
	v[n] = 0;
}
    v[1] = 1;
    return DQ(v,8);
};

DQ DQ::set_j() {
    double v[8];
    for(int n = 0; n < 8; n++){
	v[n] = 0;
}
    v[2] = 1;
    return DQ(v,8);
};

DQ DQ::set_k() {
    double v[8];
    for(int n = 0; n < 8; n++){
	v[n] = 0;
}
    v[3] = 1;
    return DQ(v,8);
};

DQ DQ::set_P() {
    double v[8];
    for(int n = 0; n < 4; n++){
        v[n] = q[n];
	}
	for(int n = 4; n < 8; n++){
        v[n] = 0;
	}
    return DQ(v,8);
};

DQ DQ::set_D() {
    double v[8];
    for(int n = 0; n < 4; n++){
        v[n] = q[n+4];
	}
		for(int n = 4; n < 8; n++){
        v[n] = 0;
	}
    return DQ(v,8);
};

DQ DQ::set_Re() {
    double v[8];
    for(int n = 0; n < 8; n++){
        v[n] = 0;
	}
	v[0] = q[0];
	v[4] = q[4];
    return DQ(v,8);
};

DQ DQ::set_Im() {
    double v[8];
    for(int n = 0; n < 8; n++){
        v[n] = q[n];
	}
	v[0] = 0;
	v[4] = 0;
	return DQ(v,8);
};

DQ DQ::set_conj() {
    double v[8];
    for(int n = 0; n < 8; n++){
        v[n] = -q[n];
	}
	v[0] = q[0];
	v[4] = q[4];
	return DQ(v,8);
};

double DQ::set_norm() {
    DQ aux;
    DQ aux2;
    DQ aux3;
    double norm;

    for(int n = 0; n < 8; n++){
        aux.q[n] = q[n];
	}
	if(aux.P() == 0) //Primary == 0
        norm = 0;

	else if (aux.D() == 0){ //Dual == 0
        aux2 = aux.P();
        aux2 = aux2 * aux2.conj();
        norm = sqrt(aux2.q[0]);
	}
	else{
        aux2 = aux.P();
        aux3 = aux.D();
        aux2 = aux2*aux2.conj() + aux2*aux3.conj() + aux3*aux2.conj();
        norm = sqrt(aux2.q[0]);
	}

	return norm;
};

double DQ::threshold() {
	const double threshold = 0.000000000001;
	return threshold;
};

void DQ::set_Hplus4() {
    Matriz H(4,4);
    op_Hplus4 = H;
    op_Hplus4.elem[0][0] = q[0]; op_Hplus4.elem[0][1] = -q[1]; op_Hplus4.elem[0][2] = -q[2]; op_Hplus4.elem[0][3] = -q[3];
    op_Hplus4.elem[1][0] = q[1]; op_Hplus4.elem[1][1] =  q[0]; op_Hplus4.elem[1][2] = -q[3]; op_Hplus4.elem[1][3] =  q[2];
    op_Hplus4.elem[2][0] = q[2]; op_Hplus4.elem[2][1] =  q[3]; op_Hplus4.elem[2][2] =  q[0]; op_Hplus4.elem[2][3] = -q[1];
    op_Hplus4.elem[3][0] = q[3]; op_Hplus4.elem[3][1] = -q[2]; op_Hplus4.elem[3][2] =  q[1]; op_Hplus4.elem[3][3] =  q[0];
};

void DQ::set_Hminus4() {
    Matriz H(4,4);
    op_Hminus4 = H;
    op_Hminus4.elem[0][0] = q[0]; op_Hminus4.elem[0][1] = -q[1]; op_Hminus4.elem[0][2] = -q[2]; op_Hminus4.elem[0][3] = -q[3];
    op_Hminus4.elem[1][0] = q[1]; op_Hminus4.elem[1][1] =  q[0]; op_Hminus4.elem[1][2] =  q[3]; op_Hminus4.elem[1][3] = -q[2];
    op_Hminus4.elem[2][0] = q[2]; op_Hminus4.elem[2][1] = -q[3]; op_Hminus4.elem[2][2] =  q[0]; op_Hminus4.elem[2][3] =  q[1];
    op_Hminus4.elem[3][0] = q[3]; op_Hminus4.elem[3][1] =  q[2]; op_Hminus4.elem[3][2] = -q[1]; op_Hminus4.elem[3][3] =  q[0];
};

void DQ::set_Hplus8() {
    Matriz H(8,8);
    op_Hplus8 = H;
    op_Hplus8.elem[0][0] = q[0]; op_Hplus8.elem[0][1] = -q[1]; op_Hplus8.elem[0][2] = -q[2]; op_Hplus8.elem[0][3] = -q[3];
    op_Hplus8.elem[1][0] = q[1]; op_Hplus8.elem[1][1] =  q[0]; op_Hplus8.elem[1][2] = -q[3]; op_Hplus8.elem[1][3] =  q[2];
    op_Hplus8.elem[2][0] = q[2]; op_Hplus8.elem[2][1] =  q[3]; op_Hplus8.elem[2][2] =  q[0]; op_Hplus8.elem[2][3] = -q[1];
    op_Hplus8.elem[3][0] = q[3]; op_Hplus8.elem[3][1] = -q[2]; op_Hplus8.elem[3][2] =  q[1]; op_Hplus8.elem[3][3] =  q[0];

    op_Hplus8.elem[0][4] = 0; op_Hplus8.elem[0][5] = 0; op_Hplus8.elem[0][6] = 0; op_Hplus8.elem[0][7] = 0;
    op_Hplus8.elem[1][4] = 0; op_Hplus8.elem[1][5] = 0; op_Hplus8.elem[1][6] = 0; op_Hplus8.elem[1][7] = 0;
    op_Hplus8.elem[2][4] = 0; op_Hplus8.elem[2][5] = 0; op_Hplus8.elem[2][6] = 0; op_Hplus8.elem[2][7] = 0;
    op_Hplus8.elem[3][4] = 0; op_Hplus8.elem[3][5] = 0; op_Hplus8.elem[3][6] = 0; op_Hplus8.elem[3][7] = 0;

    op_Hplus8.elem[4][0] = q[4]; op_Hplus8.elem[4][1] = -q[5]; op_Hplus8.elem[4][2] = -q[6]; op_Hplus8.elem[4][3] = -q[7];
    op_Hplus8.elem[5][0] = q[5]; op_Hplus8.elem[5][1] =  q[4]; op_Hplus8.elem[5][2] = -q[7]; op_Hplus8.elem[5][3] =  q[6];
    op_Hplus8.elem[6][0] = q[6]; op_Hplus8.elem[6][1] =  q[7]; op_Hplus8.elem[6][2] =  q[4]; op_Hplus8.elem[6][3] = -q[5];
    op_Hplus8.elem[7][0] = q[7]; op_Hplus8.elem[7][1] = -q[6]; op_Hplus8.elem[7][2] =  q[5]; op_Hplus8.elem[7][3] =  q[4];

    op_Hplus8.elem[4][4] = q[0]; op_Hplus8.elem[4][5] = -q[1]; op_Hplus8.elem[4][6] = -q[2]; op_Hplus8.elem[4][7] = -q[3];
    op_Hplus8.elem[5][4] = q[1]; op_Hplus8.elem[5][5] =  q[0]; op_Hplus8.elem[5][6] = -q[3]; op_Hplus8.elem[5][7] =  q[2];
    op_Hplus8.elem[6][4] = q[2]; op_Hplus8.elem[6][5] =  q[3]; op_Hplus8.elem[6][6] =  q[0]; op_Hplus8.elem[6][7] = -q[1];
    op_Hplus8.elem[7][4] = q[3]; op_Hplus8.elem[7][5] = -q[2]; op_Hplus8.elem[7][6] =  q[1]; op_Hplus8.elem[7][7] =  q[0];
};

void DQ::set_Hminus8() {
    Matriz H(8,8);
    op_Hminus8 = H;
    op_Hminus8.elem[0][0] = q[0]; op_Hminus8.elem[0][1] = -q[1]; op_Hminus8.elem[0][2] = -q[2]; op_Hminus8.elem[0][3] = -q[3];
    op_Hminus8.elem[1][0] = q[1]; op_Hminus8.elem[1][1] =  q[0]; op_Hminus8.elem[1][2] =  q[3]; op_Hminus8.elem[1][3] = -q[2];
    op_Hminus8.elem[2][0] = q[2]; op_Hminus8.elem[2][1] = -q[3]; op_Hminus8.elem[2][2] =  q[0]; op_Hminus8.elem[2][3] =  q[1];
    op_Hminus8.elem[3][0] = q[3]; op_Hminus8.elem[3][1] =  q[2]; op_Hminus8.elem[3][2] = -q[1]; op_Hminus8.elem[3][3] =  q[0];

    op_Hminus8.elem[0][4] = 0; op_Hminus8.elem[0][5] = 0; op_Hminus8.elem[0][6] = 0; op_Hminus8.elem[0][7] = 0;
    op_Hminus8.elem[1][4] = 0; op_Hminus8.elem[1][5] = 0; op_Hminus8.elem[1][6] = 0; op_Hminus8.elem[1][7] = 0;
    op_Hminus8.elem[2][4] = 0; op_Hminus8.elem[2][5] = 0; op_Hminus8.elem[2][6] = 0; op_Hminus8.elem[2][7] = 0;
    op_Hminus8.elem[3][4] = 0; op_Hminus8.elem[3][5] = 0; op_Hminus8.elem[3][6] = 0; op_Hminus8.elem[3][7] = 0;

    op_Hminus8.elem[4][0] = q[4]; op_Hminus8.elem[4][1] = -q[5]; op_Hminus8.elem[4][2] = -q[6]; op_Hminus8.elem[4][3] = -q[7];
    op_Hminus8.elem[5][0] = q[5]; op_Hminus8.elem[5][1] =  q[4]; op_Hminus8.elem[5][2] =  q[7]; op_Hminus8.elem[5][3] = -q[6];
    op_Hminus8.elem[6][0] = q[6]; op_Hminus8.elem[6][1] = -q[7]; op_Hminus8.elem[6][2] =  q[4]; op_Hminus8.elem[6][3] =  q[5];
    op_Hminus8.elem[7][0] = q[7]; op_Hminus8.elem[7][1] =  q[6]; op_Hminus8.elem[7][2] = -q[5]; op_Hminus8.elem[7][3] =  q[4];

    op_Hminus8.elem[4][4] = q[0]; op_Hminus8.elem[4][5] = -q[1]; op_Hminus8.elem[4][6] = -q[2]; op_Hminus8.elem[4][7] = -q[3];
    op_Hminus8.elem[5][4] = q[1]; op_Hminus8.elem[5][5] =  q[0]; op_Hminus8.elem[5][6] =  q[3]; op_Hminus8.elem[5][7] = -q[2];
    op_Hminus8.elem[6][4] = q[2]; op_Hminus8.elem[6][5] = -q[3]; op_Hminus8.elem[6][6] =  q[0]; op_Hminus8.elem[6][7] =  q[1];
    op_Hminus8.elem[7][4] = q[3]; op_Hminus8.elem[7][5] =  q[2]; op_Hminus8.elem[7][6] = -q[1]; op_Hminus8.elem[7][7] =  q[0];
};

void DQ::set_vec4() {
    Matriz H(4,1);
    op_vec4 = H;
    op_vec4.elem[0][0] = q[0];
    op_vec4.elem[1][0] = q[1];
    op_vec4.elem[2][0] = q[2];
    op_vec4.elem[3][0] = q[3];
};

void DQ::set_vec8() {
    Matriz H(8,1);
    op_vec8 = H;
    op_vec8.elem[0][0] = q[0];
    op_vec8.elem[1][0] = q[1];
    op_vec8.elem[2][0] = q[2];
    op_vec8.elem[3][0] = q[3];
    op_vec8.elem[4][0] = q[4];
    op_vec8.elem[5][0] = q[5];
    op_vec8.elem[6][0] = q[6];
    op_vec8.elem[7][0] = q[7];
};

//void DQ::display() {
//    cout << "\n";
//    char *aparentesesP;
//    char *q1; char *q2; char *ip; char *q3; char *jp; char *q4; char *kp;
//    char *fparentesesP;
//    char *aparentesesD;
//    char *E; char *q5; char *q6; char *id; char *q7; char *jd; char *q8; char *kd;
//    char *fparentesesD;
//    if(P() == 0){
//    aparentesesP = "";
//    fparentesesP = "";
//    }
//    else{
//    aparentesesP = "(";
//    fparentesesP = ")";
//    }
//    if(q[0] != 0)
//
//    q1 = (char) q[0];
//    if(q[1] != 0)
//    cout << " " << q[1] << "i"
//    if(q[2] != 0)
//    cout << " " << q[2] << "j"
//    if(q[3] != 0)
//    cout << " " << q[3] << "k"

//};

                                        //Operadores sobrecarregados
                //SOBRECARGA OPERADOR (+)
DQ operator+(DQ dq1, DQ dq2){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq1.q[n] + dq2.q[n];
    }
    return DQ(v,8);
};

//Sobrecarga + para escalar int
DQ operator+(DQ dq, int scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = dq.q[0] + scalar;
    return DQ(v,8);
};
DQ operator+(int scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = scalar + dq.q[0];
    return DQ(v,8);
};

//Sobrecarga + para escalar float
DQ operator+(DQ dq, float scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = dq.q[0] + scalar;
    return DQ(v,8);
};
DQ operator+(float scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = scalar + dq.q[0];
    return DQ(v,8);
};

//Sobrecarga + para escalar double
DQ operator+(DQ dq, double scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = dq.q[0] + scalar;
    return DQ(v,8);
};
DQ operator+(double scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = scalar + dq.q[0];
    return DQ(v,8);
};

                //SOBRECARGA OPERADOR (-)
DQ operator-(DQ dq1, DQ dq2){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq1.q[n] - dq2.q[n];
    }
    return DQ(v,8);
};

//Sobrecarga - para escalar int
DQ operator-(DQ dq, int scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = dq.q[0] - scalar;
    return DQ(v,8);
};
DQ operator-(int scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = scalar - dq.q[0];
    return DQ(v,8);
};

//Sobrecarga - para escalar float
DQ operator-(DQ dq, float scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = dq.q[0] - scalar;
    return DQ(v,8);
};
DQ operator-(float scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = scalar - dq.q[0];
    return DQ(v,8);
};

//Sobrecarga - para escalar double
DQ operator-(DQ dq, double scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = dq.q[0] - scalar;
    return DQ(v,8);
};
DQ operator-(double scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n];
    }
    v[0] = scalar - dq.q[0];
    return DQ(v,8);
};

                //SOBRECARGA OPERADOR (*)
DQ operator*(DQ dq1, DQ dq2){
    double v[8];
    v[0] = dq1.q[0]*dq2.q[0] - dq1.q[1]*dq2.q[1] - dq1.q[2]*dq2.q[2] - dq1.q[3]*dq2.q[3];
    v[1] = dq1.q[0]*dq2.q[1] + dq1.q[1]*dq2.q[0] + dq1.q[2]*dq2.q[3] - dq1.q[3]*dq2.q[2];
    v[2] = dq1.q[0]*dq2.q[2] - dq1.q[1]*dq2.q[3] + dq1.q[2]*dq2.q[0] + dq1.q[3]*dq2.q[1];
    v[3] = dq1.q[0]*dq2.q[3] + dq1.q[1]*dq2.q[2] - dq1.q[2]*dq2.q[1] + dq1.q[3]*dq2.q[0];

    v[4] = dq1.q[0]*dq2.D().q[0] - dq1.q[1]*dq2.D().q[1] - dq1.q[2]*dq2.D().q[2] - dq1.q[3]*dq2.D().q[3];
    v[5] = dq1.q[0]*dq2.D().q[1] + dq1.q[1]*dq2.D().q[0] + dq1.q[2]*dq2.D().q[3] - dq1.q[3]*dq2.D().q[2];
    v[6] = dq1.q[0]*dq2.D().q[2] - dq1.q[1]*dq2.D().q[3] + dq1.q[2]*dq2.D().q[0] + dq1.q[3]*dq2.D().q[1];
    v[7] = dq1.q[0]*dq2.D().q[3] + dq1.q[1]*dq2.D().q[2] - dq1.q[2]*dq2.D().q[1] + dq1.q[3]*dq2.D().q[0];

    v[4] = v[4] + dq1.D().q[0]*dq2.P().q[0] - dq1.D().q[1]*dq2.P().q[1] - dq1.D().q[2]*dq2.P().q[2] - dq1.D().q[3]*dq2.P().q[3];
    v[5] = v[5] + dq1.D().q[0]*dq2.P().q[1] + dq1.D().q[1]*dq2.P().q[0] + dq1.D().q[2]*dq2.P().q[3] - dq1.D().q[3]*dq2.P().q[2];
    v[6] = v[6] + dq1.D().q[0]*dq2.P().q[2] - dq1.D().q[1]*dq2.P().q[3] + dq1.D().q[2]*dq2.P().q[0] + dq1.D().q[3]*dq2.P().q[1];
    v[7] = v[7] + dq1.D().q[0]*dq2.P().q[3] + dq1.D().q[1]*dq2.P().q[2] - dq1.D().q[2]*dq2.P().q[1] + dq1.D().q[3]*dq2.P().q[0];

    return DQ(v,8);
};

//Sobrecarga * para escalar int
DQ operator*(DQ dq, int scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n] * scalar;
    }

    return DQ(v,8);
};
DQ operator*(int scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = scalar * dq.q[n];
    }

    return DQ(v,8);
};

//Sobrecarga * para escalar float
DQ operator*(DQ dq, float scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n] * scalar;
    }

    return DQ(v,8);
};
DQ operator*(float scalar, DQ dq){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = scalar * dq.q[n];
    }

    return DQ(v,8);
};

//Sobrecarga * para escalar double
DQ operator*(DQ dq, double scalar){
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = dq.q[n] * scalar;
    }

    return DQ(v,8);
};
DQ operator*(double scalar, DQ dq) {
    double v[8];
    for(int n = 0; n<8; n++){
        v[n] = scalar * dq.q[n];
    }

    return DQ(v,8);
};
                    //SOBRECARGA OPERADOR (==)
bool DQ::operator==(DQ dq2) {
    for(int n = 0; n<8; n++){
        if(q[n] != dq2.q[n])
        return false; //quaternios não são iguais
    }
    return true; //quaternios iguais
};

//Sobrecarga == para escalar int
bool operator==(DQ dq, int scalar) {
    for(int n = 0; n<8; n++){
        if(dq.q[n] != scalar)
        return false; //quaternio diferente do escalar
    }
    return true; //quaternios igual ao escalar
};

bool operator==(int scalar, DQ dq) {
    for(int n = 0; n<8; n++){
        if(scalar != dq.q[n])
        return false; //quaternio diferente do escalar
    }
    return true; //quaternios igual ao escalar
};

int main() {
	double v[8] = {1,2,3,4,5,6,7,8};
    int equal = 0;
	DQ dq2(v,8);
	DQ dq3 = dq2;
    cout << "linhas A" << dq2.Hplus8().L << "\n";
    cout << "colunas B" << dq3.vec8().C <<"\n";
    Matriz prod = dq2.Hplus8() * dq3.vec8();
    DQ dq5 = dq2 * dq3;
    dq3.q[0] = dq3.q[0] + 0;
    if(dq2 == dq3){
        equal = 1;
    }
    cout << "\n" << "igualdade" << equal << "\n";
    //Matriz A = dq2.Hplus4() + dq3.Hplus4();
    for(int i=0; i<4; i++){
        cout << " " << dq5.P().q[i];
    }
    cout << "\n";
    for(int i=0; i<4; i++){
        cout << " " << dq5.D().q[i];
    }
    cout << "\n";
    for(int i=0; i<prod.L; i++) {
        for(int j=0; j<prod.C; j++) {
         cout << " " << prod.elem[i][j];
        }
        cout << "\n";
    }
	system("PAUSE");
}
