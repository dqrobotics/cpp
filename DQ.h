
#include <iostream>
#include<math.h> //biblioteca para funçoes matemáticas

#ifndef DQ_H
#define DQ_H
#include "Matriz.cpp"

class DQ{
    //atributos
	public:
	double q[8];
	//atributos privados da classe
    private:
    Matriz op_Hplus4;
    Matriz op_Hminus4;
    Matriz op_Hplus8;
    Matriz op_Hminus8;
    Matriz op_vec4;
    Matriz op_vec8;

    //metodos
    public:
    DQ();
	DQ(double v[], int tamanho);
	//DQ(const DQ& dq);
	~DQ();
    //metodos constantes

    DQ const E();
    DQ const i();
    DQ const j();
    DQ const k();
    DQ const P();
    DQ const D();
    DQ const Re();
    DQ const Im();
    DQ const conj();
    double const norm();
    static double threshold();
    Matriz const Hplus4();
    Matriz const Hminus4();
    Matriz const Hplus8();
    Matriz const Hminus8();
    Matriz const vec4();
    Matriz const vec8();
    //void display();

    //metodos privados da classe
    private:
    DQ set_E();
    DQ set_i();
    DQ set_j();
    DQ set_k();
    DQ set_P();
    DQ set_D();
    DQ set_Re();
    DQ set_Im();
    DQ set_conj();
    double set_norm();
    void set_Hplus4();
    void set_Hminus4();
    void set_Hplus8();
    void set_Hminus8();
    void set_vec4();
    void set_vec8();

    //Operadores sobrecarregados
	public:
	friend DQ operator+(DQ dq1, DQ dq2);
	friend DQ operator+(DQ dq, int scalar);
	friend DQ operator+(int scalar, DQ dq);
    friend DQ operator+(DQ dq, float scalar);
	friend DQ operator+(float scalar, DQ dq);
    friend DQ operator+(DQ dq, double scalar);
	friend DQ operator+(double scalar, DQ dq);

	friend DQ operator-(DQ dq1, DQ dq2);
	friend DQ operator-(DQ dq, int scalar);
	friend DQ operator-(int scalar, DQ dq);
    friend DQ operator-(DQ dq, float scalar);
	friend DQ operator-(float scalar, DQ dq);
    friend DQ operator-(DQ dq, double scalar);
	friend DQ operator-(double scalar, DQ dq);

	friend DQ operator*(DQ dq1, DQ dq2);
    friend DQ operator*(DQ dq, int scalar);
	friend DQ operator*(int scalar, DQ dq);
    friend DQ operator*(DQ dq, float scalar);
	friend DQ operator*(float scalar, DQ dq);
    friend DQ operator*(DQ dq, double scalar);
	friend DQ operator*(double scalar, DQ dq);

	bool DQ::operator==(DQ dq2);
    friend bool operator==(DQ dq, int scalar);
	friend bool operator==(int scalar, DQ dq);
	//private vector<double> a();
	//private: boost::numeric::ublas::vector<int> a();

	//static array8 constantes(); Declaração da funcao para as constantes E,i,j,k

};

#endif // DQ_H
