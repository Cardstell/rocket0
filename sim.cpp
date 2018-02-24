#include <iostream>
#include <cmath>
#include <fstream>

using namespace std;

const double PI = 3.141592653589793;
const double cG = 6.67408e-11;
const double mEarth = 5.972e24;
const double rEarth = 6371e3;
const double cT0 = 288.15;
const double cp0 = 101325;
const double cg = 9.880665;
const double gL = -0.0065;
const double cR = 8.31447;
const double cM = 0.0289644;

int tickRate = 1000;

double mR = 0.15; //масса пустой ракеты
double dF = 0.07;//диаметр топливного бака
double lF = 0.45; //длина топливного бака
double lmF = 0.45; //длина топливного бака
double uF = 0.0076;//скорость сгорания топлива 
double vdS = 6.0376e-5;//объем сопла
double pF = 1840; //плотность топлива
double dS = 0.025;//средний диаметр сопла
double cxAero = 0.2;//сопротивление воздуха

double gas(double temp, double m) {
	double
	mCO2 = 	m*0.1763,
	mCO =	m*0.1494,
	mH2O = 	m*0.1863,
	mH2 = 	m*0.0091,
	mN2 = 	m*0.1003,
	mNaCO3= m*0.3735,
	mNaOH = m*0.0048;
	double vm = 22.413/273.15*(temp+273.15);
	double v = mNaCO3/2530.0 + mNaOH/2130.0;
	v += mCO2*vm/44;
	v += mCO*vm/28;
	v += mH2O*vm/18;
	v += mH2*vm/2;
	v += mN2*vm/28;
	return v;
}

void simulate() {
	double height = 0;
	double u = 0;
	double time, m, vF;
	double max_h = 0, max_u = 0, max_p = 0, max_q = 0, max_qh = 0, max_a = 0;
	int iters = 0;
	ofstream file;
	file.open("rdata.txt");
	for (time = 0;(height>0.0)||(lF>0.1); time += 1.0/tickRate) {
		vF = (dF*dF/4*PI)*lF;
		m = mR + vF*pF;
		//engine
		double sE = dF*dF/4*PI;
		double aE = 0;
		if (lF>0) {
			double temp = 1500.0 - pow(1.5, 18.0 - time);
			double vsF = (lF < 0.05)? (sE * uF * lF / 0.05) : (sE * uF);
			double vG = gas(temp, vsF*pF);
			double uG = (vG - vsF) / (PI*dS*dS/4);
			aE = uG*vsF*pF / m;
			lF -= ((lF < 0.05)? (uF * lF / 0.05) : (uF)) / tickRate;
		}
		//gravity
		double aG =  -cG * mEarth / ((rEarth+height)*(rEarth+height));
		//aero
		double pP = cp0*pow(1+gL*height/cT0,-(cg*cM)/(cR*gL));
		double pS = (pP*cM)/(cR*(cT0 + gL*height));
		double aA = cxAero*pS*u*u*sE/2.0/m;
		if (aA*m>max_q) {
			max_qh = height;
			max_q = aA*m;
		}
		if (u>0.0) aA = -aA;
		//
		double a = (aG + aE + aA);
		u += a/tickRate;
		height += u / tickRate;
		if (height<0) {
			u = 0;
			height = 0;
		}
		max_h = max(max_h, height);
		max_u = max(max_u, u);
		max_a = max(max_a, a);
		if (iters*10%tickRate==0) {
			file << time << " "<< height << " " << u << " " << a << " " << aE*m << " " << m << " " << abs(aA*m) << endl;
		}
		++iters;
	}
	file.close();
	cout<<"Max height:   "<<max_h<<" m"<<endl;
	cout<<"Max speed:    "<<max_u<<" m/s"<<endl;
	cout<<"Max acc:      "<<max_a<<" m/s2"<<endl;
	cout<<"Max-Q height: "<<max_qh<<" m"<<endl;
}

int main() {
	simulate();
}