/*
 * dataAssociation.cpp
 *
 *  Created on: 9 févr. 2015
 *      Author: younes
 */
#include <stack>
#include "dataAssociation.h"
#define ROWS 63
#define COLS 65
namespace ph2 {

dataAssociation::dataAssociation() {
	// TODO Auto-generated constructor stub

}


dataAssociation::~dataAssociation() {
	// TODO Auto-generated destructor stub
}

void dataAssociation::Motion(VectorXd X, VectorXd U ){

	 MatrixXd M(ROWS,COLS);
map<int,O>
	stack<float> etat;
	Vision v;
	VectorXd X_next(3);
	etat.push(X(0)+U(0)*cos(X(2))+U(1)*sin(X(2)));
	etat.push(X(0)-U(0)*sin(X(2))+U(1)*cos(X(2)));
	etat.push(X(2)+U(2));
	M=v.brisc(2);
for(i=0;i<COLS;i++){
	etat.push(M(i,0));
	etat.push(M(i,1));
}
}





} /* namespace ph2 */
