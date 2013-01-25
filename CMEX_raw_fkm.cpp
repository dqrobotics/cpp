#include "mex.h" /* Always include this */
#include "DQ.h"
#include "DQ.cpp"
#include "DQ_kinematics.h"
#include "DQ_kinematics.cpp"
#include<math.h>

/* Input Arguments */
#define	DH_PARAM prhs[0]
#define	DH_CONVENTION prhs[1]
#define THETA prhs[2]
#define ITH prhs[3]

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[] )

{
    mwSize rows, columns, thcolumns, qcolumns, i, j;
    double *DH_pointer, *theta_pointer, *fkm_pointer;
    matrix <double> A;
    vector <double> theta_vec, q_vec;
    DQ fkm;
    char *input_buf;
    std::string conv;
    mwSize buflen;

/* Check for proper number of arguments */

    if (nrhs != 4) {
	mexErrMsgTxt("Four input arguments required.");
    } else if (nlhs > 1) {
	mexErrMsgTxt("Too many output arguments.");
    }
    if ( mxIsDouble(DH_PARAM) != 1)
      mexErrMsgTxt("DH Param Input must be mxArray Double.");

    // Transformation of the DH_PARAM input in a boost matrix for use with DQ_kinematics
    rows = mxGetM(DH_PARAM); //number of rows: 4 or 5(if have dummy joints)
    columns = mxGetN(DH_PARAM); //number of columns = number of links of robot system
    A.resize(rows, columns); //boost matrix to create DQ_kinematics object with DH_PARAM values
    DH_pointer = (double *)mxGetPr(DH_PARAM);
    for(i = 0; i < columns; i++) {
        for(j = 0; j < rows; j++) {
            A(j,i) = *DH_pointer++;
        }
    }

        /* input must be a string */
    if ( mxIsChar(DH_CONVENTION) != 1)
      mexErrMsgTxt("Input must be a string.");

    /* input must be a row vector */
    if (mxGetM(DH_CONVENTION)!=1)
      mexErrMsgTxt("Input must be a row vector.");

    /* get the length of the input string */
    buflen = (mxGetM(DH_CONVENTION) * mxGetN(DH_CONVENTION)) + 1;

    /* copy the string data from prhs[0] into a C string input_ buf.    */
    input_buf = mxArrayToString(DH_CONVENTION);

//    if(input_buf != "standard" && input_buf != "modified") {
//        mexErrMsgTxt("convention must be standard or modfied. Write it correctly");
//    }

    DQ_kinematics Robot_kin(A,input_buf);//Creates a DQ_kinematics object

    // Transformation of the input THETA in a boost vector for use with DQ_kinematics
    thcolumns = mxGetNumberOfElements(THETA);
    theta_vec.resize(thcolumns);
    theta_pointer = (double *)mxGetPr(THETA);
    for(i = 0; i < thcolumns; i++) {
        theta_vec(i) = theta_pointer[i];
    }

    fkm = Robot_kin.raw_fkm(theta_vec, (int) mxGetScalar(ITH)); //calculation of robotic system fkm

    //Passing the jacobian result to the matlab output interface plhs
    q_vec = fkm.q;
    qcolumns = q_vec.size();
    plhs[0] = mxCreateNumericMatrix(1, qcolumns, mxDOUBLE_CLASS, mxREAL);
    fkm_pointer = (double *)mxGetPr(plhs[0]);
        /* Copy data into the mxArray */
    for(i = 0; i < qcolumns; i++) {
            *fkm_pointer++ = q_vec(i);
    }

    return;
}
