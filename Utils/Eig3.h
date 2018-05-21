/*
 * Eig3.h
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#ifndef UTILS_EIG3_H_
#define UTILS_EIG3_H_

/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. */
void eigen_decomposition(double A[3][3], double V[3][3], double d[3]);


#endif /* UTILS_EIG3_H_ */
