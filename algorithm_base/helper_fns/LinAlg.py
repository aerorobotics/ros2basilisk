import numpy as np
import scipy.linalg as la

class LinAlg():

    def __init__(self):
        pass
    
    @staticmethod
    def lstsq_cov_weighted(A,b,cov):
        """ 
        This function returns covariance-weighted least square solution.
        The list of outputs is the same as that of 
            scipy.linalg.lstsq
        """
        
        # uppper cholesky u such that cov = u' * u
        U = la.cholesky(cov)
        R = la.inv(U).T
        A_w = R @ A
        b_w = R @ b

        return la.lstsq(A_w,b_w)

    @staticmethod
    def blockSplit(denseMat,dimRow, dimCol):

        if denseMat.ndim != 2:
            raise ValueError('denseMat must be 2D array')

        nrow, ncol = denseMat.shape
        nBlockRow = int(nrow/dimRow)
        nBlockCol = int(ncol/dimCol)

        idxRow = 0

        blockMat = []
        for i in range(nBlockRow):

            blockMatRow = []
            idxCol = 0  
            for j in range(nBlockCol):
                blockSub = denseMat[idxRow:(idxRow+dimRow),idxCol:(idxCol+dimCol)]
                blockMatRow.append(blockSub)
                idxCol += dimCol

            idxRow += dimRow

            blockMat.append(blockMatRow)

        return blockMat

    @staticmethod
    def vectorize_sym_mat(mat_sym):
        """
        Convert a symmetric matrix into a vectorized upper-triangular representation.
        E.g. 
            [[1,2,3],
            [2,4,5],    -->  [1,2,3,4,5,6]
            [3,5,6]]
        """
        
        return mat_sym[np.triu_indices(mat_sym.shape[0])]

    @staticmethod
    def unvectorize_sym_mat(mat_vectorized, dim_mat):
        """
        converted the vectorized upper-triangular representation of symmetric matrix to
        the full symmetric matrix
        """
        
        mat_symm = np.zeros((dim_mat,dim_mat))
        mat_symm[np.triu_indices(dim_mat, k = 0)] = mat_vectorized
        mat_symm = mat_symm + mat_symm.T - np.diag(np.diag(mat_symm))

        return mat_symm


def skew(w): 
    w_cross = np.array([[    0, -w[2],  w[1]],
                        [ w[2],     0, -w[0]],
                        [-w[1],  w[0],     0]]) 
    return w_cross 

def jac_vec_noralized(vec):
    """This function computes the jacobian of 
        u: R^n --> R^n defined as u(p) = p/||p||
    """
    n = len(vec)
    norm_vec = np.linalg.norm(vec)
    return np.eye(n)/norm_vec - np.outer(vec,vec) / norm_vec**3


def replace_submatrix(mat, ind1, ind2, mat_replace):
    for i, index in enumerate(ind1):
        mat[index, ind2] = mat_replace[i, :]
    return mat
