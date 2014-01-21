/**
 * @file ndt.cpp
 * @brief Scanmatching using the NDT algorithm
 * @author Hordur Johannsson
 * @version $Id:  $
 */

#include <algorithm>

#include <opencv/cv.h>
//#include <opencv/cxcore.h>
//#include <cvaux.h>
//#include <highgui.h>

#include "perls-hauv/lm.h"
#include "perls-hauv/misc.h"

#include "ndt.h"
using namespace isam;

namespace hauv {

    /**
     * Constructs a new NDT scan with a given set of points.
     */
    NDT::NDT(const std::vector<Point2d> & p, double cellSize, double minE)
    {
        this->cellSize = cellSize;
        minEigenVRatio = minE;

        // Find boundaries of point set
        double minX, maxX, minY, maxY;
        std::vector<Point2d>::const_iterator pointItr;
        pointItr = p.begin();
        minX = maxX = (*pointItr).x();
        minY = maxY = (*pointItr).y();
        pointItr++;
        while(pointItr!=p.end()) {
            if (minX > (*pointItr).x()) minX = (*pointItr).x();
            if (maxX < (*pointItr).x()) maxX = (*pointItr).x();
            if (minY > (*pointItr).y()) minY = (*pointItr).y();
            if (maxY < (*pointItr).y()) maxY = (*pointItr).y();
            pointItr++;
        }
        R[0] = minX;	R[1] = maxX;	R[2] = minY;	R[3] = maxY;

        int W = ceil((maxX-minX)/cellSize); // Max column index
        int H = ceil((maxY-minY)/cellSize); // Max row index
        double width = W*cellSize;
        double height = H*cellSize;
        double ix,iy;
        int S = (W+1)*(H+1);

        for (int i=0; i<4; i++) {
            N[i] = new int[S];
            std::fill_n(N[i], S, 0);

            U[i] = new double[S*2];
            std::fill_n(U[i], S*2, 0.0);

            C[i] = new double[S*4];
            std::fill_n(C[i], S*4, 0.0);

            I[i] = new double[S*4];
            std::fill_n(I[i], S*4, 0.0);

            Q[i] = new double[S*4];
            std::fill_n(Q[i], S*4, 0.0);
        }

        std::cout << "NDT::NDT(...) " 
                  << S << " " << W << " " << H << std::endl;

        int ij; // Calculated offset into the array
        // Calculate the count for the scan
        // and sum the points in each cell so 
        // we can calculate the mean
        for (int i=0; i<4; i++) {
            double ox=0.0, oy=0.0;
            if (i == 1) {ox = cellSize/2.0; oy = 0.0;}
            else if (i == 2) {oy = cellSize/2.0; ox = 0.0;}
            else if (i == 3) {ox = oy =  cellSize/2.0;}

            for (pointItr = p.begin();pointItr != p.end(); pointItr++) {
                ix = ((*pointItr).x() - minX + ox)/width*W;
                iy = ((*pointItr).y() - minY + oy)/height*H;
	
                //			ij = ((int)(iy+oy))*(W+1) + (int)(ix+ox);
                ij = ((int)(iy))*(W+1) + (int)(ix);
                N[i][ij]++;

                U[i][2*ij] += (*pointItr).x();
                U[i][2*ij+1] += (*pointItr).y();
            }
        }

        // Calculate the mean
        for (int k=0; k<4; k++) {
            for (int i=0; i<=H; i++) {
                for (int j=0; j<=W; j++) {
                    ij = i*(W+1) + j;
                    if (N[k][ij]>4) {
                        U[k][ij*2]   = U[k][ij*2  ]/N[k][ij];
                        U[k][ij*2+1] = U[k][ij*2+1]/N[k][ij];
                    }
                }
            }
        }

        // Calculate covariance
        // Sum differences from mean
        for (int i=0; i<4; i++) {
            double ox=0.0, oy=0.0;
            if (i == 1) {ox = cellSize/2.0; oy = 0.0;}
            else if (i == 2) {oy = cellSize/2.0; ox = 0.0;}
            else if (i == 3) {ox = oy = cellSize/2.0;}

            for (pointItr = p.begin();pointItr != p.end(); pointItr++) {
                double x = (*pointItr).x();
                double y = (*pointItr).y();
 
                ix = (x - minX + ox)/width*W;
                iy = (y - minY + oy)/height*H;
	
                //			ij = ((int)(iy+oy))*(W+1) + (int)(ix+ox);
                ij = ((int)(iy))*(W+1) + (int)(ix);

                double mx = U[i][ij*2];
                double my = U[i][ij*2+1];

                C[i][ij*4]   += (x-mx)*(x-mx); // xx
                C[i][ij*4+1] += (x-mx)*(y-my); // xy
                C[i][ij*4+2] += (x-mx)*(y-my); // yx
                C[i][ij*4+3] += (y-my)*(y-my); // yy
            }
        }

        double T1[4] = {0.0,0.0,0.0,0.0};
        double T2[4] = {0.0,0.0,0.0,0.0};
        double D[4] = {0.0,0.0,0.0,0.0};
        double E[4] = {0.0,0.0,0.0,0.0};
        double e[2] = {0.0,0.0};
        double A[4] = {0.0,0.0,0.0,0.0};

        CvMat evals = cvMat(2, 1, CV_64FC1,e);
        CvMat evects = cvMat(2, 2, CV_64FC1,E);
        CvMat matD = cvMat(2, 2, CV_64FC1,D);
        CvMat matT1 = cvMat(2, 2, CV_64FC1,T1);
        CvMat matT2 = cvMat(2, 2, CV_64FC1,T2);

        // Then divide by N to get covariance
        for (int k=0; k<4; k++) {
            for (int i=0; i<=H; i++) {
                for (int j=0; j<=W; j++) {
                    ij = i*(W+1) + j;
                    if (N[k][ij]>4) {
                        // double c11, c12, c21, c22;
                        // c11 = C[k][ij*4]   = C[k][ij*4  ]/N[k][ij];
                        // c12 = C[k][ij*4+1] = C[k][ij*4+1]/N[k][ij];
                        // c21 = C[k][ij*4+2] = C[k][ij*4+2]/N[k][ij];
                        // c22 = C[k][ij*4+3] = C[k][ij*4+3]/N[k][ij];

                        // Calculate the eigenvalues of C
                        // For a 2x2 matrix: e = 1/2 (tr(C) +- sqrt(tr(C)^2 - 4det(C)))
                        //double e1 = 0.5 * ((c11+c22) + sqrt((c11+c22)*(c11+c22) - 4.0*(c11*c22 - c12*c21)));
                        //double e2 = 0.5 * ((c11+c22) - sqrt((c11+c22)*(c11+c22) - 4.0*(c11*c22 - c12*c21)));
                        double a = minEigenVRatio;

                        A[0] = C[k][ij*4 + 0]; A[1] = C[k][ij*4 + 1];
                        A[2] = C[k][ij*4 + 2]; A[3] = C[k][ij*4 + 3];
                        CvMat matA = cvMat(2, 2, CV_64FC1, A);

                        cvEigenVV(&matA, &evects, &evals, DBL_EPSILON);
                        double e1 = cvmGet(&evals,0,0);
                        double e2 = cvmGet(&evals,1,0);
					
                        bool l = false;

                        if (e2/e1<a) { 
                            // std::cout << "lambda<: " << e2/e1 << std::endl;
                            e2 = e1*a;
                            l = true;
                        }
                        cvmSet(&matD, 0, 0, 1.0/e1); cvmSet(&matD, 0, 1, 0.0);
                        cvmSet(&matD, 1, 1, 1.0/e2); cvmSet(&matD, 1, 0, 0.0);
                        // E*D*E' 
                        cvGEMM(&evects,&matD,1,NULL,0,&matT1,0);
                        cvGEMM(&matT1,&evects,1,NULL,0,&matT2,CV_GEMM_B_T);

                        // Calculate the inverse of the covariance matrix
                        // We use the formula C^-1 = 1/det(C) * [c22 -c12; -c21 c11]
                        // where det(C) is the determinant 
                        I[k][ij*4 + 0] = cvmGet(&matT2, 0, 0);
                        I[k][ij*4 + 1] = cvmGet(&matT2, 0, 1);
                        I[k][ij*4 + 2] = cvmGet(&matT2, 1, 0);
                        I[k][ij*4 + 3] = cvmGet(&matT2, 1, 1);

                        if (l && false) {
                            int p = std::cout.precision(20);
                            std::cout << "C: " << C[k][ij*4 + 0] << " "
                                      << C[k][ij*4 + 1] << " "
                                      << C[k][ij*4 + 2] << " "
                                      << C[k][ij*4 + 3] << std::endl;

                            std::cout << "e: " << cvmGet(&evals,0,0) << " "
                                      << cvmGet(&evals,1,0) << std::endl;

                            std::cout << "E: " << cvmGet(&evects, 0, 0) << " " 
                                      << cvmGet(&evects, 0, 1) << " " 
                                      << cvmGet(&evects, 1, 0) << " " 
                                      << cvmGet(&evects, 1, 1) << " " << std::endl;
                            std::cout << "I: " << I[k][ij*4 + 0] << " "
                                      << I[k][ij*4 + 1] << " "
                                      << I[k][ij*4 + 2] << " "
                                      << I[k][ij*4 + 3] << std::endl;
                            std::cout.precision(p);
                        }

                        // Calculate the square root factor
                        // int retval = dlevmar_chol(&(I[k][ij*4]), &(Q[k][ij*4]), 2);
                    }
                }
            }
        }
    }

    NDT::~NDT()
    {
        for (int i=0; i<4; i++) {
            delete N[i];
            delete U[i];
            delete C[i];
            delete I[i];
            delete Q[i];
        }
    }

    // Calculates the score of the provided point set
    // All the points are transformed by the specified pose
    double NDT::calculateScore(const std::vector<Point2d> & points, 
                               const Pose2d & pose) const
    {
        double score = 0;
        std::vector<Point2d>::const_iterator p;
        for (p = points.begin();p < points.end(); p++) {
            Point2d x = pose.transform_from(*p);
            score+=evaluate(x.x(), x.y());
        }
        return score;
    }

    // Returns the score at the specified point
    double NDT::evaluate(double x, double y) const
    {
        double score = 0.0;
        double minX, maxX, minY, maxY;

        minX = R[0]; maxX = R[1];	
        minY = R[2]; maxY = R[3];

        int W = ceil((maxX-minX)/cellSize); // Max column index
        int H = ceil((maxY-minY)/cellSize); // Max row index
        double width = W*cellSize;
        double height = H*cellSize;
        double ix,iy;
        int ij; // Calculated offset into the array

        ix = (x - minX)/width*W;
        iy = (y - minY)/height*H;
        double a = 2.0 * fabs((ix)-(int)((ix))-0.5);
        double b = 2.0 * fabs((iy)-(int)((iy))-0.5);
        double w;

        for (int i=0; i<4; i++) {
            double ox=0.0, oy=0.0;
            if (i == 1) {ox = cellSize/2.0; oy = 0.0;}
            else if (i == 2) {oy = cellSize/2.0; ox = 0.0;}
            else if (i == 3) {ox = oy =  cellSize/2.0;}

            if (i==0) w = (1.0-a)*(1.0-b);
            else if (i==1) w = a*(1.0-b);
            else if (i==2) w = (1.0-a)*b;
            else if (i==3) w = a*b;

            ix = (x - minX + ox)/width*W;
            iy = (y - minY + oy)/height*H;

            //		ij = ((int)(iy+oy))*(W+1) + (int)(ix+ox);
            ij = ((int)(iy))*(W+1) + (int)(ix);
            if (ij >= 0 && ij < (W+1)*(H+1) && N[i][ij]>4) 
                {
                    double m[2];
                    m[0] = x-U[i][ij*2]; 
                    m[1] = y-U[i][ij*2 + 1]; 
	
                    // Evaluate the normal distribution at point x,y
                    // exp(-([x y]-U')I([x y]-U')')
                    score += w*exp(-( (m[0]*I[i][ij*4]     + m[1]*I[i][ij*4 + 2])*m[0]
                                      +(m[0]*I[i][ij*4 + 1] + m[1]*I[i][ij*4 + 3])*m[1]));
                }
        }
        return score;
    }

    void NDT::ndt_fun(double *p, double *hx, int m, int n, double* points)
    {
        // apply transformation to the points
	
        double x = 0.0;
        double y = 0.0;

        double ct = cos(p[2]);
        double st = sin(p[2]);

        // TODO: this can be precomputed for the transform
        double minX, maxX, minY, maxY;
        minX = R[0]; maxX = R[1];	
        minY = R[2]; maxY = R[3];
        int W = ceil((maxX-minX)/cellSize); // Max column index
        int H = ceil((maxY-minY)/cellSize); // Max row index
        double width = W*cellSize;
        double height = H*cellSize;	

        //std::cout << "NDT::ndt_fun: " << n << " " << m << std::endl;
        //std::cout << "NDT::ndt_fun: " << p[0] << " " << p[1] << " " << p[2] << std::endl;

        // evaluate the cost for each point
        for (int k=0; k<n/2; k++)
            {
                // Transform the point using p
                x = ct*points[2*k] - st*points[2*k + 1] + p[0];	
                y = st*points[2*k] + ct*points[2*k + 1] + p[1];	
                //std::cout << "::" << x << " " << y << " " << minX << " " << maxX <<
                //				" " << minY << " " << maxY << "::";

                hx[2*k]     = 0.0;
                hx[2*k + 1] = 0.0;

                // Check if points is out of bounds
                if (x<minX || x>maxX || y<minY || y>maxY) {
                    continue;
                }

                double ix,iy;
                int ij; // Calculated offset into the array
                ix = (x - minX)/width*W;
                iy = (y - minY)/height*H;
                // std::cout << " " << ix << " - " << iy;

                double a = 2.0 * fabs((ix)-(int)((ix))-0.5);
                double b = 2.0 * fabs((iy)-(int)((iy))-0.5);
                double w;

                //bool isInCell = false;			// True if points hits a cell with any value

                for (int i=0; i<4; i++) {
                    double ox=0.0, oy=0.0;
                    if (i == 1) {ox = cellSize/2.0; oy = 0.0;}
                    else if (i == 2) {oy = cellSize/2.0; ox = 0.0;}
                    else if (i == 3) {ox = oy =  cellSize/2.0;}
                    if (i==0) w = (1.0-a)*(1.0-b);
                    else if (i==1) w = a*(1.0-b);
                    else if (i==2) w = (1.0-a)*b;
                    else if (i==3) w = a*b;
		
                    ix = (x - minX + ox)/width*W;
                    iy = (y - minY + oy)/height*H;
                    ij = ((int)(iy))*(W+1) + (int)(ix);
                    //			ij = ((int)(iy+oy))*(W+1) + (int)(ix+ox);
                    // int S = (W+1)*(H+1);
                    // std::cout << ":" << N[i][ij] << " " << ij << " " << S << ":";
                    if (N[i][ij]>4 && ij >= 0 && ij < (W+1)*(H+1)) 
                        {
                            double m[2];
                            m[0] = x-U[i][ij*2]; 
                            m[1] = y-U[i][ij*2 + 1]; 

                            // This can be precomputed
                            hx[2*k]   += w*(m[0] * Q[i][ij*4]);
                            hx[2*k+1] += w*(m[0] * Q[i][ij*4+1] + m[1] * Q[i][ij*4+3]); 

                            //isInCell = true;
                            /*
                              double d2 = hx[2*k]*hx[2*k] + hx[2*k+1]*hx[2*k+1];
                              if (d2>1E-6) {
                              double b2 = 0.5*0.5;
                              double c = sqrt(b2*log(1.0+d2/b2)/d2);
                              //std::cout << " c=" << c << " ";
                              hx[2*k]			*= c;
                              hx[2*k + 1] *= c;
                              }
                            */
                        } 
			
                    // This is if we want to give penalty for points in empty cells
                    //if (!isInCell) {
                    //	hx[2*k]   += 0.25;
                    //	hx[2*k+1] += 0.25; 
                    //}
			
                }
            }
        //for (int k=0; k<n; k++) {
        //	std::cout << hx[k] << ",";
        //}
        //std::cout << std::endl;

    }

    typedef struct _NDTCall 
    {
        NDT* ndt;
        double* points;
    } NDTCall;

    // Needs to be a friend??
    void ndt_fun(double *p, double *hx, int m, int n, void *adata)
    {
        NDTCall* ndtcall = (NDTCall*)adata;
        ndtcall->ndt->ndt_fun(p, hx, m, n, ndtcall->points);
        //NDT ndt = (NDT*) adata;
        // TRANSFORM points hx by p
        // return 0.0;
    }

    // Calculates the transformation from this scan to the
    // scan provided
    void NDT::findTransformation(std::vector<Point2d> & points, 
                                 Pose2d initial, 
                                 ScanMatch & matchResult,
                                 int itmax)
    {
        NDTCall ndtcall;
        ndtcall.ndt = this;
        ndtcall.points = new double[2*points.size()];

        for (unsigned int i=0; i<points.size();i++) {
            ndtcall.points[2*i] = points[i].x();
            ndtcall.points[2*i + 1] = points[i].y();
        }

        //void* adata = &ndtcall;
        double p[3]; // x,y,th
        p[0] = initial.x(); p[1] = initial.y(); p[2] = initial.t();

        // double* x; // contains x,y of all the points
        // x = NULL;

        //int m = 3; // 3 unknowns, x,y,th
        //int n = 2*points.size();	// number of points
        //int itmax = 10; //0;
        //double* opts = NULL;
        //                  mu  ||J^T e||  ||Dp||_2  ||e||_2   delta
        //double opts[5] = {1E-3,  1E-4,      1E-4,     1.0,     -1E-6};	
        double info[LM_INFO_SZ];
        //double* work = NULL;
        double covar[9];

        //int iter = dlevmar_dif(::hauv::ndt_fun,
        //                       p, x, m, n, itmax, opts, info, work, covar, adata); 

        //std::cout << "NDT::findTransformation: " << iter << " " << points.size() << " "
        //          << p[0] << " " << p[1] << " " << p[2] << std::endl;
        //std::cout << "NDT::findTransformation.info ";
        //for (int i=0;i<10;i++) {
        //	std::cout << i << ": " << info[i] << " ";
        //}
        //std::cout << std::endl;
        matchResult.pose.set(p[0], p[1], p[2]);
        matchResult.scoreStart = info[1]; 
        matchResult.score = info[1];
        std::copy(covar, covar+9, &matchResult.covariance[0][0]);

        delete [] ndtcall.points;
    }

}
