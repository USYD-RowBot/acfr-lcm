#ifndef _HAUV_NDT_H
#define _HAUV_NDT_H

#include <vector>
#include <isam/isam.h>

/**
 * @file ndt.h
 * @brief Scanmatching using the NDT algorithm
 * @author Hordur Johannsson
 * @version $Id:  $
 */

namespace hauv {

    /**
     * Store information about a match between two scans
     */
    class ScanMatch
    {
      public:
        isam::Pose2d pose;
        // covariance
        double covariance[3][3];
        double scoreStart;
        double score;
    };

    /**
     * This class implements a 2D scan registration method
     * using NDT.
     */ 
    class NDT
    {
      public:
        /**
         * Constructs a new NDT scan with a given set of points.
         */
        NDT(const std::vector<isam::Point2d> & p, double cellSize = 1.0, double minE = 0.001);
        ~NDT();

        // Returns the score at the specified point
        double evaluate(double x, double y) const;

        // Calculates the transformation from this scan to the
        // scan provided
        void findTransformation(std::vector<isam::Point2d> & p,
                                isam::Pose2d initial,
                                ScanMatch & matchResult, int itmax=10 );

        // Calculates the score of the provided point set
        // All the points are transformed by the specified pose
        double calculateScore(const std::vector<isam::Point2d> & points,
                              const isam::Pose2d & pose) const;

        const double* getRange() const { return R; }

        // Private or protected?
        void ndt_fun(double *p, double *hx, int m, int n, double* points);
	
      private:
        // Stores the count in each cell in each overlapping
        int* N[4];

        // Stores the mean for each cell in each overlapping
        double* U[4];

        // Stores the covariance for each cell in each overlapping
        double* C[4];

        // Stores the information matrix for each cell in each overlapping
        double* I[4];

        // Stores the square root of the information matrix for each cell in each overlapping
        double* Q[4];

        // Stores the boundary information for each grid
        double R[4];

        double cellSize;

        double minEigenVRatio; // The minimum ratio of the two eigenvalues
    };

}

#endif
