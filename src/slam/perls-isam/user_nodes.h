/**
 * @file user_nodes.h
 * @brief User defined nodes
 *
 */
#include <isam/isam.h>

/* NOTE: When you add a new node, modify gparser.cpp/h to enable proper graph loading */
/* Unfortunately, currently no loading from native isam is supported. */

#pragma once
typedef Eigen::Matrix< double , 6 , 1> Vector6d;

namespace isam {

    class Norm3d {
        friend std::ostream& operator<< (std::ostream& out, const Norm3d& n) {
            n.write (out);
            return out;
        }

        double _nx;             /* normal in x direction */
        double _ny;             /* normal in y direction */
        double _nz;             /* normal in z direction */

    public:
        static const int dim = 3;
        static const char* name () {
            return "Bearning3d";
        }
        Norm3d (): _nx (0.), _ny (0.), _nz (0.) {}
        Norm3d (double nx, double ny, double nz) : _nx (nx), _ny (ny), _nz (nz) {}
        Norm3d (const Eigen::VectorXd& vec) : _nx (vec (0)), _ny (vec (1)), _nz (vec (2)) {}
        double nx () const {return _nx;}
        double ny () const {return _ny;}
        double nz () const {return _nz;}

        Norm3d exmap(const Eigen::Vector3d& delta) {
            Norm3d res = *this;
            res._nx += delta(0);
            res._ny += delta(1);
            res._nz += delta(2);
            return res;
        }

        Eigen::VectorXd vector () const {
            Eigen::VectorXd tmp (this->dim);
            tmp << _nx, _ny, _nz;
            return tmp;
        }
        void set (double nx, double ny, double nz) {
            _nx = nx;
            _ny = ny;
            _nz = nz;
        }
        void set (const Eigen::VectorXd& v) {
            _nx = v(0);
            _ny = v(1);
            _nz = v(2);
        }
        
        void write (std::ostream &out) const {
            out << "(" << _nx << ", " << _ny << ", " << _nz << ")";
        }

    }; /* class Norm3d */

    /**
     * Like a Point3d but with an associated normal
     */
    class Plane3d {
        friend std::ostream& operator<< (std::ostream& out, const Plane3d& s) {
            s.write (out);
            return out;
        }

        Norm3d _n;                  /* outward-facing normal to the plane */
        Point3d _p;                 /* point anywhere on plane */
    public:
        static const int dim = 6;
        static const char* name() {
            return "Plane3d";
        }

        Plane3d () {}
        Plane3d (double nx, double ny, double nz, double px, double py, double pz) : _n (nx, ny, nz), _p (px, py, pz) {}
        Plane3d (const Eigen::VectorXd& vec) : _n (vec (0), vec (1), vec (2)), _p (vec (3), vec (4), vec (5)) {}

        double nx () const {return _n.nx ();}
        double ny () const {return _n.ny ();}
        double nz () const {return _n.nz ();}
        double px () const {return _p.x ();}
        double py () const {return _p.y ();}
        double pz () const {return _p.z ();}

        Norm3d normal () const {return _n;}
        Point3d point () const {return _p;}

        Plane3d exmap(const Vector6d& delta) {
            /* std::cout << "IN EXMAP" << std::endl; */
            Plane3d res = *this;
            Eigen::VectorXd normExmap (Norm3d::dim);
            Eigen::VectorXd pointExmap (Point3d::dim);

            for (int i=0; i<Norm3d::dim; i++)
                normExmap(i) = delta(i);
            for (int i=0; i<Norm3d::dim; i++)
                pointExmap(i) = delta(i+3);

            /* std::cout << "EXMAP 1" << normExmap << std::endl; */
            /* std::cout << "EXMAP 2" << pointExmap << std::endl; */

            res._n = _n.exmap (normExmap);
            res._p = _p.exmap (pointExmap);
            return res;
        }

        Eigen::VectorXd vector () const {
            Eigen::VectorXd tmp (this->dim);
            tmp << nx (), ny (), nz (), px (), py (), pz ();
            return tmp;
        }

        void set (double nx, double ny, double nz, double px, double py, double pz) {
            _n = Norm3d (nx, ny, nz);
            _p = Point3d (px, py, pz);
        }
        void set (const Eigen::VectorXd& v) {
            _n = Norm3d (v (0), v(1), v(2));
            _p = Point3d (v (3), v (4), v (5));
        }

        void write (std::ostream &out) const {
            out << "(" << nx () << ", " << ny () << ", " << nz () << "; " << px () << ", " << py () << ", " << pz () << ")";
        }

    }; /* class Plane3d */

    class Plane3dMeasurement {
        friend std::ostream& operator<< (std::ostream& out, const Plane3dMeasurement& s) {
            s.write (out);
            return out;
        }

        Norm3d _n;                  /* outward-facing normal to the plane */
        double _d;                  /* orthogonal distance to surface */
    public:
        static const int dim = 4;
        static const char* name() {
            return "Plane3dMeasurement";
        }

        Plane3dMeasurement () {}
        Plane3dMeasurement (double nx, double ny, double nz, double d) : _n (nx, ny, nz), _d (d) {}
        Plane3dMeasurement (const Eigen::VectorXd& vec) : _n (vec (0), vec (1), vec (2)), _d (vec (3)) {}

        double nx () const {return _n.nx ();}
        double ny () const {return _n.ny ();}
        double nz () const {return _n.nz ();}

        Norm3d normal () const {return _n;}
        double d () const {return _d;}

        Eigen::VectorXd vector () const {
            Eigen::VectorXd tmp (this->dim);
            tmp << nx (), ny (), nz (), d ();
            return tmp;
        }

        void set (double nx, double ny, double nz, double d) {
            _n = Norm3d (nx, ny, nz);
            _d = d;
        }
        void set (const Eigen::VectorXd& v) {
            _n = Norm3d (v (0), v (1), v (2));
            _d = v (3);
        }

        void write (std::ostream &out) const {
            out << "(" << nx () << ", " << ny () << ", " << nz () << ", " << d () << ")";
        }

    }; /* class Plane3dMeasurement */

    typedef NodeT<Plane3d> Plane3d_Node;

} /* namespace isam */
